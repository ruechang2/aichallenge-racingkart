"""Detect and track other karts from a 2D LaserScan.

This is the LiDAR-side counterpart of :mod:`v2x_vehicle_tracker`. It exists
so the MPC can avoid other vehicles using *only* what the end-to-end student
(TinyLidarNet) can also see. V2X gives positions the student has no access
to, so demonstrations recorded with V2X-based avoidance are not imitable:
the student sees a scan and no reason to swerve.

Like the V2X tracker this module is pure Python/NumPy with no rclpy import,
so it can be unit-tested and replayed offline against rosbag scans.

Pipeline per scan:

  1. polar ranges -> Cartesian points in the sensor frame
  2. split into clusters of angularly adjacent points
  3. drop clusters that are not kart-shaped (too few points / too large)
  4. transform cluster centers into the map frame
  5. drop centers that sit on the static map (walls, curbs)
  6. associate with existing tracks and finite-difference the velocity

Steps 3 and 5 are what separates "another kart" from "the wall". Step 3
alone is not enough (a wall seen edge-on is short), and step 5 alone is not
enough either (a kart driving close to the wall lands on occupied cells).
"""

import math
from collections import deque
from typing import Callable, Deque, Dict, List, Optional, Sequence, Tuple

import numpy as np


class Detection:
    """One kart-shaped cluster, in map coordinates.

    Attributes:
        x, y: cluster center [m] in the map frame.
        extent: bounding-box diagonal of the cluster [m]. Kept for logging —
            it is the number that says why something was accepted or rejected.
        n_points: how many scan returns formed the cluster.
    """

    __slots__ = ("x", "y", "extent", "n_points")

    def __init__(self, x: float, y: float, extent: float, n_points: int):
        self.x = float(x)
        self.y = float(y)
        self.extent = float(extent)
        self.n_points = int(n_points)

    def __repr__(self) -> str:  # pragma: no cover - debugging aid
        return (f"Detection(x={self.x:.2f}, y={self.y:.2f}, "
                f"extent={self.extent:.2f}, n={self.n_points})")


def scan_to_points(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    min_range: float,
    max_range: float,
) -> Tuple[np.ndarray, np.ndarray]:
    """Convert polar ranges to Cartesian points in the sensor frame.

    Args:
        ranges: raw LaserScan ranges [m]. NaN/Inf are treated as "no return".
        angle_min: angle of the first beam [rad].
        angle_increment: angular step between beams [rad].
        min_range: returns closer than this are discarded (self-returns).
        max_range: returns beyond this are discarded. Far returns are too
            sparse to cluster reliably, and the MPC horizon is short.

    Returns:
        (points, beam_indices) where ``points`` is (N, 2) float64 in the
        sensor frame (x forward, y left) and ``beam_indices`` holds the
        originating beam index of each point, ascending.
    """
    r = np.asarray(ranges, dtype=np.float64)
    if r.size == 0:
        return np.empty((0, 2), dtype=np.float64), np.empty(0, dtype=np.int64)

    beams = np.arange(r.size, dtype=np.int64)
    angles = angle_min + beams * angle_increment
    valid = np.isfinite(r) & (r >= min_range) & (r <= max_range)

    r_v = r[valid]
    a_v = angles[valid]
    points = np.stack([r_v * np.cos(a_v), r_v * np.sin(a_v)], axis=1)
    return points, beams[valid]


def cluster_index_ranges(
    points: np.ndarray,
    beam_indices: np.ndarray,
    gap_distance: float,
    max_beam_gap: int = 2,
) -> List[Tuple[int, int]]:
    """Split the scan into clusters, as ``[start, end)`` index ranges.

    Ranges rather than copies, because the isolation test in
    :meth:`LidarVehicleTracker.detect` needs to look at the points on either
    side of a cluster.

    Args:
        points: (N, 2) sensor-frame points, ordered by beam index.
        beam_indices: beam index of each point.
        gap_distance: a jump larger than this [m] between consecutive points
            starts a new cluster.
        max_beam_gap: how many missing beams may sit inside one cluster.
            Dropouts are common on dark/edge-on surfaces, and splitting on
            every single dropout shatters one kart into several clusters.
    """
    if len(points) == 0:
        return []

    ranges: List[Tuple[int, int]] = []
    start = 0
    for i in range(1, len(points)):
        beam_gap = int(beam_indices[i] - beam_indices[i - 1])
        step = float(np.hypot(*(points[i] - points[i - 1])))
        if beam_gap > max_beam_gap or step > gap_distance:
            ranges.append((start, i))
            start = i
    ranges.append((start, len(points)))
    return ranges


def cluster_points(
    points: np.ndarray,
    beam_indices: np.ndarray,
    gap_distance: float,
    max_beam_gap: int = 2,
) -> List[np.ndarray]:
    """Group angularly adjacent points that are close to each other.

    Returns:
        List of (M, 2) arrays, one per cluster, in scan order.
    """
    return [points[a:b] for a, b in cluster_index_ranges(
        points, beam_indices, gap_distance, max_beam_gap)]


def cluster_extent(cluster: np.ndarray) -> float:
    """Bounding-box diagonal of a cluster [m]."""
    if len(cluster) == 0:
        return 0.0
    span = cluster.max(axis=0) - cluster.min(axis=0)
    return float(np.hypot(span[0], span[1]))


def is_isolated(
    points: np.ndarray,
    start: int,
    end: int,
    isolation_gap: float,
) -> bool:
    """True if the cluster stands free of whatever surrounds it in the scan.

    A kart is separated from the scenery by a large range step on both sides.
    A piece of wall that got split off by a dropout is not: its neighbours
    continue at almost the same range. Without this test the wall keeps
    producing kart-sized fragments wherever the returns break up, and each
    fragment becomes a phantom obstacle in the MPC corridor.

    The ends of the scan count as isolated: a kart running beside us is cut
    by the 180-degree field of view, and that is exactly the case this whole
    module exists for.
    """
    if start > 0:
        step = float(np.hypot(*(points[start] - points[start - 1])))
        if step <= isolation_gap:
            return False
    if end < len(points):
        step = float(np.hypot(*(points[end] - points[end - 1])))
        if step <= isolation_gap:
            return False
    return True


def sensor_to_map(
    point_xy: Tuple[float, float],
    pose_xy_yaw: Tuple[float, float, float],
    sensor_offset_x: float,
    sensor_offset_y: float = 0.0,
) -> Tuple[float, float]:
    """Transform a sensor-frame point into the map frame.

    Args:
        point_xy: (x, y) in the sensor frame.
        pose_xy_yaw: (x, y, yaw) of base_link in the map frame.
        sensor_offset_x: sensor x position in base_link [m]. The kart's LiDAR
            sits well ahead of the rear axle, so ignoring this biases every
            detection by that distance.
        sensor_offset_y: sensor y position in base_link [m].
    """
    px, py, yaw = pose_xy_yaw
    lx = point_xy[0] + sensor_offset_x
    ly = point_xy[1] + sensor_offset_y
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
    return (px + cos_y * lx - sin_y * ly, py + sin_y * lx + cos_y * ly)


class LidarVehicleTracker:
    """Turns LaserScans into tracked, velocity-carrying vehicle detections.

    The public shape mirrors :class:`v2x_vehicle_tracker.V2XVehicleTracker`
    (``update`` / ``predict_all`` / ``active_vehicle_ids``) so the controller
    can feed either source into ``predictions_to_obstacles`` unchanged.
    """

    def __init__(
        self,
        vehicle_radius: float,
        min_range: float = 0.5,
        max_range: float = 12.0,
        cluster_gap: float = 0.5,
        min_points: int = 3,
        max_extent: float = 2.5,
        isolation_gap: float = 1.0,
        association_radius: float = 2.0,
        track_timeout: float = 0.5,
        v_max_safety: float = 30.0,
        sensor_offset_x: float = 1.65,
        sensor_offset_y: float = 0.0,
        is_static: Optional[Callable[[float, float], bool]] = None,
        warn_callback=None,
    ):
        """Initializes the tracker.

        Args:
            vehicle_radius: radius of the circle used to represent another
                kart [m]. Also used to push the cluster center back from the
                visible surface to the kart's center.
            min_range: ignore returns closer than this [m].
            max_range: ignore returns beyond this [m]. Far returns are too
                sparse to tell a kart from a piece of wall, and the MPC
                horizon is 0.5 s, so they cannot change the plan anyway.
            cluster_gap: distance between consecutive returns that starts a
                new cluster [m].
            min_points: clusters with fewer returns are treated as noise.
            max_extent: clusters whose bounding-box diagonal exceeds this [m]
                are walls/curbs, not karts.
            isolation_gap: a cluster counts as free-standing only if the range
                steps to its neighbours on both sides exceed this [m].
            association_radius: a detection joins an existing track if it is
                within this distance of it [m].
            track_timeout: tracks not matched for this long are dropped [s].
            v_max_safety: velocities above this magnitude are treated as an
                association error and zeroed [m/s].
            sensor_offset_x: LiDAR x position in base_link [m].
            sensor_offset_y: LiDAR y position in base_link [m].
            is_static: optional ``(x, y) -> bool`` in map coordinates, true
                where the static map is occupied. Detections there are walls.
            warn_callback: optional ``(str) -> None`` for warnings.
        """
        self.vehicle_radius = float(vehicle_radius)
        self.min_range = float(min_range)
        self.max_range = float(max_range)
        self.cluster_gap = float(cluster_gap)
        self.min_points = int(min_points)
        self.max_extent = float(max_extent)
        self.isolation_gap = float(isolation_gap)
        self.association_radius = float(association_radius)
        self.track_timeout = float(track_timeout)
        self.v_max_safety = float(v_max_safety)
        self.sensor_offset_x = float(sensor_offset_x)
        self.sensor_offset_y = float(sensor_offset_y)
        self._is_static = is_static
        self._warn = warn_callback if warn_callback is not None else (lambda _msg: None)

        self._samples: Dict[str, Deque[Tuple[float, float, float]]] = {}
        self._velocities: Dict[str, Tuple[float, float]] = {}
        self._last_seen: Dict[str, float] = {}
        self._active: List[str] = []
        self._next_id = 0
        # 直近フレームで採用/棄却した数。ログでしきい値を詰めるための材料。
        self.last_detections: List[Detection] = []
        self.last_rejected_static = 0
        self.last_rejected_shape = 0

    def detect(
        self,
        ranges: Sequence[float],
        angle_min: float,
        angle_increment: float,
        pose_xy_yaw: Tuple[float, float, float],
    ) -> List[Detection]:
        """Runs one scan through clustering and filtering (no tracking)."""
        points, beams = scan_to_points(
            ranges, angle_min, angle_increment, self.min_range, self.max_range)
        index_ranges = cluster_index_ranges(points, beams, self.cluster_gap)

        detections: List[Detection] = []
        rejected_shape = 0
        rejected_static = 0
        for start, end in index_ranges:
            cluster = points[start:end]
            if len(cluster) < self.min_points:
                rejected_shape += 1
                continue
            extent = cluster_extent(cluster)
            if extent > self.max_extent:
                rejected_shape += 1
                continue
            if not is_isolated(points, start, end, self.isolation_gap):
                rejected_shape += 1
                continue

            centroid = cluster.mean(axis=0)
            # LiDAR が見ているのは相手の手前の面だけなので、重心は車体の
            # 中心より手前に寄る。視線方向へ半径分だけ押し戻して中心に直す。
            norm = float(np.hypot(centroid[0], centroid[1]))
            if norm > 1e-6:
                centroid = centroid * (1.0 + self.vehicle_radius / norm)

            x, y = sensor_to_map(
                (float(centroid[0]), float(centroid[1])), pose_xy_yaw,
                self.sensor_offset_x, self.sensor_offset_y)

            if self._is_static is not None and self._is_static(x, y):
                rejected_static += 1
                continue

            detections.append(Detection(x, y, extent, len(cluster)))

        self.last_detections = detections
        self.last_rejected_shape = rejected_shape
        self.last_rejected_static = rejected_static
        return detections

    def update(
        self,
        now: float,
        ranges: Sequence[float],
        angle_min: float,
        angle_increment: float,
        pose_xy_yaw: Tuple[float, float, float],
    ) -> None:
        """Detects on one scan and associates the result with existing tracks.

        Args:
            now: monotonic timestamp of the scan [s].
            ranges: LaserScan ranges [m].
            angle_min: angle of the first beam [rad].
            angle_increment: angular step between beams [rad].
            pose_xy_yaw: (x, y, yaw) of base_link in the map frame.
        """
        detections = self.detect(ranges, angle_min, angle_increment, pose_xy_yaw)
        self._expire_tracks(now)

        unmatched = list(detections)
        active: List[str] = []
        # 近いものから貪欲に対応づける。相手は最大 3 台なので、これで十分。
        for track_id in self._tracks_by_recency():
            if not unmatched:
                break
            _t, tx, ty = self._samples[track_id][-1]
            best_i = min(
                range(len(unmatched)),
                key=lambda i: math.hypot(unmatched[i].x - tx, unmatched[i].y - ty))
            best = unmatched[best_i]
            if math.hypot(best.x - tx, best.y - ty) > self.association_radius:
                continue
            unmatched.pop(best_i)
            self._append_sample(track_id, now, best.x, best.y)
            active.append(track_id)

        for det in unmatched:
            track_id = f"lidar_{self._next_id}"
            self._next_id += 1
            self._samples[track_id] = deque(maxlen=2)
            self._append_sample(track_id, now, det.x, det.y)
            active.append(track_id)

        self._active = active

    def velocity(self, vehicle_id: str) -> Tuple[float, float]:
        return self._velocities.get(vehicle_id, (0.0, 0.0))

    def predict_positions(
        self, vehicle_id: str, t_samples
    ) -> List[Tuple[float, float]]:
        buf = self._samples.get(vehicle_id)
        if not buf:
            return []
        _t_last, x_last, y_last = buf[-1]
        vx, vy = self._velocities.get(vehicle_id, (0.0, 0.0))
        return [(x_last + vx * t, y_last + vy * t) for t in t_samples]

    def active_vehicle_ids(self) -> List[str]:
        return list(self._active)

    def predict_all(self, t_samples) -> Dict[str, List[Tuple[float, float]]]:
        return {vid: self.predict_positions(vid, t_samples) for vid in self._active}

    def confirmed_positions(self, min_samples: int = 2) -> List[Tuple[float, float]]:
        """Latest position of each track seen on at least ``min_samples`` scans.

        One-frame detections are the ones that turn out to be wall fragments,
        and a phantom obstacle handed to the planner costs a lap. Requiring a
        second sighting drops them at the price of one scan of latency.
        """
        out: List[Tuple[float, float]] = []
        for vid in self._active:
            buf = self._samples.get(vid)
            if buf is None or len(buf) < min_samples:
                continue
            _t, x, y = buf[-1]
            out.append((x, y))
        return out

    def _tracks_by_recency(self) -> List[str]:
        return sorted(self._last_seen, key=lambda k: self._last_seen[k], reverse=True)

    def _append_sample(self, track_id: str, now: float, x: float, y: float) -> None:
        buf = self._samples[track_id]
        buf.append((now, x, y))
        self._last_seen[track_id] = now

        if len(buf) < 2:
            self._velocities[track_id] = (0.0, 0.0)
            return
        t0, x0, y0 = buf[0]
        t1, x1, y1 = buf[1]
        dt = t1 - t0
        if dt <= 0.0:
            self._velocities[track_id] = (0.0, 0.0)
            return
        vx = (x1 - x0) / dt
        vy = (y1 - y0) / dt
        if math.hypot(vx, vy) > self.v_max_safety:
            # 対応づけを間違えた徴候。速度を信じて外挿すると、ありもしない
            # 障害物をコース前方にばらまくことになるので 0 に倒す。
            self._velocities[track_id] = (0.0, 0.0)
            self._warn(
                f"LiDAR: velocity for track '{track_id}' exceeds "
                f"{self.v_max_safety} m/s — clamped to zero")
        else:
            self._velocities[track_id] = (vx, vy)

    def _expire_tracks(self, now: float) -> None:
        stale = [k for k, t in self._last_seen.items()
                 if (now - t) > self.track_timeout]
        for k in stale:
            self._samples.pop(k, None)
            self._velocities.pop(k, None)
            self._last_seen.pop(k, None)
