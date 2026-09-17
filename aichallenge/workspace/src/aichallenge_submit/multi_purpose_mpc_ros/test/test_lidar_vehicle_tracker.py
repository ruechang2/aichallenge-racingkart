"""Unit tests for LidarVehicleTracker (pure Python, no rclpy)."""

import math
from typing import List, Optional, Tuple

import numpy as np
import pytest

from multi_purpose_mpc_ros.lidar_vehicle_tracker import (
    LidarVehicleTracker,
    cluster_extent,
    cluster_index_ranges,
    cluster_points,
    is_isolated,
    scan_to_points,
    sensor_to_map,
)


ANGLE_MIN = -math.pi / 2
ANGLE_INCREMENT = math.pi / 180.0  # 1 deg, 181 beams over the front 180 deg
N_BEAMS = 181


def _empty_scan(fill: float = float('inf')) -> List[float]:
    return [fill] * N_BEAMS


def _beam_of(angle_deg: float) -> int:
    return int(round((math.radians(angle_deg) - ANGLE_MIN) / ANGLE_INCREMENT))


def _put_object(ranges: List[float], angle_deg: float, distance: float,
                width_deg: float) -> None:
    """Paints a constant-range return over a span of beams."""
    half = int(round(width_deg / 2.0))
    center = _beam_of(angle_deg)
    for b in range(center - half, center + half + 1):
        if 0 <= b < len(ranges):
            ranges[b] = distance


def _tracker(**kwargs) -> LidarVehicleTracker:
    defaults = dict(
        vehicle_radius=0.5,
        min_range=0.5,
        max_range=20.0,
        cluster_gap=0.5,
        min_points=3,
        max_extent=2.5,
        association_radius=2.0,
        track_timeout=0.5,
        v_max_safety=30.0,
        sensor_offset_x=0.0,
    )
    defaults.update(kwargs)
    return LidarVehicleTracker(**defaults)


def test_scan_to_points_drops_out_of_range_and_non_finite():
    ranges = [float('nan'), float('inf'), 0.1, 5.0, 50.0]
    points, beams = scan_to_points(
        ranges, angle_min=0.0, angle_increment=0.1, min_range=0.5, max_range=20.0)
    assert len(points) == 1
    assert beams.tolist() == [3]
    assert points[0][0] == pytest.approx(5.0 * math.cos(0.3))


def test_cluster_points_splits_on_distance_jump():
    points = np.array([[1.0, 0.0], [1.05, 0.0], [3.0, 0.0], [3.05, 0.0]])
    beams = np.array([0, 1, 2, 3])
    clusters = cluster_points(points, beams, gap_distance=0.5)
    assert [len(c) for c in clusters] == [2, 2]


def test_cluster_points_tolerates_small_beam_dropouts():
    points = np.array([[1.0, 0.0], [1.05, 0.0], [1.1, 0.0]])
    beams = np.array([0, 2, 3])  # one missing beam inside the cluster
    assert len(cluster_points(points, beams, gap_distance=0.5)) == 1

    beams_wide = np.array([0, 5, 6])  # too many missing beams
    assert len(cluster_points(points, beams_wide, gap_distance=0.5)) == 2


def test_sensor_to_map_applies_offset_and_yaw():
    # Vehicle at (10, 5) facing +y; a point 2 m ahead of a sensor mounted 1.65 m
    # ahead of base_link must land 3.65 m along +y.
    x, y = sensor_to_map((2.0, 0.0), (10.0, 5.0, math.pi / 2), sensor_offset_x=1.65)
    assert x == pytest.approx(10.0)
    assert y == pytest.approx(8.65)


def test_kart_sized_cluster_is_detected_at_its_center():
    ranges = _empty_scan()
    # A kart 5 m ahead spanning ~14 deg (~1.2 m wide at that distance).
    _put_object(ranges, angle_deg=0.0, distance=5.0, width_deg=14.0)

    tracker = _tracker()
    detections = tracker.detect(ranges, ANGLE_MIN, ANGLE_INCREMENT, (0.0, 0.0, 0.0))

    assert len(detections) == 1
    # The visible surface is at 5 m; the center is pushed back by vehicle_radius.
    assert detections[0].x == pytest.approx(5.5, abs=0.1)
    assert detections[0].y == pytest.approx(0.0, abs=0.1)


def test_wall_is_rejected_by_extent():
    # A continuous arc across the whole field of view: too large to be a kart.
    ranges = [8.0] * N_BEAMS
    tracker = _tracker()
    assert tracker.detect(ranges, ANGLE_MIN, ANGLE_INCREMENT, (0.0, 0.0, 0.0)) == []
    assert tracker.last_rejected_shape >= 1


def test_noise_speck_is_rejected_by_min_points():
    ranges = _empty_scan()
    ranges[_beam_of(30.0)] = 6.0  # single stray return
    tracker = _tracker()
    assert tracker.detect(ranges, ANGLE_MIN, ANGLE_INCREMENT, (0.0, 0.0, 0.0)) == []


def test_detection_on_static_map_is_rejected():
    ranges = _empty_scan()
    _put_object(ranges, angle_deg=0.0, distance=5.0, width_deg=14.0)

    # Everything to the right of x = 5 m is "wall" in this fake map.
    tracker = _tracker(is_static=lambda x, _y: x > 5.0)
    assert tracker.detect(ranges, ANGLE_MIN, ANGLE_INCREMENT, (0.0, 0.0, 0.0)) == []
    assert tracker.last_rejected_static == 1


def test_two_frames_give_finite_difference_velocity_and_prediction():
    tracker = _tracker()

    first = _empty_scan()
    _put_object(first, angle_deg=0.0, distance=5.0, width_deg=14.0)
    tracker.update(0.0, first, ANGLE_MIN, ANGLE_INCREMENT, (0.0, 0.0, 0.0))

    # 20 Hz のスキャン 2 枚ぶん（0.1 s）で 0.5 m 離れた = 5 m/s で前に逃げている。
    second = _empty_scan()
    _put_object(second, angle_deg=0.0, distance=5.5, width_deg=13.0)
    tracker.update(0.1, second, ANGLE_MIN, ANGLE_INCREMENT, (0.0, 0.0, 0.0))

    ids = tracker.active_vehicle_ids()
    assert len(ids) == 1, "the moving kart must stay one track, not become two"
    vx, vy = tracker.velocity(ids[0])
    assert vx == pytest.approx(5.0, abs=0.5)
    assert vy == pytest.approx(0.0, abs=0.5)

    predictions = tracker.predict_all([0.0, 1.0])
    start, ahead = predictions[ids[0]]
    assert ahead[0] - start[0] == pytest.approx(vx, abs=1e-6)


def test_association_gate_starts_a_new_track_on_a_jump():
    tracker = _tracker(association_radius=1.0)

    first = _empty_scan()
    _put_object(first, angle_deg=0.0, distance=5.0, width_deg=14.0)
    tracker.update(0.0, first, ANGLE_MIN, ANGLE_INCREMENT, (0.0, 0.0, 0.0))
    first_id = tracker.active_vehicle_ids()[0]

    far = _empty_scan()
    _put_object(far, angle_deg=60.0, distance=9.0, width_deg=8.0)
    tracker.update(0.1, far, ANGLE_MIN, ANGLE_INCREMENT, (0.0, 0.0, 0.0))

    assert tracker.active_vehicle_ids() != [first_id]
    assert tracker.velocity(tracker.active_vehicle_ids()[0]) == (0.0, 0.0)


def test_stale_tracks_expire():
    tracker = _tracker(track_timeout=0.5)

    ranges = _empty_scan()
    _put_object(ranges, angle_deg=0.0, distance=5.0, width_deg=14.0)
    tracker.update(0.0, ranges, ANGLE_MIN, ANGLE_INCREMENT, (0.0, 0.0, 0.0))
    assert len(tracker.active_vehicle_ids()) == 1

    tracker.update(2.0, _empty_scan(), ANGLE_MIN, ANGLE_INCREMENT, (0.0, 0.0, 0.0))
    assert tracker.active_vehicle_ids() == []
    assert tracker.predict_all([0.0, 0.5]) == {}


def test_velocity_above_safety_limit_is_zeroed():
    warnings: List[str] = []
    tracker = _tracker(association_radius=50.0, v_max_safety=5.0,
                       warn_callback=warnings.append)

    first = _empty_scan()
    _put_object(first, angle_deg=0.0, distance=5.0, width_deg=14.0)
    tracker.update(0.0, first, ANGLE_MIN, ANGLE_INCREMENT, (0.0, 0.0, 0.0))

    second = _empty_scan()
    _put_object(second, angle_deg=0.0, distance=15.0, width_deg=6.0)
    tracker.update(0.1, second, ANGLE_MIN, ANGLE_INCREMENT, (0.0, 0.0, 0.0))

    vid = tracker.active_vehicle_ids()[0]
    assert tracker.velocity(vid) == (0.0, 0.0)
    assert warnings and "clamped to zero" in warnings[0]


def test_detections_are_reported_in_map_frame():
    ranges = _empty_scan()
    _put_object(ranges, angle_deg=0.0, distance=5.0, width_deg=14.0)

    tracker = _tracker(sensor_offset_x=1.65)
    # Vehicle at (100, 50) facing +y: a kart straight ahead lands north of it.
    detections = tracker.detect(
        ranges, ANGLE_MIN, ANGLE_INCREMENT, (100.0, 50.0, math.pi / 2))

    assert len(detections) == 1
    assert detections[0].x == pytest.approx(100.0, abs=0.1)
    assert detections[0].y == pytest.approx(50.0 + 1.65 + 5.5, abs=0.15)


def test_cluster_extent_is_the_bounding_box_diagonal():
    cluster = np.array([[0.0, 0.0], [3.0, 4.0]])
    assert cluster_extent(cluster) == pytest.approx(5.0)


def test_cluster_index_ranges_match_cluster_points():
    points = np.array([[1.0, 0.0], [1.05, 0.0], [3.0, 0.0]])
    beams = np.array([0, 1, 2])
    assert cluster_index_ranges(points, beams, gap_distance=0.5) == [(0, 2), (2, 3)]


def test_is_isolated_requires_a_jump_on_both_sides():
    # A cluster [2:4] with neighbours 0.2 m away on the left and 3 m on the right.
    points = np.array([[5.0, -0.4], [5.0, -0.2], [5.0, 0.0], [5.0, 0.2], [8.0, 0.4]])
    assert not is_isolated(points, 2, 4, isolation_gap=1.0)
    assert is_isolated(points, 3, 4, isolation_gap=0.1)


def test_wall_fragment_next_to_the_wall_is_rejected():
    """A kart-sized piece of wall, split off by a dropout, must not count."""
    ranges = _empty_scan()
    # Continuous wall along the left side, with a 3-beam dropout in the middle.
    for deg in range(20, 61):
        _put_object(ranges, angle_deg=float(deg), distance=6.0, width_deg=0.0)
    for deg in (39, 40, 41):
        ranges[_beam_of(float(deg))] = float('inf')

    tracker = _tracker(isolation_gap=1.0)
    assert tracker.detect(ranges, ANGLE_MIN, ANGLE_INCREMENT, (0.0, 0.0, 0.0)) == []


def test_kart_at_the_edge_of_the_field_of_view_is_kept():
    """The kart running beside us is cut by the 180-degree FOV — still a kart."""
    ranges = _empty_scan()
    _put_object(ranges, angle_deg=88.0, distance=3.0, width_deg=20.0)

    tracker = _tracker(isolation_gap=1.0)
    assert len(tracker.detect(ranges, ANGLE_MIN, ANGLE_INCREMENT, (0.0, 0.0, 0.0))) == 1


def test_confirmed_positions_needs_a_second_sighting():
    """1 フレームだけの検出は provider に渡さない（幻の障害物を作らないため）。"""
    tracker = _tracker()

    first = _empty_scan()
    _put_object(first, angle_deg=0.0, distance=5.0, width_deg=14.0)
    tracker.update(0.0, first, ANGLE_MIN, ANGLE_INCREMENT, (0.0, 0.0, 0.0))
    assert tracker.confirmed_positions() == [], "初回の目撃だけでは渡さない"

    second = _empty_scan()
    _put_object(second, angle_deg=0.0, distance=5.1, width_deg=14.0)
    tracker.update(0.05, second, ANGLE_MIN, ANGLE_INCREMENT, (0.0, 0.0, 0.0))

    positions = tracker.confirmed_positions()
    assert len(positions) == 1
    assert positions[0][0] == pytest.approx(5.6, abs=0.2)


def test_confirmed_positions_drops_expired_tracks():
    tracker = _tracker(track_timeout=0.5)

    for t in (0.0, 0.05):
        ranges = _empty_scan()
        _put_object(ranges, angle_deg=0.0, distance=5.0, width_deg=14.0)
        tracker.update(t, ranges, ANGLE_MIN, ANGLE_INCREMENT, (0.0, 0.0, 0.0))
    assert len(tracker.confirmed_positions()) == 1

    tracker.update(3.0, _empty_scan(), ANGLE_MIN, ANGLE_INCREMENT, (0.0, 0.0, 0.0))
    assert tracker.confirmed_positions() == []
