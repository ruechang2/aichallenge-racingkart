"""実走中の topic から LiDAR 障害物推定を読み取り専用で検証する（指令は一切出さない）。"""
import math
import sys
import time

sys.path.insert(0, "/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros")

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from autoware_auto_vehicle_msgs.msg import VelocityReport

from multi_purpose_mpc_ros.core.map import Map
from multi_purpose_mpc_ros.lidar_vehicle_tracker import LidarVehicleTracker

MAP_YAML = "/aichallenge/workspace/install/multi_purpose_mpc_ros/share/multi_purpose_mpc_ros/env/final_ver3/occupancy_grid_map.yaml"


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.x * q.y + q.w * q.z),
                      q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z)


class Probe(Node):
    def __init__(self):
        super().__init__("lidar_obstacle_probe")
        self.map = Map(MAP_YAML)
        self.static_margin = 0.3

        def is_static(x, y):
            dx, dy = self.map.w2m(x, y)
            cells = max(int(round(self.static_margin / self.map.resolution)), 0)
            x0, x1 = max(dx - cells, 0), min(dx + cells + 1, self.map.width)
            y0, y1 = max(dy - cells, 0), min(dy + cells + 1, self.map.height)
            if x0 >= x1 or y0 >= y1:
                return False
            return bool(np.any(self.map.data_backup[y0:y1, x0:x1] == 0))

        self.tracker = LidarVehicleTracker(vehicle_radius=0.5, is_static=is_static)
        self.tracker_nomap = LidarVehicleTracker(vehicle_radius=0.5)
        self.odom = None
        self.ego_speed = 0.0
        self.ego_speeds = []
        self.n_scans = 0
        self.with_map = []
        self.without_map = []
        self.tracks = []
        # 追跡 id ごとの (時刻, x, y)。壁の断片は 1-2 フレームで消えるが、
        # 本物のカートは長く生き残る。真偽の切り分けはこの寿命で行う。
        self.history = {}

        self.create_subscription(Odometry, "/localization/kinematic_state",
                                 self._odom_cb, 1)
        self.create_subscription(VelocityReport, "/vehicle/status/velocity_status",
                                 self._vel_cb, QoSProfile(
                                     reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                     history=QoSHistoryPolicy.KEEP_LAST, depth=1))
        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(LaserScan, "/sensing/lidar/scan", self._scan_cb, qos)

    def _odom_cb(self, msg):
        self.odom = msg

    def _vel_cb(self, msg):
        self.ego_speed = float(msg.longitudinal_velocity)

    def _scan_cb(self, msg):
        now = time.monotonic()
        self.ego_speeds.append(self.ego_speed)
        if self.odom is not None:
            p = self.odom.pose.pose
            pose = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))
            self.tracker.update(now, msg.ranges, msg.angle_min, msg.angle_increment, pose)
            self.with_map.append(len(self.tracker.last_detections))
            self.tracks.append(len(self.tracker.active_vehicle_ids()))
            self._record(self.tracker, now)
        else:
            # 自己位置が出ていない（E2E モードは imu/gnss off）。地図フィルタは
            # map 座標が要るので、形状フィルタだけをセンサ座標系で評価する。
            pose = (0.0, 0.0, 0.0)
            self.tracker_nomap.update(now, msg.ranges, msg.angle_min, msg.angle_increment, pose)
            self.tracks.append(len(self.tracker_nomap.active_vehicle_ids()))
            self._record(self.tracker_nomap, now)
            self.without_map.append(len(self.tracker_nomap.last_detections))
            self.n_scans += 1
            return
        self.without_map.append(
            len(self.tracker_nomap.detect(msg.ranges, msg.angle_min, msg.angle_increment, pose)))
        self.n_scans += 1


    def _record(self, tracker, now):
        for vid in tracker.active_vehicle_ids():
            pos = tracker.predict_positions(vid, [0.0])
            if pos:
                self.history.setdefault(vid, []).append((now, pos[0][0], pos[0][1]))


def main():
    rclpy.init()
    node = Probe()
    deadline = time.monotonic() + 25.0
    while time.monotonic() < deadline and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.n_scans == 0:
        print("no scans received (odom present: %s)" % (node.odom is not None))
    else:
        nm = np.array(node.without_map)
        tk = np.array(node.tracks)
        es = np.array(node.ego_speeds) if node.ego_speeds else np.zeros(1)
        print(f"ego speed: mean={es.mean():.2f} m/s  max={es.max():.2f}  "
              f"stopped(<0.5m/s)={100*(es<0.5).mean():.0f}% of the window")
        print(f"scans={node.n_scans}  odom={'yes' if node.odom is not None else 'NO (map filter not evaluated)'}")
        print(f"  shape filter only : {nm.mean():.2f}/scan  max={nm.max()}  clean={100*(nm==0).mean():.0f}%")
        if node.with_map:
            wm = np.array(node.with_map)
            print(f"  + static map      : {wm.mean():.2f}/scan  max={wm.max()}  clean={100*(wm==0).mean():.0f}%")
        print(f"  active tracks     : mean={tk.mean():.2f}  max={tk.max()}")
        print(f"  detections/scan histogram: "
              + ", ".join(f"{k}:{int((nm==k).sum())}" for k in range(0, min(int(nm.max()), 5) + 1)))

        lifetimes = []
        for vid, h in node.history.items():
            if len(h) < 2:
                lifetimes.append((vid, len(h), 0.0, 0.0))
                continue
            dur = h[-1][0] - h[0][0]
            disp = math.hypot(h[-1][1] - h[0][1], h[-1][2] - h[0][2])
            lifetimes.append((vid, len(h), dur, disp / dur if dur > 0 else 0.0))
        n_total = len(lifetimes)
        short = [l for l in lifetimes if l[2] < 0.2]
        longl = sorted([l for l in lifetimes if l[2] >= 0.5], key=lambda l: -l[2])
        print(f"  tracks: {n_total} total | <0.2 s (ちらつき): {len(short)} "
              f"| >=0.5 s (実体らしい): {len(longl)}")
        for vid, n, dur, spd in longl[:8]:
            print(f"    {vid}: {n} frames, {dur:.2f} s, mean speed {spd:.2f} m/s")
    node.destroy_node()
    rclpy.shutdown()


main()
