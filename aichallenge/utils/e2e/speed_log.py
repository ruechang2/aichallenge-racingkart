"""録画と突き合わせるための速度・位置ログ（epoch 秒つき CSV）。"""
import csv, math, time, sys
import numpy as np, rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from autoware_auto_vehicle_msgs.msg import VelocityReport
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

SHARE = "/aichallenge/workspace/install/multi_purpose_mpc_ros/share/multi_purpose_mpc_ros"
rows = list(csv.DictReader(open(f"{SHARE}/env/final_ver3/traj_mincurv.csv")))
PTS = np.array([[float(r['x_m']), float(r['y_m'])] for r in rows])
DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 150.0
OUT = sys.argv[2] if len(sys.argv) > 2 else "/output/_check/speed_log.csv"

class L(Node):
    def __init__(self):
        super().__init__("speed_logger"); self.speed=0.0; self.rows=[]
        self.create_subscription(Odometry, "/localization/kinematic_state", self._o, 1)
        self.create_subscription(VelocityReport, "/vehicle/status/velocity_status", self._v,
            QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                       history=QoSHistoryPolicy.KEEP_LAST, depth=1))
    def _v(self, m): self.speed = float(m.longitudinal_velocity)
    def _o(self, m):
        p = m.pose.pose.position
        d = float(np.min(np.hypot(PTS[:,0]-p.x, PTS[:,1]-p.y)))
        wp = int(np.argmin(np.hypot(PTS[:,0]-p.x, PTS[:,1]-p.y)))
        self.rows.append((time.time(), self.speed, p.x, p.y, p.z, d, wp))

rclpy.init(); n=L()
end = time.monotonic() + DURATION
while time.monotonic() < end and rclpy.ok():
    rclpy.spin_once(n, timeout_sec=0.1)
with open(OUT, "w", newline="") as f:
    w = csv.writer(f); w.writerow(["epoch","speed","x","y","z","dev","wp"]); w.writerows(n.rows)
print(f"wrote {len(n.rows)} rows to {OUT}")
