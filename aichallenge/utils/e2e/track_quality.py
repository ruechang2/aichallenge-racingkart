"""走行品質を測る: レースラインからの横ずれ・速度・高さ(z)の推移。"""
import csv, math, time
import numpy as np, rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from autoware_auto_vehicle_msgs.msg import VelocityReport
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

SHARE = "/aichallenge/workspace/install/multi_purpose_mpc_ros/share/multi_purpose_mpc_ros"
rows = list(csv.DictReader(open(f"{SHARE}/env/final_ver3/traj_mincurv.csv")))
PTS = np.array([[float(r['x_m']), float(r['y_m'])] for r in rows])

class Q(Node):
    def __init__(self):
        super().__init__("track_quality_probe")
        self.samples = []; self.speed = 0.0
        self.create_subscription(Odometry, "/localization/kinematic_state", self._o, 1)
        self.create_subscription(VelocityReport, "/vehicle/status/velocity_status", self._v,
            QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                       history=QoSHistoryPolicy.KEEP_LAST, depth=1))
    def _v(self, m): self.speed = float(m.longitudinal_velocity)
    def _o(self, m):
        p = m.pose.pose.position
        d = np.hypot(PTS[:,0]-p.x, PTS[:,1]-p.y)
        self.samples.append((time.monotonic(), float(d.min()), p.z, self.speed))

rclpy.init(); n=Q()
DURATION = 90
end = time.monotonic() + DURATION
while time.monotonic() < end and rclpy.ok():
    rclpy.spin_once(n, timeout_sec=0.2)

if not n.samples:
    print("no odometry"); raise SystemExit
a = np.array([(s[1], s[2], s[3]) for s in n.samples])
dev, z, v = a[:,0], a[:,1], a[:,2]
print(f"{DURATION} 秒間, サンプル {len(a)}")
print(f"  レースラインからの距離: 平均={dev.mean():.2f} m 中央値={np.median(dev):.2f} 最大={dev.max():.2f}")
print(f"  z（高さ）: 最小={z.min():.2f} 最大={z.max():.2f} 変動={z.max()-z.min():.2f} m")
print(f"  速度: 平均={v.mean():.2f} m/s 最大={v.max():.2f} 停止(<0.5)={100*np.mean(v<0.5):.0f}%")
print(f"  ライン逸脱 >2.5 m の割合: {100*np.mean(dev>2.5):.0f}%")

t0 = n.samples[0][0]
ts = np.array([s[0]-t0 for s in n.samples])
print("  10 秒ごと（平均速度 / 最大速度 / ライン距離 / z）:")
for lo in range(0, DURATION, 10):
    m = (ts >= lo) & (ts < lo+10)
    if m.any():
        print(f"    {lo:3d}-{lo+10:3d}s: v={v[m].mean():5.2f}/{v[m].max():5.2f} m/s  "
              f"dev={dev[m].mean():4.2f} m  z={z[m].mean():5.2f}")
# 走行距離（自己位置の移動量）
pos = np.array([[s[0]] for s in n.samples])  # placeholder

