"""コリドーが車幅より狭くなる区間を、周回位置つきで洗い出す。"""
import time
import numpy as np, rclpy
from rclpy.node import Node
from multi_purpose_mpc_ros_msgs.msg import PathConstraints

class C(Node):
    def __init__(self):
        super().__init__("pinch_probe"); self.msg=None
        self.create_subscription(PathConstraints,
            "/path_constraints_provider/path_constraints", self._pc, 1)
    def _pc(self, m): self.msg=m
rclpy.init(); n=C(); end=time.monotonic()+30
while time.monotonic()<end and n.msg is None: rclpy.spin_once(n, timeout_sec=0.2)
m=n.msg
ub=np.asarray(m.upper_bounds).reshape(m.rows, m.cols)
lb=np.asarray(m.lower_bounds).reshape(m.rows, m.cols)
w=(ub-lb)[:,0]     # 各 waypoint の、その地点での幅
print(f"waypoints={m.rows}")
print(f"幅: 平均={w.mean():.2f} 最小={w.min():.2f} @wp={int(np.argmin(w))}")
VEH = 2.30  # bicycle_model.width（安全余裕込み）
narrow = np.where(w < VEH)[0]
print(f"車幅 {VEH} m を下回る waypoint: {len(narrow)} 箇所")
if len(narrow):
    # 連続した区間にまとめる
    groups=[]; start=narrow[0]; prev=narrow[0]
    for i in narrow[1:]:
        if i != prev+1:
            groups.append((start, prev)); start=i
        prev=i
    groups.append((start, prev))
    for a,b in groups:
        print(f"  wp {a}-{b}: 幅 {w[a:b+1].min():.2f}〜{w[a:b+1].max():.2f} m")
print("\n止まった地点の周辺 (wp 268-281):")
for i in range(268, 282):
    print(f"  wp {i}: {w[i]:.2f} m")
