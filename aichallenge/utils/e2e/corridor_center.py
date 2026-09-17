"""MPC が目標にする横位置 (lb+ub)/2 が、レースラインからどれだけ離れているかを見る。"""
import time
import numpy as np, rclpy
from rclpy.node import Node
from multi_purpose_mpc_ros_msgs.msg import PathConstraints

class C(Node):
    def __init__(self):
        super().__init__("corridor_center_probe"); self.msg=None
        self.create_subscription(PathConstraints,
            "/path_constraints_provider/path_constraints", self._pc, 1)
    def _pc(self, m): self.msg = m

rclpy.init(); n=C(); end=time.monotonic()+30
while time.monotonic()<end and n.msg is None: rclpy.spin_once(n, timeout_sec=0.2)
m = n.msg
if m is None:
    print("no constraints received"); raise SystemExit
ub = np.asarray(m.upper_bounds); lb = np.asarray(m.lower_bounds)
center = (ub + lb) / 2.0          # MPC が xr に入れる目標横位置
width = ub - lb
print(f"rows={m.rows} cols={m.cols} 要素数={len(ub)}")
print(f"コリドー中心のオフセット（レースライン=0、左が正）:")
print(f"  平均={center.mean():+.2f} m  中央値={np.median(center):+.2f} m  "
      f"最小={center.min():+.2f}  最大={center.max():+.2f}")
print(f"  |中心| > 0.5 m の区間: {100*np.mean(np.abs(center)>0.5):.0f}%")
print(f"  |中心| > 1.0 m の区間: {100*np.mean(np.abs(center)>1.0):.0f}%")
print(f"コリドー幅: 平均={width.mean():.2f} m 最小={width.min():.2f} m")
print(f"参考: ub 平均={ub.mean():+.2f} m, lb 平均={lb.mean():+.2f} m")
