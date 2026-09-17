"""LiDAR 障害物が provider のコリドーに効いているかを、制約の幅の変化で見る。"""
import time
import numpy as np, rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from multi_purpose_mpc_ros_msgs.msg import PathConstraints

class C(Node):
    def __init__(self):
        super().__init__("corridor_effect_probe")
        self.objs = None
        self.samples = []
        self.create_subscription(PathConstraints,
            "/path_constraints_provider/path_constraints", self._pc, 1)
        self.create_subscription(Float64MultiArray, "/aichallenge/objects", self._ob, 1)
    def _ob(self, m):
        self.objs = len(m.data) // 4
        print(f"[objects] {self.objs} 個を受信", flush=True)
    def _pc(self, m):
        ub = np.asarray(m.upper_bounds); lb = np.asarray(m.lower_bounds)
        w = ub - lb
        self.samples.append((self.objs, float(np.min(w)), float(np.median(w)), int(np.sum(w <= 1e-6))))
        print(f"[constraints] objects={self.objs} 最小幅={np.min(w):.2f} m "
              f"中央値={np.median(w):.2f} m 幅0の区間={int(np.sum(w<=1e-6))}/{len(w)}", flush=True)

rclpy.init(); n = C()
end = time.monotonic() + 45
while time.monotonic() < end and rclpy.ok():
    rclpy.spin_once(n, timeout_sec=0.2)
if n.samples:
    with_obj = [s for s in n.samples if s[0]]
    without = [s for s in n.samples if not s[0]]
    print(f"\nまとめ: 障害物あり {len(with_obj)} 回, なし {len(without)} 回")
    for label, rows in (("あり", with_obj), ("なし", without)):
        if rows:
            print(f"  {label}: 最小幅 平均={np.mean([r[1] for r in rows]):.2f} m, "
                  f"幅0区間 平均={np.mean([r[3] for r in rows]):.1f}")
