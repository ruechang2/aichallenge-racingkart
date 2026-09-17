"""実走中の自車姿勢で、回避モードのコリドー境界 ub/lb が自車を含んでいるかを見る。"""
import sys, math, time
sys.path.insert(0, "/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros")
import numpy as np, yaml, rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from multi_purpose_mpc_ros.core.map import Map
from multi_purpose_mpc_ros.core.reference_path import ReferencePath
from multi_purpose_mpc_ros.core.utils import load_ref_path

class O(Node):
    def __init__(self):
        super().__init__("bounds_probe"); self.odom=None
        self.create_subscription(Odometry, "/localization/kinematic_state", self._o, 1)
    def _o(self, m): self.odom = m

rclpy.init(); n=O(); end=time.monotonic()+10
while time.monotonic()<end and n.odom is None: rclpy.spin_once(n, timeout_sec=0.2)
p = n.odom.pose.pose
q = p.orientation
yaw = math.atan2(2*(q.x*q.y+q.w*q.z), q.w*q.w+q.x*q.x-q.y*q.y-q.z*q.z)
print(f"live pose: ({p.position.x:.1f}, {p.position.y:.1f}) yaw={math.degrees(yaw):.1f} deg")

SHARE = "/aichallenge/workspace/install/multi_purpose_mpc_ros/share/multi_purpose_mpc_ros"
cfg = yaml.safe_load(open(f"{SHARE}/config/config.yaml"))
m = Map(f"{SHARE}/{cfg['map']['yaml_path']}")
wx, wy, _, _ = load_ref_path(f"{SHARE}/{cfg['reference_path']['csv_path']}")
rc = cfg["reference_path"]
rp = ReferencePath(m, wx, wy, rc["resolution"], rc["smoothing_distance"], rc["max_width"], rc["circular"])
length, width, N = cfg["bicycle_model"]["length"], cfg["bicycle_model"]["width"], cfg["mpc"]["N"]

# 自車に最も近い waypoint を wp_id とする（コントローラ内の get_current_waypoint 相当）
d = [ (math.hypot(w.x-p.position.x, w.y-p.position.y), i) for i, w in enumerate(rp.waypoints) ]
d.sort(); dist, wp_id = d[0]
wp = rp.get_waypoint(wp_id)
# e_y: 経路直交方向の自車オフセット
dx, dy = p.position.x - wp.x, p.position.y - wp.y
e_y = -dx*math.sin(wp.psi) + dy*math.cos(wp.psi)
print(f"nearest wp={wp_id} dist={dist:.2f} m, e_y={e_y:+.2f} m (経路左が正)")

for sm in (0.0, 0.1, 0.5):
    ub, lb, _ = rp.update_path_constraints(wp_id+1, [p.position.x, p.position.y, yaw], N, length, width, sm)
    ub = np.asarray(ub); lb = np.asarray(lb)
    inside = (e_y <= ub[0]) and (e_y >= lb[0])
    print(f"safety_margin={sm}: ub[0..3]={np.round(ub[:4],2)} lb[0..3]={np.round(lb[:4],2)} "
          f"幅={np.round(ub[:4]-lb[:4],2)}  自車 e_y は最初の区間の内側? {inside}")
    print(f"    幅が 0 の区間: {int(np.sum((ub-lb) <= 1e-6))}/{len(ub)}")
