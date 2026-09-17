"""自車が地図上どこにいるか（走行可能セルか、周囲の占有状況）を見る。"""
import sys, math, time
sys.path.insert(0, "/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros")
import numpy as np, yaml, rclpy, csv
from rclpy.node import Node
from nav_msgs.msg import Odometry
from multi_purpose_mpc_ros.core.map import Map

class O(Node):
    def __init__(self):
        super().__init__("where_probe"); self.odom=None
        self.create_subscription(Odometry, "/localization/kinematic_state", self._o, 1)
    def _o(self, m): self.odom=m
rclpy.init(); n=O(); end=time.monotonic()+10
while time.monotonic()<end and n.odom is None: rclpy.spin_once(n, timeout_sec=0.2)
p = n.odom.pose.pose.position
SHARE="/aichallenge/workspace/install/multi_purpose_mpc_ros/share/multi_purpose_mpc_ros"
cfg=yaml.safe_load(open(f"{SHARE}/config/config.yaml"))
m=Map(f"{SHARE}/{cfg['map']['yaml_path']}")
dx,dy=m.w2m(p.x,p.y)
free = m.data_backup[dy,dx]==1
print(f"pose=({p.x:.1f}, {p.y:.1f}, z={p.z:.2f})  地図セル: {'空き(走行可)' if free else '占有(コース外)'}")
# 周囲 3 m の空き率
cells=int(3.0/m.resolution)
win=m.data_backup[max(dy-cells,0):dy+cells, max(dx-cells,0):dx+cells]
print(f"半径3m窓の空き率: {100*np.mean(win==1):.0f}%")
rows=list(csv.DictReader(open(f"{SHARE}/env/final_ver3/traj_mincurv.csv")))
pts=np.array([[float(r['x_m']),float(r['y_m'])] for r in rows])
d=np.hypot(pts[:,0]-p.x, pts[:,1]-p.y)
i=int(np.argmin(d))
print(f"最寄り waypoint={i} 距離={d.min():.2f} m  (全周 {len(pts)} 点)")
