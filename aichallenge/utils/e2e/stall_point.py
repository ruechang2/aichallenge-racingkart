"""停止地点が地図上どこか、その waypoint のコリドーはどうなっているかを見る。"""
import csv, math, sys, time
sys.path.insert(0, "/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros")
import numpy as np, yaml, rclpy
from rclpy.node import Node
from multi_purpose_mpc_ros.core.map import Map
from multi_purpose_mpc_ros_msgs.msg import PathConstraints

rows = list(csv.DictReader(open("/output/_check/speed_log.csv")))
last = rows[-1]
px, py, pz, wp = float(last['x']), float(last['y']), float(last['z']), int(last['wp'])
SHARE="/aichallenge/workspace/install/multi_purpose_mpc_ros/share/multi_purpose_mpc_ros"
cfg=yaml.safe_load(open(f"{SHARE}/config/config.yaml"))
m=Map(f"{SHARE}/{cfg['map']['yaml_path']}")
dx,dy=m.w2m(px,py)
print(f"停止地点 ({px:.1f}, {py:.1f}, z={pz:.2f}) wp={wp}")
print(f"  地図セル: {'空き' if m.data_backup[dy,dx]==1 else '占有'}")
for r in (1.0, 2.0):
    c=int(r/m.resolution)
    win=m.data_backup[max(dy-c,0):dy+c, max(dx-c,0):dx+c]
    print(f"  半径{r}m の空き率: {100*np.mean(win==1):.0f}%")

class C(Node):
    def __init__(self):
        super().__init__("stall_probe"); self.msg=None
        self.create_subscription(PathConstraints,
            "/path_constraints_provider/path_constraints", self._pc, 1)
    def _pc(self, msg): self.msg=msg
rclpy.init(); n=C(); end=time.monotonic()+25
while time.monotonic()<end and n.msg is None: rclpy.spin_once(n, timeout_sec=0.2)
if n.msg:
    ub=np.asarray(n.msg.upper_bounds).reshape(n.msg.rows, n.msg.cols)
    lb=np.asarray(n.msg.lower_bounds).reshape(n.msg.rows, n.msg.cols)
    print(f"  wp {wp} のコリドー: ub={ub[wp,0]:+.2f} lb={lb[wp,0]:+.2f} 幅={ub[wp,0]-lb[wp,0]:.2f} m")
    print("  前後 wp の幅:", " ".join(f"{i}:{ub[i,0]-lb[i,0]:.1f}" for i in range(max(wp-4,0), min(wp+5, n.msg.rows))))
