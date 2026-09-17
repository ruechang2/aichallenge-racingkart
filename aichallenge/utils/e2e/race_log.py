"""走行の監視ログ。速度と /awsim/status（自車分）を epoch 秒つき CSV に書き、ラップが進むたびに print。
自車の status は data[5]（残りブースト数）が 2 の行で見分ける（NPC は 0）。
自己位置は E2E モード（gnss/imu off）では来ないので、来たときだけ x,y を入れる。

    python3 race_log.py <秒数> <出力 CSV>
"""
import csv, sys, time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from autoware_auto_vehicle_msgs.msg import VelocityReport
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 480.0
OUT = sys.argv[2] if len(sys.argv) > 2 else "/output/_check/race_log.csv"
BE = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=10)

class L(Node):
    def __init__(self):
        super().__init__("race_logger")
        self.speed = 0.0; self.x = float("nan"); self.y = float("nan")
        self.lap = None; self.laptime = None; self.sector = None; self.boost = None
        self.rows = []; self.t0 = time.time(); self.finished = False
        self.create_subscription(Odometry, "/localization/kinematic_state", self._o, 1)
        self.create_subscription(VelocityReport, "/vehicle/status/velocity_status", self._v, BE)
        self.create_subscription(Float32MultiArray, "/awsim/status", self._s, BE)
        self.create_timer(0.1, self._tick)
    def _v(self, m): self.speed = float(m.longitudinal_velocity)
    def _o(self, m): self.x = m.pose.pose.position.x; self.y = m.pose.pose.position.y
    def _s(self, m):
        d = list(m.data)
        if len(d) < 6 or d[5] < 1.0:   # NPC 分は捨てる
            return
        lap, laptime, sector = int(d[1]), float(d[2]), int(d[3])
        if lap != self.lap:
            print(f"[race_log] t={time.time()-self.t0:7.1f}s lap {self.lap} -> {lap} (前ラップ {self.laptime}s) status={[round(x,1) for x in d]}", flush=True)
        self.lap, self.laptime, self.sector, self.boost = lap, laptime, sector, d[5]
    def _tick(self):
        self.rows.append((time.time(), self.speed, self.x, self.y, self.lap if self.lap is not None else -1,
                          self.laptime if self.laptime is not None else -1, self.sector if self.sector is not None else -1))

rclpy.init(); n = L()
end = time.monotonic() + DURATION; last_report = time.monotonic()
while time.monotonic() < end and rclpy.ok():
    rclpy.spin_once(n, timeout_sec=0.1)
    if time.monotonic() - last_report > 30:
        last_report = time.monotonic()
        print(f"[race_log] t={time.time()-n.t0:7.1f}s speed={n.speed:5.2f} lap={n.lap} laptime={n.laptime} sector={n.sector}", flush=True)
with open(OUT, "w", newline="") as f:
    w = csv.writer(f); w.writerow(["epoch", "speed", "x", "y", "lap", "laptime", "sector"]); w.writerows(n.rows)
print(f"[race_log] wrote {len(n.rows)} rows to {OUT}", flush=True)
