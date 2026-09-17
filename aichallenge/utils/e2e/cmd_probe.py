import rclpy, time
from rclpy.node import Node
from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_auto_vehicle_msgs.msg import VelocityReport, GearReport
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
BE=QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,history=QoSHistoryPolicy.KEEP_LAST,depth=1)
class P(Node):
    def __init__(s):
        super().__init__("cmd_probe"); s.c=None; s.v=None; s.g=None
        s.create_subscription(AckermannControlCommand,"/control/command/control_cmd",lambda m: setattr(s,'c',m),1)
        s.create_subscription(VelocityReport,"/vehicle/status/velocity_status",lambda m: setattr(s,'v',m),BE)
        s.create_subscription(GearReport,"/vehicle/status/gear_status",lambda m: setattr(s,'g',m),BE)
rclpy.init(); n=P(); end=time.monotonic()+6
while time.monotonic()<end: rclpy.spin_once(n,timeout_sec=0.2)
c=n.c; print("cmd:", None if c is None else f"speed={c.longitudinal.speed:.2f} acc={c.longitudinal.acceleration:.2f} steer={c.lateral.steering_tire_angle:.2f}")
print("vel:", None if n.v is None else round(n.v.longitudinal_velocity,3), "gear:", None if n.g is None else n.g.report)
