import rclpy, time
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
BE=QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,history=QoSHistoryPolicy.KEEP_LAST,depth=50)
msgs=[]
rclpy.init(); n=Node("status_probe")
n.create_subscription(Float32MultiArray,"/awsim/status",lambda m: msgs.append((time.time(),list(m.data))),BE)
end=time.monotonic()+3
while time.monotonic()<end: rclpy.spin_once(n,timeout_sec=0.1)
print(len(msgs),"msgs in 3s")
for t,d in msgs[:12]: print(f"{t%100:6.2f}", [round(x,2) for x in d])
