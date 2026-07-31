#!/usr/bin/env python3
"""Record the ego's relative geometry to every V2X kart, to measure an overtake.

The MPC's throttled traffic log is too coarse to answer the question that actually
matters about a pass: how close did we get, and for how long were we alongside.
This logs `t,vehicle_id,ahead,lateral,gap,ego_v` at the odometry rate so a pass can
be reconstructed afterwards.

`ahead`/`lateral` are in the ego frame (+lateral = left). `gap` is the straight-line
centre-to-centre distance, which is the number to compare against the two karts'
footprints (1.45 m wide, so centres closer than ~1.45 m means contact).

Usage (inside the autoware container, same ROS_DOMAIN_ID as the stack):
  python3 trace_traffic.py --out /output/overtake.csv [--duration 300]
"""

import argparse
import csv
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from v2x_msgs.msg import V2XVehiclePositionArray


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class TrafficTracer(Node):

    def __init__(self, args):
        super().__init__("traffic_tracer")
        self._v2x = None
        self._t0 = None
        self._file = open(args.out, "w", newline="")
        self._w = csv.writer(self._file)
        self._w.writerow(["t", "vehicle_id", "ahead", "lateral", "gap", "ego_v"])

        best_effort = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                 history=HistoryPolicy.KEEP_LAST)
        reliable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(V2XVehiclePositionArray, "/v2x/vehicle_positions",
                                 self._on_v2x, reliable)
        self.create_subscription(Odometry, "/localization/kinematic_state",
                                 self._on_odom, best_effort)
        self.get_logger().info("tracing traffic to %s" % args.out)

    def _on_v2x(self, msg):
        self._v2x = msg

    def _on_odom(self, msg: Odometry):
        if self._v2x is None:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._t0 is None:
            self._t0 = t
        p = msg.pose.pose
        yaw = yaw_from_quaternion(p.orientation)
        hx, hy = math.cos(yaw), math.sin(yaw)
        ego_v = msg.twist.twist.linear.x

        for veh in self._v2x.vehicles:
            rx = veh.position.x - p.position.x
            ry = veh.position.y - p.position.y
            ahead = rx * hx + ry * hy
            lateral = -rx * hy + ry * hx
            self._w.writerow(["%.3f" % (t - self._t0), veh.vehicle_id,
                              "%.3f" % ahead, "%.3f" % lateral,
                              "%.3f" % math.hypot(rx, ry), "%.3f" % ego_v])
        self._file.flush()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="/output/traffic.csv")
    args = ap.parse_args()

    rclpy.init()
    node = TrafficTracer(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
