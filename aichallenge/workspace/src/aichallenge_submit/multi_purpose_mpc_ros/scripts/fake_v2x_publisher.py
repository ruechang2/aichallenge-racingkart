#!/usr/bin/env python3
"""Publish synthetic V2X karts so avoidance can be tested without a second stack.

Running a real opponent means a second full Autoware stack, which saturates an
8-core host (load average 12-19, both MPC processes pinned, neither kart able to
complete a lap) — that measures CPU starvation, not control behaviour. This node
instead injects virtual karts that travel along the reference raceline at a chosen
speed and lateral offset, publishing exactly the ``/v2x/vehicle_positions``
messages AWSIM would. Cost is negligible and the scenario is deterministic.

AWSIM only publishes that topic for ego-type vehicles, and with ``--vehicles 1``
it publishes nothing at all, so with ``SIM_MODE=dev`` this node is the sole
publisher and there is no conflict.

Usage (inside the autoware container, same ROS_DOMAIN_ID as the stack):
  # one kart 25 m up the road doing 18 km/h on the racing line
  python3 fake_v2x_publisher.py --karts d2:18:25

  # two karts, the second offset 1.2 m to the left of the line
  python3 fake_v2x_publisher.py --karts d2:18:25 d3:22:60:1.2

  # a stationary kart parked 30 m ahead
  python3 fake_v2x_publisher.py --karts d2:0:30
"""

import argparse
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from v2x_msgs.msg import V2XVehiclePosition, V2XVehiclePositionArray

from multi_purpose_mpc_ros.core.utils import load_ref_path, kmh_to_m_per_sec


class FakeKart:
    """A virtual kart travelling along the raceline at a constant speed."""

    def __init__(self, vehicle_id, speed_kmh, start_s, lateral=0.0):
        self.vehicle_id = vehicle_id
        self.speed = kmh_to_m_per_sec(float(speed_kmh))
        self.s = float(start_s)
        self.lateral = float(lateral)

    def advance(self, dt, length):
        self.s = (self.s + self.speed * dt) % length


class FakeV2XPublisher(Node):

    def __init__(self, args):
        super().__init__("fake_v2x_publisher")

        x, y, _psi, _kappa = load_ref_path(args.csv)
        self._x = np.asarray(x, dtype=np.float64)
        self._y = np.asarray(y, dtype=np.float64)
        # Cumulative arc length, closing the loop so karts can lap forever.
        dx = np.diff(self._x, append=self._x[0])
        dy = np.diff(self._y, append=self._y[0])
        seg = np.hypot(dx, dy)
        self._s = np.concatenate(([0.0], np.cumsum(seg)[:-1]))
        self._length = float(seg.sum())

        self._karts = [FakeKart(*spec) for spec in args.karts]
        self._frame_id = args.frame_id

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self._pub = self.create_publisher(
            V2XVehiclePositionArray, "/v2x/vehicle_positions", qos)

        self._dt = 1.0 / args.rate
        self.create_timer(self._dt, self._tick)

        self.get_logger().info(
            "fake V2X: raceline %.1f m, %d virtual kart(s): %s" % (
                self._length, len(self._karts),
                ", ".join("%s @ %.0f km/h from s=%.0f%s" % (
                    k.vehicle_id, k.speed * 3.6, k.s,
                    (" lat %+.1f m" % k.lateral) if k.lateral else "")
                    for k in self._karts)))

    def _pose_at(self, s, lateral):
        """Interpolate the raceline at arc length ``s``, offset laterally."""
        s = s % self._length
        i = int(np.searchsorted(self._s, s, side="right") - 1)
        j = (i + 1) % len(self._x)
        span = (self._s[j] - self._s[i]) if j > i else (self._length - self._s[i])
        t = 0.0 if span <= 0.0 else (s - self._s[i]) / span

        px = self._x[i] + t * (self._x[j] - self._x[i])
        py = self._y[i] + t * (self._y[j] - self._y[i])
        if lateral:
            heading = math.atan2(self._y[j] - self._y[i], self._x[j] - self._x[i])
            px -= lateral * math.sin(heading)
            py += lateral * math.cos(heading)
        return px, py

    def _tick(self):
        stamp = self.get_clock().now().to_msg()
        msg = V2XVehiclePositionArray()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id

        for kart in self._karts:
            kart.advance(self._dt, self._length)
            px, py = self._pose_at(kart.s, kart.lateral)

            v = V2XVehiclePosition()
            v.header.stamp = stamp
            v.header.frame_id = self._frame_id
            v.vehicle_id = kart.vehicle_id
            v.position.x = px
            v.position.y = py
            v.position.z = 0.0
            v.covariance.x = 0.005
            v.covariance.y = 0.005
            v.covariance.z = 0.005
            msg.vehicles.append(v)

        self._pub.publish(msg)


def _kart_spec(text):
    """``id:speed_kmh:start_s[:lateral]``"""
    parts = text.split(":")
    if len(parts) < 3 or len(parts) > 4:
        raise argparse.ArgumentTypeError(
            "expected id:speed_kmh:start_s[:lateral], got %r" % text)
    return parts


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--csv", default="/aichallenge/workspace/src/aichallenge_submit/"
                                     "multi_purpose_mpc_ros/env/final_ver3/traj_mincurv.csv")
    ap.add_argument("--karts", nargs="+", type=_kart_spec, default=[["d2", "18", "25"]],
                    help="one or more id:speed_kmh:start_s[:lateral]")
    ap.add_argument("--rate", type=float, default=10.0, help="publish rate [Hz]")
    ap.add_argument("--frame-id", default="map")
    args = ap.parse_args()

    rclpy.init()
    node = FakeV2XPublisher(args)
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
