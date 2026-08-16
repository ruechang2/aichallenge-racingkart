# Copyright 2026 aichallenge
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.

"""Steer around a kart ahead instead of queueing behind it.

Sits BEFORE collision_guard in the command chain:

    MPC -> control_cmd_mpc -> lateral_avoidance -> control_cmd_avoided
        -> collision_guard -> ... -> vehicle

That order is the whole point. The guard tests obstacles against the arc the ego
is actually on, so as soon as this node steers away, the kart leaves the guard's
corridor and its speed cap releases by itself — no coordination between the two
nodes, no new state, just geometry. Putting this after the guard instead would
mean steering around a kart the guard had already stopped us for.

The guard keeps sole authority over speed and this node keeps sole authority
over the lateral bias, so neither can undo the other.

What it deliberately will not do: leave the drivable surface. ``max_offset``
bounds how far off the racing line it will go, and if neither side has that much
room it emits no bias at all and lets the guard slow for the kart. Following a
kart costs seconds; beaching costs the session (measured: 0 laps).
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_auto_planning_msgs.msg import Trajectory
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32
from v2x_msgs.msg import V2XVehiclePositionArray

from lateral_avoidance.avoidance import blend, pass_side, steering_bias


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class LateralAvoidanceNode(Node):
    def __init__(self) -> None:
        super().__init__("lateral_avoidance")

        self._enabled = bool(self.declare_parameter("enable_avoidance", True).value)

        # Geometry of a pass. 1.45 m kart + 1.45 m kart side by side needs ~1.5 m
        # between centrelines; 1.6 m leaves a little for localisation error.
        self._pass_clearance = float(self.declare_parameter("pass_clearance", 1.6).value)
        # How far off the racing line we are willing to run. The track is ~6 m, so
        # the line plus 1.6 m still leaves surface under the outside wheels. This
        # is the guard rail against the failure that killed the MPC's own
        # avoidance: it swerved to 2.0-2.3 m and stranded the car on the edge.
        self._max_offset = float(self.declare_parameter("max_offset", 1.6).value)
        self._preview = float(self.declare_parameter("preview_distance", 12.0).value)
        # Keep reacting right down to contact range: the guard parks the car
        # 0.73 m behind a kart, and that is the case that needs steering.
        self._min_preview = float(self.declare_parameter("min_preview_distance", 0.5).value)
        self._corridor_half_width = float(
            self.declare_parameter("corridor_half_width", 1.2).value)

        # Command conversion. steering_tire_angle carries kappa * gain, not an
        # angle — see avoidance.py.
        self._gain = float(self.declare_parameter("steering_tire_angle_gain", 1.639).value)
        self._max_curvature_bias = float(
            self.declare_parameter("max_curvature_bias", 0.25).value)
        # Per-cycle change limit on the emitted bias, in command units.
        self._bias_rate = float(self.declare_parameter("bias_rate", 0.05).value)

        self._self_ignore_radius = float(
            self.declare_parameter("self_ignore_radius", 1.5).value)
        self._sensor_stale_sec = float(self.declare_parameter("sensor_stale_sec", 0.5).value)

        # --- state ---
        self._odom = None
        self._v2x = None
        self._trajectory = None
        self._bias = 0.0
        self._was_avoiding = False

        reliable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              history=HistoryPolicy.KEEP_LAST)
        best_effort = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                 history=HistoryPolicy.KEEP_LAST)

        self._pub = self.create_publisher(AckermannControlCommand, "~/output/control_cmd", 1)
        self._bias_pub = self.create_publisher(Float32, "~/output/lateral_bias", 1)
        self._active_pub = self.create_publisher(Bool, "~/output/is_avoiding", 1)

        self.create_subscription(AckermannControlCommand, "~/input/control_cmd",
                                 self._on_cmd, 1)
        self.create_subscription(Odometry, "/localization/kinematic_state",
                                 self._on_odom, best_effort)
        self.create_subscription(Trajectory, "/planning/scenario_planning/trajectory",
                                 self._on_trajectory, best_effort)
        self.create_subscription(V2XVehiclePositionArray, "/v2x/vehicle_positions",
                                 self._on_v2x, reliable)

        self.get_logger().info(
            f"lateral_avoidance up (enabled={self._enabled}, "
            f"clearance={self._pass_clearance} m, max_offset={self._max_offset} m, "
            f"preview={self._preview} m)")

    # --- subscriptions ---
    def _on_odom(self, msg: Odometry) -> None:
        self._odom = msg

    def _on_v2x(self, msg: V2XVehiclePositionArray) -> None:
        self._v2x = msg

    def _on_trajectory(self, msg: Trajectory) -> None:
        self._trajectory = [(p.pose.position.x, p.pose.position.y) for p in msg.points]

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _is_fresh(self, msg) -> bool:
        if msg is None:
            return False
        try:
            stamp = msg.header.stamp
        except AttributeError:
            return True
        msg_t = stamp.sec + stamp.nanosec * 1e-9
        if msg_t <= 0.0:
            return True
        return (self._now() - msg_t) <= self._sensor_stale_sec

    # --- geometry ---
    def _offset_from_line(self, x, y, yaw):
        """How far left of the racing line the ego currently sits.

        Without a trajectory we cannot bound the excursion, so the caller treats
        None as "no room known" and declines to avoid rather than guessing.
        """
        if not self._trajectory or len(self._trajectory) < 2:
            return None
        best_i, best_d = 0, float("inf")
        for i, (px, py) in enumerate(self._trajectory):
            d = (px - x) ** 2 + (py - y) ** 2
            if d < best_d:
                best_d, best_i = d, i
        px, py = self._trajectory[best_i]
        rx, ry = px - x, py - y
        # Positive when the LINE is to our left, so the ego offset is the negative.
        lat_to_line = -rx * math.sin(yaw) + ry * math.cos(yaw)
        return -lat_to_line

    def _kart_ahead(self, x, y, yaw):
        """The nearest kart in front of us inside the corridor, as (lon, lat)."""
        if not self._is_fresh(self._v2x):
            return None
        hx, hy = math.cos(yaw), math.sin(yaw)
        nearest = None
        for veh in self._v2x.vehicles:
            rx, ry = veh.position.x - x, veh.position.y - y
            if math.hypot(rx, ry) < self._self_ignore_radius:
                continue
            lon = rx * hx + ry * hy
            lat = -rx * hy + ry * hx
            if lon < self._min_preview or lon > self._preview:
                continue
            if abs(lat) > self._corridor_half_width + self._pass_clearance:
                continue  # far enough to the side that we are already going round
            if nearest is None or lon < nearest[0]:
                nearest = (lon, lat)
        return nearest

    # --- core ---
    def _on_cmd(self, msg: AckermannControlCommand) -> None:
        out = msg
        if not self._enabled or self._odom is None:
            self._publish(out, avoiding=False)
            return

        p = self._odom.pose.pose
        x, y = p.position.x, p.position.y
        yaw = yaw_from_quaternion(p.orientation)

        target = 0.0
        avoiding = False
        kart = self._kart_ahead(x, y, yaw)
        if kart is not None:
            ego_offset = self._offset_from_line(x, y, yaw)
            if ego_offset is not None:
                lon, lat = kart
                shift = pass_side(lat, ego_offset, self._pass_clearance, self._max_offset)
                if shift:
                    # Aim to have completed the shift by the time we reach the kart.
                    target = steering_bias(shift, max(lon, self._min_preview),
                                           self._gain, self._max_curvature_bias)
                    avoiding = True
                elif shift is None:
                    self.get_logger().warn(
                        "kart ahead with no room to pass on either side — "
                        "leaving it to collision_guard", once=True)

        self._bias = blend(self._bias, target, self._bias_rate)
        if abs(self._bias) > 1e-6:
            out.lateral.steering_tire_angle = \
                msg.lateral.steering_tire_angle + self._bias
        self._publish(out, avoiding)

        if avoiding != self._was_avoiding:
            if avoiding:
                self.get_logger().info(
                    "avoiding: kart %.1f m ahead, %.2f m lateral, bias %.3f"
                    % (kart[0], kart[1], self._bias))
            else:
                self.get_logger().info("avoidance released")
            self._was_avoiding = avoiding

    def _publish(self, cmd, avoiding: bool) -> None:
        self._pub.publish(cmd)
        b = Float32()
        b.data = float(self._bias)
        self._bias_pub.publish(b)
        a = Bool()
        a.data = bool(avoiding)
        self._active_pub.publish(a)


def main(args=None):
    rclpy.init(args=args)
    node = LateralAvoidanceNode()
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
