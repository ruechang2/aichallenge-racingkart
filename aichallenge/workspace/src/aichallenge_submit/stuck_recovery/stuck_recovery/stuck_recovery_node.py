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

"""Automatic recovery from being stuck against a wall or another kart.

Sits at the end of the command chain, after collision_guard:

    MPC -> control_cmd_mpc -> collision_guard -> control_cmd_guarded
        -> stuck_recovery -> control_cmd -> vehicle

It republishes the incoming command untouched until the car stops making
progress while the controller is still asking it to drive. Then it takes over:
shift to reverse, back away with the steering mirrored so the nose swings back
towards the racing line, shift to drive, pull away, and hand control back to the
controller. Being last in the chain is deliberate -- the guard would otherwise
brake for the very obstacle we are backing away from.

The recovery is armed only after the car has driven once, so it can never fire
on the starting grid, where standing still with a speed command is normal.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy)

from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_auto_planning_msgs.msg import Trajectory
from autoware_auto_vehicle_msgs.msg import GearCommand, GearReport
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from v2x_msgs.msg import V2XVehiclePositionArray

from stuck_recovery.recovery_logic import (
    FORWARD, IDLE, REVERSE, SHIFT_DRIVE, SHIFT_REVERSE,
    RecoveryConfig, StuckRecovery, mirrored_for_reverse, path_target, recovery_steer,
)


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class StuckRecoveryNode(Node):
    def __init__(self) -> None:
        super().__init__("stuck_recovery")

        # --- parameters ---
        self._enabled = bool(self.declare_parameter("enable_recovery", True).value)

        cfg = RecoveryConfig()
        cfg.arm_speed = float(self.declare_parameter("arm_speed", cfg.arm_speed).value)
        cfg.stuck_radius = float(self.declare_parameter("stuck_radius", cfg.stuck_radius).value)
        cfg.stuck_duration = float(self.declare_parameter("stuck_duration", cfg.stuck_duration).value)
        cfg.intent_speed = float(self.declare_parameter("intent_speed", cfg.intent_speed).value)
        cfg.shift_timeout = float(self.declare_parameter("shift_timeout", cfg.shift_timeout).value)
        cfg.reverse_distance = float(self.declare_parameter("reverse_distance", cfg.reverse_distance).value)
        cfg.reverse_timeout = float(self.declare_parameter("reverse_timeout", cfg.reverse_timeout).value)
        cfg.forward_duration = float(self.declare_parameter("forward_duration", cfg.forward_duration).value)
        cfg.forward_release_speed = float(
            self.declare_parameter("forward_release_speed", cfg.forward_release_speed).value)
        cfg.cooldown = float(self.declare_parameter("cooldown", cfg.cooldown).value)
        cfg.max_attempts = int(self.declare_parameter("max_attempts", cfg.max_attempts).value)
        cfg.give_up_cooldown = float(
            self.declare_parameter("give_up_cooldown", cfg.give_up_cooldown).value)
        cfg.attempt_reset_distance = float(
            self.declare_parameter("attempt_reset_distance", cfg.attempt_reset_distance).value)
        self._fsm = StuckRecovery(cfg)

        # Commanded motion during the manoeuvre. Speeds are modest on purpose: the
        # point is to get unstuck without turning the recovery itself into the
        # next collision. AWSIM caps reverse at ~1.4 m/s whatever we ask for.
        self._reverse_speed = float(self.declare_parameter("reverse_speed", 2.0).value)
        self._reverse_accel = float(self.declare_parameter("reverse_accel", 3.0).value)
        self._forward_speed = float(self.declare_parameter("forward_speed", 4.0).value)
        self._forward_accel = float(self.declare_parameter("forward_accel", 3.0).value)
        # AWSIM applies `acceleration` along the direction of the selected gear, so
        # backing up is a POSITIVE acceleration in reverse gear. Kept as a parameter
        # because it is a property of the vehicle interface, not of this logic: flip
        # it to -1.0 for an interface that expects a signed, gear-independent value.
        self._reverse_accel_sign = float(self.declare_parameter("reverse_accel_sign", 1.0).value)

        # Steering back onto the line.
        self._k_lat = float(self.declare_parameter("steer_k_lat", 0.12).value)
        self._k_psi = float(self.declare_parameter("steer_k_psi", 0.8).value)
        # delta_max (32 deg) x steering_tire_angle_gain (1.639): the same full-lock
        # command the MPC itself is allowed to emit.
        self._max_steer = float(self.declare_parameter("max_steer", 0.9).value)
        self._lookahead = float(self.declare_parameter("lookahead", 6.0).value)

        # Rear check (V2X): do not back into a kart sitting behind us.
        self._use_v2x = bool(self.declare_parameter("use_v2x_rear_check", True).value)
        self._rear_clear_distance = float(
            self.declare_parameter("rear_clear_distance", 4.0).value)
        self._rear_half_width = float(self.declare_parameter("rear_half_width", 1.2).value)
        self._self_ignore_radius = float(self.declare_parameter("self_ignore_radius", 1.5).value)
        self._sensor_stale_sec = float(self.declare_parameter("sensor_stale_sec", 0.5).value)
        self._gear_publish_period = float(
            self.declare_parameter("gear_publish_period", 0.1).value)

        # --- state ---
        self._odom = None
        self._planned = None
        self._planned_t = None
        self._trajectory = None
        self._gear_report = None
        self._v2x = None
        self._last_gear_pub_t = None
        self._requested_gear = None
        self._last_phase = self._fsm.phase

        # --- io ---
        reliable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              history=HistoryPolicy.KEEP_LAST)
        best_effort = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                 history=HistoryPolicy.KEEP_LAST)

        self._pub = self.create_publisher(AckermannControlCommand, "~/output/control_cmd", 1)
        self._gear_pub = self.create_publisher(GearCommand, "/control/command/gear_cmd", 1)
        self._active_pub = self.create_publisher(Bool, "~/output/is_recovering", 1)

        self.create_subscription(AckermannControlCommand, "~/input/control_cmd",
                                 self._on_cmd, 1)
        # The controller's intent, taken upstream of the guard: when the guard is
        # holding the car at 0 m/s for the kart we are wedged against, the input
        # command says "stop" while the controller still wants to race.
        self.create_subscription(AckermannControlCommand, "~/input/planned_cmd",
                                 self._on_planned, 1)
        self.create_subscription(Odometry, "/localization/kinematic_state",
                                 self._on_odom, best_effort)
        self.create_subscription(Trajectory, "/planning/scenario_planning/trajectory",
                                 self._on_trajectory, best_effort)
        self.create_subscription(GearReport, "/vehicle/status/gear_status",
                                 self._on_gear_report, best_effort)
        if self._use_v2x:
            self.create_subscription(V2XVehiclePositionArray, "/v2x/vehicle_positions",
                                     self._on_v2x, reliable)

        self.get_logger().info(
            f"stuck_recovery up (enabled={self._enabled}, "
            f"stuck={cfg.stuck_radius} m / {cfg.stuck_duration} s, "
            f"reverse={cfg.reverse_distance} m, arm_speed={cfg.arm_speed} m/s)")

    # --- subscription callbacks ---
    def _on_odom(self, msg: Odometry) -> None:
        self._odom = msg

    def _on_planned(self, msg: AckermannControlCommand) -> None:
        self._planned = msg
        self._planned_t = self._now()

    def _on_trajectory(self, msg: Trajectory) -> None:
        self._trajectory = [(p.pose.position.x, p.pose.position.y) for p in msg.points]

    def _on_gear_report(self, msg: GearReport) -> None:
        self._gear_report = msg

    def _on_v2x(self, msg: V2XVehiclePositionArray) -> None:
        self._v2x = msg

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
        if msg_t <= 0.0:  # unstamped; accept rather than drop
            return True
        return (self._now() - msg_t) <= self._sensor_stale_sec

    # --- core ---
    def _wants_to_move(self, fallback: AckermannControlCommand) -> bool:
        """Is the controller still asking for speed?

        Read from the pre-guard command when it is available, so that a guard
        holding us at a standstill does not read as "the controller wants to
        stop" -- being held at 0 m/s against an obstacle is exactly the case
        this node exists for.
        """
        cmd = fallback
        if self._planned is not None and self._planned_t is not None \
                and (self._now() - self._planned_t) <= self._sensor_stale_sec:
            cmd = self._planned
        return cmd.longitudinal.speed > self._fsm.cfg.intent_speed

    def _rear_blocked(self, ego_x: float, ego_y: float, yaw: float) -> bool:
        if not self._use_v2x or not self._is_fresh(self._v2x):
            return False
        hx, hy = math.cos(yaw), math.sin(yaw)
        for veh in self._v2x.vehicles:
            rx = veh.position.x - ego_x
            ry = veh.position.y - ego_y
            if math.hypot(rx, ry) < self._self_ignore_radius:
                continue  # this is (approximately) us
            lon = rx * hx + ry * hy
            lat = -rx * hy + ry * hx
            if -self._rear_clear_distance <= lon < 0.0 and abs(lat) <= self._rear_half_width:
                return True
        return False

    def _gear_ready(self, wanted: int) -> bool:
        return self._gear_report is not None and self._gear_report.report == wanted

    def _publish_gear(self, command: int, now: float, force: bool = False) -> None:
        """Hold the requested gear. AWSIM latches the last gear command, but the
        shift is not instantaneous, which is why the manoeuvre has a shift phase
        before it commands any motion."""
        if not force and self._requested_gear == command and self._last_gear_pub_t is not None \
                and (now - self._last_gear_pub_t) < self._gear_publish_period:
            return
        msg = GearCommand()
        msg.stamp = self.get_clock().now().to_msg()
        msg.command = command
        self._gear_pub.publish(msg)
        self._requested_gear = command
        self._last_gear_pub_t = now

    def _steer_towards_path(self, ego_x: float, ego_y: float, yaw: float) -> float:
        """Forward-driving steering command that heads back to the racing line."""
        if not self._trajectory:
            return 0.0
        target = path_target(self._trajectory, ego_x, ego_y, yaw, self._lookahead)
        if target is None:
            return 0.0
        lat, e_psi = target
        return recovery_steer(lat, e_psi, self._k_lat, self._k_psi, self._max_steer)

    def _on_cmd(self, msg: AckermannControlCommand) -> None:
        out = msg

        if not self._enabled or self._odom is None:
            self._pub.publish(out)
            return

        now = self._now()
        p = self._odom.pose.pose
        ego_x, ego_y = p.position.x, p.position.y
        yaw = yaw_from_quaternion(p.orientation)
        v = self._odom.twist.twist.linear.x

        wanted_gear = GearCommand.REVERSE if self._fsm.wants_reverse_gear() else GearCommand.DRIVE
        phase = self._fsm.update(
            now, ego_x, ego_y, v,
            wants_to_move=self._wants_to_move(msg),
            gear_ready=self._gear_ready(wanted_gear),
            rear_blocked=self._rear_blocked(ego_x, ego_y, yaw),
        )
        self._log_transition(phase, ego_x, ego_y)

        if not self._fsm.is_active():
            # Leaving the manoeuvre: make sure the vehicle is back in DRIVE before
            # the controller's command takes effect again.
            if self._requested_gear is not None and self._requested_gear != GearCommand.DRIVE:
                self._publish_gear(GearCommand.DRIVE, now, force=True)
            self._publish_active(False)
            self._pub.publish(out)
            return

        steer = self._steer_towards_path(ego_x, ego_y, yaw)
        stamp = self.get_clock().now().to_msg()
        out.stamp = stamp
        out.longitudinal.stamp = stamp
        out.lateral.stamp = stamp

        if phase in (SHIFT_REVERSE, REVERSE):
            self._publish_gear(GearCommand.REVERSE, now)
            out.lateral.steering_tire_angle = mirrored_for_reverse(steer)
            if phase == REVERSE:
                out.longitudinal.speed = -abs(self._reverse_speed)
                out.longitudinal.acceleration = self._reverse_accel_sign * abs(self._reverse_accel)
            else:
                # Still shifting: wheels already turning, no drive yet.
                out.longitudinal.speed = 0.0
                out.longitudinal.acceleration = 0.0
        else:  # SHIFT_DRIVE / FORWARD
            self._publish_gear(GearCommand.DRIVE, now)
            out.lateral.steering_tire_angle = steer
            if phase == FORWARD:
                out.longitudinal.speed = abs(self._forward_speed)
                out.longitudinal.acceleration = abs(self._forward_accel)
            else:
                out.longitudinal.speed = 0.0
                out.longitudinal.acceleration = 0.0

        self._publish_active(True)
        self._pub.publish(out)

    def _publish_active(self, active: bool) -> None:
        msg = Bool()
        msg.data = active
        self._active_pub.publish(msg)

    def _log_transition(self, phase: str, ego_x: float, ego_y: float) -> None:
        if phase == self._last_phase:
            return
        if phase == SHIFT_REVERSE or (phase == SHIFT_DRIVE and self._last_phase == IDLE):
            self.get_logger().warn(
                f"STUCK detected at ({ego_x:.1f}, {ego_y:.1f}) — "
                f"recovery attempt {self._fsm.attempts}/{self._fsm.cfg.max_attempts}"
                + (" (rear blocked, forward only)" if phase == SHIFT_DRIVE else ""))
        else:
            self.get_logger().info(f"RECOVERY phase -> {phase}")
        self._last_phase = phase


def main(args=None):
    rclpy.init(args=args)
    node = StuckRecoveryNode()
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
