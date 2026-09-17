#!/usr/bin/env python3
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_auto_vehicle_msgs.msg import GearCommand, VelocityReport
from std_msgs.msg import Int32

from collision_detector import CollisionDetector
from stuck_recovery import StuckRecovery
from tiny_lidar_net_controller_core import TinyLidarNetCore


class TinyLidarNetNode(Node):
    """ROS 2 Node for TinyLidarNet autonomous driving control.

    This node subscribes to LaserScan messages, processes them using the
    TinyLidarNetCore logic, and publishes AckermannControlCommand messages.
    """

    def __init__(self):
        super().__init__('tiny_lidar_net_node')

        # --- Parameter Declaration ---
        self.declare_parameter('log_interval_sec', 5.0)
        self.declare_parameter('model.input_dim', 1080)
        self.declare_parameter('model.output_dim', 2)
        self.declare_parameter('model.architecture', 'large')
        self.declare_parameter('model.n_frames', 1)
        self.declare_parameter('model.ckpt_path', '')
        self.declare_parameter('max_range', 30.0)
        self.declare_parameter('acceleration', 0.1)
        self.declare_parameter('max_speed', 8.34)
        self.declare_parameter('speed_kp', 1.0)
        self.declare_parameter('recovery.enabled', True)
        self.declare_parameter('recovery.stuck_speed', 0.5)
        self.declare_parameter('recovery.stuck_duration', 2.0)
        self.declare_parameter('recovery.collision_stuck_duration', 0.5)
        self.declare_parameter('recovery.collision_window', 3.0)
        self.declare_parameter('recovery.release_speed', 0.8)
        self.declare_parameter('recovery.forward_duration', 1.5)
        self.declare_parameter('recovery.forward_attempts', 2)
        self.declare_parameter('recovery.forward_attempts_when_blocked', 1)
        self.declare_parameter('recovery.front_blocked_range', 1.2)
        self.declare_parameter('recovery.rear_wait_duration', 2.0)
        self.declare_parameter('recovery.reverse_duration', 1.2)
        self.declare_parameter('recovery.reverse_probe_duration', 1.0)
        self.declare_parameter('recovery.reverse_move_speed', 0.15)
        self.declare_parameter('recovery.max_escape_cycles', 3)
        self.declare_parameter('recovery.cooldown_duration', 2.0)
        self.declare_parameter('recovery.sidestep_duration', 0.0)
        self.declare_parameter('recovery.sidestep_steer_gain', 0.5)
        self.declare_parameter('recovery.forward_accel', 1.2)
        self.declare_parameter('recovery.reverse_accel', 1.2)
        self.declare_parameter('collision.enabled', True)
        self.declare_parameter('collision.condition_topic',
                               '/aichallenge/pitstop/condition')
        self.declare_parameter('collision.damage_threshold', 30)
        self.declare_parameter('collision.damage_window', 1.0)
        self.declare_parameter('collision.decel_threshold', 4.0)
        self.declare_parameter('collision.min_impact_speed', 1.0)
        self.declare_parameter('collision.contact_range', 0.8)
        self.declare_parameter('collision.contact_speed', 0.5)
        self.declare_parameter('collision.contact_duration', 0.5)
        self.declare_parameter('collision.refractory_duration', 1.5)
        self.declare_parameter('collision.missing_topic_warn_sec', 15.0)
        self.declare_parameter('control_mode', 'ai')
        self.declare_parameter('debug', False)

        # --- Initialization ---
        input_dim = self.get_parameter('model.input_dim').value
        output_dim = self.get_parameter('model.output_dim').value
        architecture = self.get_parameter('model.architecture').value
        n_frames = self.get_parameter('model.n_frames').value
        ckpt_path = self.get_parameter('model.ckpt_path').value
        max_range = self.get_parameter('max_range').value
        acceleration = self.get_parameter('acceleration').value
        max_speed = self.get_parameter('max_speed').value
        speed_kp = self.get_parameter('speed_kp').value

        recovery = None
        if self.get_parameter('recovery.enabled').value:
            recovery = StuckRecovery(
                stuck_speed=self.get_parameter('recovery.stuck_speed').value,
                stuck_duration=self.get_parameter('recovery.stuck_duration').value,
                collision_stuck_duration=self.get_parameter(
                    'recovery.collision_stuck_duration').value,
                collision_window=self.get_parameter('recovery.collision_window').value,
                release_speed=self.get_parameter('recovery.release_speed').value,
                forward_duration=self.get_parameter('recovery.forward_duration').value,
                forward_attempts=self.get_parameter('recovery.forward_attempts').value,
                forward_attempts_when_blocked=self.get_parameter(
                    'recovery.forward_attempts_when_blocked').value,
                front_blocked_range=self.get_parameter(
                    'recovery.front_blocked_range').value,
                rear_wait_duration=self.get_parameter('recovery.rear_wait_duration').value,
                reverse_duration=self.get_parameter('recovery.reverse_duration').value,
                reverse_probe_duration=self.get_parameter(
                    'recovery.reverse_probe_duration').value,
                reverse_move_speed=self.get_parameter(
                    'recovery.reverse_move_speed').value,
                max_escape_cycles=self.get_parameter('recovery.max_escape_cycles').value,
                cooldown_duration=self.get_parameter('recovery.cooldown_duration').value,
                sidestep_duration=self.get_parameter('recovery.sidestep_duration').value,
                sidestep_steer_gain=self.get_parameter('recovery.sidestep_steer_gain').value,
                forward_accel=self.get_parameter('recovery.forward_accel').value,
                reverse_accel=self.get_parameter('recovery.reverse_accel').value,
            )

        # 壁・カートへの衝突検知。ダメージ topic が来ていればそれを使い、
        # 来ていなければ車輪速と LiDAR からの推定に切り替わる。
        collision_detector = None
        if self.get_parameter('collision.enabled').value:
            collision_detector = CollisionDetector(
                damage_threshold=self.get_parameter('collision.damage_threshold').value,
                damage_window=self.get_parameter('collision.damage_window').value,
                decel_threshold=self.get_parameter('collision.decel_threshold').value,
                min_impact_speed=self.get_parameter('collision.min_impact_speed').value,
                contact_range=self.get_parameter('collision.contact_range').value,
                contact_speed=self.get_parameter('collision.contact_speed').value,
                contact_duration=self.get_parameter('collision.contact_duration').value,
                refractory_duration=self.get_parameter(
                    'collision.refractory_duration').value,
            )
        control_mode = self.get_parameter('control_mode').value
        
        self.debug = self.get_parameter('debug').value
        self.log_interval = self.get_parameter('log_interval_sec').value

        try:
            self.core = TinyLidarNetCore(
                input_dim=input_dim,
                output_dim=output_dim,
                architecture=architecture,
                ckpt_path=ckpt_path,
                acceleration=acceleration,
                control_mode=control_mode,
                max_range=max_range,
                n_frames=n_frames,
                max_speed=max_speed,
                speed_kp=speed_kp,
                recovery=recovery,
                collision_detector=collision_detector
            )
            self.get_logger().info(
                f"Core initialized. Arch: {architecture}, MaxRange: {max_range}, "
                f"Frames: {n_frames}"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to initialize core logic: {e}")
            raise e

        # --- Communication Setup ---
        self.inference_times = []
        self.last_log_time = self.get_clock().now()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.sub_scan = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, qos
        )
        # control_mode "speed" では、目標速度との差から加速度を作るために現在速度が要る。
        self.sub_velocity = self.create_subscription(
            VelocityReport, "/vehicle/status/velocity_status", self.velocity_callback, qos
        )
        self.pub_control = self.create_publisher(
            AckermannControlCommand, "/control/command/control_cmd", 1
        )
        # 車両ダメージ。QoS は multi_purpose_mpc_ros の購読側に合わせて既定のまま。
        self.condition_topic = self.get_parameter('collision.condition_topic').value
        self.missing_topic_warn_sec = self.get_parameter(
            'collision.missing_topic_warn_sec').value
        self.sub_condition = None
        self.condition_received = False
        self.missing_topic_warned = False
        self.node_start_time = time.monotonic()
        if collision_detector is not None:
            self.sub_condition = self.create_subscription(
                Int32, self.condition_topic, self.condition_callback, 1
            )

        # 復帰時のみギアを切り替える。通常走行では DRIVE のまま触らない。
        self.pub_gear = self.create_publisher(
            GearCommand, "/control/command/gear_cmd", 1
        )
        self.last_gear = None
        self.last_recovery_state = None
        self.logged_collisions = 0

        self.get_logger().info("TinyLidarNetNode is ready.")

    def velocity_callback(self, msg: VelocityReport):
        """Feeds the measured vehicle speed to the core logic.

        Args:
            msg (VelocityReport): The incoming vehicle velocity status.
        """
        self.core.set_current_speed(msg.longitudinal_velocity)

    def condition_callback(self, msg: Int32):
        """車両ダメージを衝突検知に流し込み、衝突を検知したらログに出す。

        Args:
            msg (Int32): /aichallenge/pitstop/condition の値（たまったダメージ量）。
        """
        self.condition_received = True
        self.core.update_condition(time.monotonic(), msg.data)

    def scan_callback(self, msg: LaserScan):
        """Callback for LaserScan subscription.

        Processes the scan data via the core logic and publishes a control command.

        Args:
            msg (LaserScan): The incoming ROS 2 LaserScan message.
        """
        start_time = time.monotonic()

        # 1. Convert ROS message to Numpy
        # We pass the raw array; the core logic handles NaN/Inf and normalization.
        ranges = np.array(msg.ranges, dtype=np.float32)

        # 2. Process via Core Logic
        # 角度情報は復帰ロジックが左右の空きを測るのに使う。
        accel, steer = self.core.process(
            ranges,
            now=start_time,
            angle_min=msg.angle_min,
            angle_increment=msg.angle_increment,
        )
        self._publish_gear(self.core.reverse_requested)
        self._log_collision()
        self._warn_if_condition_missing()
        self._log_recovery_state()

        # 3. Publish Command
        cmd = AckermannControlCommand()
        cmd.stamp = self.get_clock().now().to_msg()
        cmd.longitudinal.acceleration = float(accel)
        cmd.lateral.steering_tire_angle = float(steer)
        self.pub_control.publish(cmd)

        # 4. Debug Logging
        if self.debug:
            duration_ms = (time.monotonic() - start_time) * 1000.0
            self.inference_times.append(duration_ms)
            self._log_performance_metrics()

    def _publish_gear(self, reverse: bool):
        """必要なときだけギア指令を出す。

        毎周期 publish すると無駄なので、切り替わった瞬間だけ送る。

        Args:
            reverse (bool): True なら REVERSE、False なら DRIVE を要求する。
        """
        gear = GearCommand.REVERSE if reverse else GearCommand.DRIVE
        if gear == self.last_gear:
            return
        msg = GearCommand()
        msg.stamp = self.get_clock().now().to_msg()
        msg.command = gear
        self.pub_gear.publish(msg)
        self.last_gear = gear

    def _log_collision(self):
        """衝突を検知した瞬間だけログに出す。

        検知の経路（ダメージ topic / センサ推定）によらず件数の増加で拾うので、
        ここ 1 か所で済む。
        """
        detector = self.core.collision_detector
        if detector is None or detector.event_count == self.logged_collisions:
            return
        self.logged_collisions = detector.event_count
        event = self.core.last_collision
        self.get_logger().warn(
            f"collision detected: {event.kind} (#{detector.event_count}) {event.detail}"
        )

    def _warn_if_condition_missing(self):
        """ダメージ topic が来ていないことを一度だけ警告する。

        実車や、この topic を出さない構成では衝突検知が働かない。黙って
        効かないより、復帰が低速継続の判定だけになると分かるほうがよい。
        """
        if (self.sub_condition is None or self.condition_received
                or self.missing_topic_warned):
            return
        if (time.monotonic() - self.node_start_time) < self.missing_topic_warn_sec:
            return
        self.missing_topic_warned = True
        self.get_logger().warn(
            f"no message on {self.condition_topic} after "
            f"{self.missing_topic_warn_sec:.0f}s: "
            f"collision detection falls back to wheel speed and LiDAR."
        )

    def _log_recovery_state(self):
        """復帰の状態遷移をログに出す。走行後に発火回数と経緯を追えるようにする。"""
        if self.core.recovery is None:
            return
        state = self.core.recovery.state
        if state is self.last_recovery_state:
            return
        self.get_logger().info(
            f"stuck recovery: {state.value} "
            f"(trigger #{self.core.recovery.trigger_count} "
            f"by {self.core.recovery.trigger_reason}, "
            f"cycle={self.core.recovery.escape_cycle}, "
            f"speed={self.core.current_speed:.2f} m/s, "
            f"peak={self.core.recovery.peak_speed_in_recovery:.2f} m/s, "
            f"reverse_sign={self.core.recovery.reverse_sign:+.0f})"
        )
        self.last_recovery_state = state

    def _log_performance_metrics(self):
        """Logs internal performance metrics at fixed intervals."""
        now = self.get_clock().now()
        elapsed_sec = (now - self.last_log_time).nanoseconds / 1e9

        if elapsed_sec > self.log_interval:
            if self.inference_times:
                avg_time = np.mean(self.inference_times)
                max_time = np.max(self.inference_times)
                fps = 1000.0 / avg_time if avg_time > 0 else 0.0

                self.get_logger().info(
                    f"DEBUG: Avg Inference: {avg_time:.2f}ms ({fps:.2f}Hz) | "
                    f"Max: {max_time:.2f}ms"
                )
                self.inference_times.clear()
            
            self.last_log_time = now


def main(args=None):
    rclpy.init(args=args)
    node = TinyLidarNetNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
