#!/usr/bin/env python3
"""Bounded blade-free forward test with external-IMU yaw-rate hold.

This is a lab tool, not a normal navigation controller. It must be run only
with a supervised mower in the BehaviorTree RECORDING mode. The tool requires
an explicit motion flag, fresh IMU/wheel data, a blade-off hardware status, and
no emergency before it will publish a non-zero command.
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from mowgli_interfaces.msg import Emergency, HighLevelStatus, Status
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu


RECORDING_STATE = 3


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(value, upper))


def yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class ExternalImuYawHold(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("external_imu_yaw_hold_lab")
        self.args = args
        self.publisher = self.create_publisher(TwistStamped, "/cmd_vel_tuning", 10)
        self.create_subscription(Imu, "/imu/data", self.on_imu, 20)
        self.create_subscription(Odometry, "/wheel_odom", self.on_wheel, 20)
        self.create_subscription(Status, "/hardware_bridge/status", self.on_status, 10)
        self.create_subscription(Emergency, "/hardware_bridge/emergency", self.on_emergency, 10)
        self.create_subscription(
            HighLevelStatus,
            "/behavior_tree_node/high_level_status",
            self.on_mode,
            10,
        )
        self.imu_z: float | None = None
        self.filtered_imu_z: float | None = None
        self.imu_seen_at: float | None = None
        self.wheel: Odometry | None = None
        self.wheel_seen_at: float | None = None
        self.status: Status | None = None
        self.emergency: Emergency | None = None
        self.mode: HighLevelStatus | None = None
        self.integral = 0.0
        self.last_control_at: float | None = None

    def on_imu(self, msg: Imu) -> None:
        self.imu_z = float(msg.angular_velocity.z)
        alpha = self.args.gyro_filter_alpha
        self.filtered_imu_z = self.imu_z if self.filtered_imu_z is None else (
            alpha * self.imu_z + (1.0 - alpha) * self.filtered_imu_z
        )
        self.imu_seen_at = time.monotonic()

    def on_wheel(self, msg: Odometry) -> None:
        self.wheel = msg
        self.wheel_seen_at = time.monotonic()

    def on_status(self, msg: Status) -> None:
        self.status = msg

    def on_emergency(self, msg: Emergency) -> None:
        self.emergency = msg

    def on_mode(self, msg: HighLevelStatus) -> None:
        self.mode = msg

    def publish(self, linear_x: float = 0.0, angular_z: float = 0.0) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_footprint"
        msg.twist.linear.x = linear_x
        msg.twist.angular.z = angular_z
        self.publisher.publish(msg)

    def guarded(self) -> tuple[bool, str]:
        now = time.monotonic()
        if self.imu_seen_at is None or now - self.imu_seen_at > self.args.max_imu_age_s:
            return False, "external IMU is missing or stale"
        if self.wheel_seen_at is None or now - self.wheel_seen_at > self.args.max_wheel_age_s:
            return False, "wheel odometry is missing or stale"
        if self.status is None:
            return False, "hardware status is missing"
        if self.status.mow_enabled or self.status.mower_motor_rpm > 1.0:
            return False, "blade is enabled or spinning"
        if self.status.is_charging:
            return False, "mower reports charging"
        if self.emergency is None:
            return False, "emergency status is missing"
        if self.emergency.active_emergency or self.emergency.latched_emergency:
            return False, "emergency is active or latched"
        if self.emergency.lift_warning:
            return False, "lift warning is active"
        if self.mode is None or self.mode.state != RECORDING_STATE:
            return False, "high-level mode is not blade-free RECORDING"
        return True, "ok"

    def wait_for_guards(self) -> None:
        deadline = time.monotonic() + self.args.guard_timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            ok, _ = self.guarded()
            if ok:
                return
            self.publish()
        _, reason = self.guarded()
        raise RuntimeError(f"refusing motion: {reason}")

    def run(self) -> None:
        self.wait_for_guards()
        assert self.wheel is not None
        start = self.wheel.pose.pose.position
        start_yaw = yaw_from_quaternion(self.wheel.pose.pose.orientation)
        started_at = time.monotonic()
        self.last_control_at = started_at
        self.get_logger().info(
            f"Starting bounded yaw-hold test: {self.args.distance_m:.2f} m "
            f"at <= {self.args.speed_mps:.2f} m/s"
        )
        while time.monotonic() - started_at < self.args.max_duration_s:
            rclpy.spin_once(self, timeout_sec=0.02)
            ok, reason = self.guarded()
            if not ok:
                raise RuntimeError(f"stopping: {reason}")
            assert self.wheel is not None and self.filtered_imu_z is not None
            current = self.wheel.pose.pose.position
            distance = math.hypot(current.x - start.x, current.y - start.y)
            if distance >= self.args.distance_m:
                break
            now = time.monotonic()
            dt = clamp(now - (self.last_control_at or now), 0.0, 0.25)
            self.last_control_at = now
            # External bridge documents +Z as CCW/left. A positive measured
            # rate therefore needs a negative/right command to hold heading.
            rate_error = -self.filtered_imu_z
            self.integral = clamp(
                self.integral + rate_error * dt,
                -self.args.integral_limit,
                self.args.integral_limit,
            )
            angular_z = clamp(
                self.args.kp * rate_error + self.args.ki * self.integral,
                -self.args.max_wz_rad_s,
                self.args.max_wz_rad_s,
            )
            elapsed = now - started_at
            speed = min(self.args.speed_mps, self.args.speed_mps * elapsed / self.args.ramp_s)
            self.publish(speed, angular_z)
            time.sleep(0.03)
        assert self.wheel is not None
        end = self.wheel.pose.pose
        dx = end.position.x - start.x
        dy = end.position.y - start.y
        end_yaw = yaw_from_quaternion(end.orientation)
        local_x = math.cos(start_yaw) * dx + math.sin(start_yaw) * dy
        local_y = -math.sin(start_yaw) * dx + math.cos(start_yaw) * dy
        yaw_deg = math.degrees(math.atan2(math.sin(end_yaw - start_yaw), math.cos(end_yaw - start_yaw)))
        self.get_logger().info(
            f"RESULT distance={math.hypot(dx, dy):.3f} m local_x={local_x:.3f} m "
            f"local_y={local_y:.3f} m wheel_yaw={yaw_deg:.2f} deg"
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--enable-motion", action="store_true", help="required explicit motion consent")
    parser.add_argument("--distance-m", type=float, default=0.8)
    parser.add_argument("--speed-mps", type=float, default=0.20)
    parser.add_argument("--max-duration-s", type=float, default=20.0)
    parser.add_argument("--kp", type=float, default=2.0)
    parser.add_argument("--ki", type=float, default=0.35)
    parser.add_argument("--max-wz-rad-s", type=float, default=0.25)
    parser.add_argument("--integral-limit", type=float, default=0.35)
    parser.add_argument("--gyro-filter-alpha", type=float, default=0.35,
                        help="EMA weight for external gyro rate")
    parser.add_argument("--ramp-s", type=float, default=1.5)
    parser.add_argument("--max-imu-age-s", type=float, default=0.35)
    parser.add_argument("--max-wheel-age-s", type=float, default=0.35)
    parser.add_argument("--guard-timeout-s", type=float, default=8.0)
    args = parser.parse_args()
    if not args.enable_motion:
        parser.error("--enable-motion is required")
    if not 0.1 <= args.distance_m <= 5.0:
        parser.error("--distance-m must be between 0.1 and 5.0")
    if not 0.05 <= args.speed_mps <= 0.25:
        parser.error("--speed-mps must be between 0.05 and 0.25")
    return args


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = ExternalImuYawHold(args)
    try:
        node.run()
        return 0
    except Exception as exc:
        node.get_logger().error(str(exc))
        return 2
    finally:
        for _ in range(10):
            node.publish()
            rclpy.spin_once(node, timeout_sec=0.02)
            time.sleep(0.03)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
