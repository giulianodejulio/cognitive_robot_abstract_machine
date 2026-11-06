# %% imports
from __future__ import annotations

import math
import argparse
import os
import time
from dataclasses import dataclass, field
from enum import StrEnum

from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from scripts.tiago_giskard_goal import BaseFeedback


# %% safety streams
@dataclass
class RobocatEnvironment:
    """
    Require the isolated Jazzy domain before creating any ROS node.
    """

    domain_id: int = 103
    """
    The local domain connected to Robocat through the filtered bridge.
    """

    def validate(self) -> None:
        """
        Reject direct Jazzy participation in the native Humble graph.
        """
        if os.environ.get("ROS_DOMAIN_ID") != str(self.domain_id):
            argparse.ArgumentParser().error(
                f"Robocat requires ROS_DOMAIN_ID={self.domain_id}; use the dedicated launcher"
            )


class RobocatTopic(StrEnum):
    """
    Feedback streams required for real Robocat base execution.
    """

    EMERGENCY = "/power/is_emergency"
    JOYSTICK = "/joy_priority"
    ODOMETRY = "/direct_laser_odometry/odom"
    WHEELS = "/mobile_base_controller/odom"
    JOINTS = "/joint_states"
    DIAGNOSTICS = "/diagnostics"


@dataclass
class RobocatFeedback(BaseFeedback):
    """
    Robocat QoS and fresh interlocks on top of the established base preflight.
    """

    received_at: dict[RobocatTopic, float] = field(default_factory=dict, init=False)
    """
    Monotonic receipt times of periodic safety streams.
    """

    maximum_age: float = 2.0
    """
    Maximum tolerated age of periodic safety feedback, in seconds.
    """

    def __post_init__(self) -> None:
        self.subscriptions = [
            self.node.create_subscription(
                Bool,
                RobocatTopic.EMERGENCY,
                self.receive_emergency,
                qos_profile_sensor_data,
            ),
            self.node.create_subscription(
                Bool,
                RobocatTopic.JOYSTICK,
                self.receive_joy,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            ),
            self.node.create_subscription(
                Odometry,
                RobocatTopic.ODOMETRY,
                self.receive_odometry,
                qos_profile_sensor_data,
            ),
            self.node.create_subscription(
                Odometry,
                RobocatTopic.WHEELS,
                self.receive_wheels,
                qos_profile_sensor_data,
            ),
            self.node.create_subscription(
                JointState,
                RobocatTopic.JOINTS,
                self.receive_joints,
                qos_profile_sensor_data,
            ),
            self.node.create_subscription(
                DiagnosticArray,
                RobocatTopic.DIAGNOSTICS,
                self.receive_diagnostics,
                qos_profile_sensor_data,
            ),
        ]

    def receive_emergency(self, message: Bool) -> None:
        super().receive_emergency(message)
        self.received_at[RobocatTopic.EMERGENCY] = time.monotonic()

    def receive_diagnostics(self, message: DiagnosticArray) -> None:
        super().receive_diagnostics(message)
        if any("twist_mux" in status.name for status in message.status):
            self.received_at[RobocatTopic.DIAGNOSTICS] = time.monotonic()

    def blocking_reasons(self, require_free_mux: bool = True) -> list[str]:
        reasons = super().blocking_reasons(require_free_mux)
        now = time.monotonic()
        for topic in (RobocatTopic.EMERGENCY, RobocatTopic.DIAGNOSTICS):
            receipt = self.received_at.get(topic)
            if receipt is None or now - receipt > self.maximum_age:
                reasons.append(f"{topic}: safety stream missing or stale")
        if not require_free_mux and self.mux.get("current priority") not in ("0", "10"):
            reasons.append("mux owned by another command source")
        if self.wheel_odometry is None:
            reasons.append("missing wheel odometry")
        else:
            message = self.wheel_odometry
            clock = self.node.get_clock().now().nanoseconds / 1e9
            stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
            if abs(clock - stamp) > self.maximum_age:
                reasons.append("wheel odometry stale")
            if not all(
                math.isfinite(value)
                for value in (
                    message.twist.twist.linear.x,
                    message.twist.twist.angular.z,
                )
            ):
                reasons.append("wheel velocity nonfinite")
        if self.odometry is not None:
            pose = self.odometry.pose.pose
            quaternion = pose.orientation
            norm = sum(
                value * value
                for value in (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
            )
            if not math.isfinite(norm) or abs(norm - 1) > 0.05:
                reasons.append("invalid odometry quaternion")
            if (
                self.odometry.header.frame_id != "odom"
                or self.odometry.child_frame_id != "base_footprint"
            ):
                reasons.append("unexpected odometry frames")
        return reasons
