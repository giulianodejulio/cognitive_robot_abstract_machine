"""
Publish a calibrated doorway and wheel-based robot view on an isolated ROS domain.
"""

# %% imports
from __future__ import annotations

import argparse
import math
from dataclasses import dataclass, field
from enum import StrEnum
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.context import Context
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from experiments.robocat_safety import RobocatEnvironment
from semantic_digital_twin.robots.robocat import Robocat


# %% frame and topic names
class ViewFrame(StrEnum):
    """
    Frames belonging to the wheel-based doorway view.
    """

    ODOMETRY = "odom"
    BASE = "base_footprint"
    DOORWAY = "elevator_doorway"


class ViewTopic(StrEnum):
    """
    Feedback and visualization topics; no motion command topics are created.
    """

    SCAN = "/scan"
    WHEELS = "/mobile_base_controller/odom"
    TRANSFORMS = "/tf"
    STATIC_TRANSFORMS = "/tf_static"
    DESCRIPTION = "/robot_description"


# %% frozen doorway calibration
@dataclass(frozen=True)
class CalibratedDoorway:
    """
    A doorway pose fixed in the wheel odometry coordinate system.
    """

    x: float
    """
    Doorway x coordinate in meters.
    """

    y: float
    """
    Doorway y coordinate in meters.
    """

    yaw: float
    """
    Heading of the normal pointing into the cabin, in radians.
    """

    def transform(self) -> TransformStamped:
        """
        Represent the fixed doorway as a static ROS transform.
        """
        result = TransformStamped()
        result.header.frame_id = ViewFrame.ODOMETRY
        result.child_frame_id = ViewFrame.DOORWAY
        result.transform.translation.x = self.x
        result.transform.translation.y = self.y
        result.transform.rotation.z = math.sin(self.yaw / 2)
        result.transform.rotation.w = math.cos(self.yaw / 2)
        return result


# %% feedback-only isolated visualization
@dataclass
class DoorwayView:
    """
    Transfer robot observations to a separate view with coherent wheel-based TF.
    """

    input_node: Node
    """
    Subscriber node on the feedback-only Jazzy domain.
    """
    output_node: Node
    """
    Publisher node on the isolated visualization domain.
    """
    calibration: CalibratedDoorway
    """
    Doorway pose calibrated against wheel odometry.
    """

    publishers: dict[ViewTopic, Publisher] = field(init=False, default_factory=dict)
    """
    Publishers restricted to scan, TF and robot description.
    """

    static_frames: dict[str, TransformStamped] = field(init=False, default_factory=dict)
    """
    Robot static frames plus the calibrated doorway.
    """

    def __post_init__(self) -> None:
        durable = QoSProfile(depth=20, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publishers = {
            ViewTopic.SCAN: self.output_node.create_publisher(
                LaserScan, ViewTopic.SCAN, qos_profile_sensor_data
            ),
            ViewTopic.TRANSFORMS: self.output_node.create_publisher(
                TFMessage, ViewTopic.TRANSFORMS, 100
            ),
            ViewTopic.STATIC_TRANSFORMS: self.output_node.create_publisher(
                TFMessage, ViewTopic.STATIC_TRANSFORMS, durable
            ),
            ViewTopic.DESCRIPTION: self.output_node.create_publisher(
                String, ViewTopic.DESCRIPTION, durable
            ),
        }
        self.static_frames[ViewFrame.DOORWAY] = self.calibration.transform()
        self.input_node.create_subscription(
            LaserScan,
            ViewTopic.SCAN,
            self.publishers[ViewTopic.SCAN].publish,
            qos_profile_sensor_data,
        )
        self.input_node.create_subscription(
            Odometry, ViewTopic.WHEELS, self.receive_wheels, qos_profile_sensor_data
        )
        self.input_node.create_subscription(
            TFMessage, ViewTopic.TRANSFORMS, self.receive_transforms, 100
        )
        self.input_node.create_subscription(
            TFMessage,
            ViewTopic.STATIC_TRANSFORMS,
            self.receive_static_transforms,
            durable,
        )

    def receive_wheels(self, message: Odometry) -> None:
        """
        Place the base from wheel feedback without forwarding native DLO TF.
        """
        if (
            message.header.frame_id != ViewFrame.ODOMETRY
            or message.child_frame_id != ViewFrame.BASE
        ):
            return
        pose = message.pose.pose
        values = (
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        if not all(math.isfinite(value) for value in values):
            return
        if abs(sum(value * value for value in values[3:]) - 1) > 0.05:
            return
        transform = TransformStamped()
        transform.header = message.header
        transform.child_frame_id = ViewFrame.BASE
        transform.transform.translation.x = pose.position.x
        transform.transform.translation.y = pose.position.y
        transform.transform.translation.z = pose.position.z
        transform.transform.rotation = pose.orientation
        self.publishers[ViewTopic.TRANSFORMS].publish(TFMessage(transforms=[transform]))

    def receive_transforms(self, message: TFMessage) -> None:
        """
        Forward relative robot transforms, excluding native odometry and localization.
        """
        transforms = [
            transform
            for transform in message.transforms
            if transform.child_frame_id
            not in (ViewFrame.ODOMETRY, ViewFrame.BASE, ViewFrame.DOORWAY)
        ]
        if transforms:
            self.publishers[ViewTopic.TRANSFORMS].publish(
                TFMessage(transforms=transforms)
            )

    def receive_static_transforms(self, message: TFMessage) -> None:
        """
        Retain static robot transforms alongside the independent doorway calibration.
        """
        for transform in message.transforms:
            if transform.child_frame_id not in (
                ViewFrame.ODOMETRY,
                ViewFrame.BASE,
                ViewFrame.DOORWAY,
            ):
                self.static_frames[transform.child_frame_id] = transform
        self.publishers[ViewTopic.STATIC_TRANSFORMS].publish(
            TFMessage(transforms=list(self.static_frames.values()))
        )


# %% passive CLI
def main() -> None:
    """
    Expose the calibrated frame and observations without hardware control.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--calibration", type=Path, required=True)
    options = parser.parse_args()
    RobocatEnvironment().validate()
    calibration = CalibratedDoorway(**yaml.safe_load(options.calibration.read_text()))
    rclpy.init()
    output_context = Context()
    rclpy.init(context=output_context, domain_id=104)
    input_node = Node("robocat_doorway_observer", enable_rosout=False)
    output_node = Node("robocat_doorway_visualization", context=output_context)
    view = DoorwayView(input_node, output_node, calibration)
    view.publishers[ViewTopic.DESCRIPTION].publish(
        String(data=Path(Robocat.get_ros_file_path()).read_text())
    )
    input_node.create_timer(
        1.0,
        lambda: view.publishers[ViewTopic.STATIC_TRANSFORMS].publish(
            TFMessage(transforms=list(view.static_frames.values()))
        ),
    )
    print(
        "VIEW READY: feedback 103 -> view 104; wheel odom; elevator_doorway; no commands",
        flush=True,
    )
    rclpy.spin(input_node)


if __name__ == "__main__":
    main()
