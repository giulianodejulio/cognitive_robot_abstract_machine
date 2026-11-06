# %% imports

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import StrEnum

import numpy as np
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from tf2_ros import Buffer, TransformListener

from experiments.tiago_monitor.status import (
    MonitorSnapshot,
    StreamCondition,
    StreamStatus,
)
from experiments.tiago_monitor.view import RobotView
from semantic_digital_twin.adapters.ros.input_synchronization import (
    PendingJointStateSynchronizer,
    SubscribedBasePoseSource,
)
from semantic_digital_twin.adapters.ros.latest_message_subscriber import (
    LatestMessageSubscriber,
)
from semantic_digital_twin.adapters.ros.lidar import SubscribedLidarSource
from semantic_digital_twin.adapters.urdf import URDFParser
from semantic_digital_twin.datastructures.field_of_view import FieldOfView
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.input_synchronization import (
    InputSynchronizer,
    WorldStateInputs,
)
from semantic_digital_twin.robots.input_source import RobotTopic
from semantic_digital_twin.robots.tiago import TiagoTopic
from semantic_digital_twin.robots.tiago_single_arm import TiagoSingleArm
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import DifferentialDrive
from semantic_digital_twin.world_description.world_entity import Body

# %% input ownership


class MonitorName(StrEnum):
    """
    ROS names specific to this observation process.
    """

    NODE = "tiago_read_only_monitor"
    ODOMETRY_FRAME = "odom"
    CAMERA_INFORMATION = "/head_front_camera/rgb/camera_info"


@dataclass
class CameraInformationSource(LatestMessageSubscriber[CameraInfo]):
    """
    The most recent camera calibration published by the robot.
    """


@dataclass
class ObservedInput:
    """
    A subscriber paired with the freshness of its last sample.
    """

    subscriber: (
        PendingJointStateSynchronizer
        | SubscribedBasePoseSource
        | SubscribedLidarSource
        | CameraInformationSource
    )
    """
    The input receiving messages without publishing robot commands.
    """

    status: StreamStatus = field(init=False)
    """
    The condition of the last sample, including consumed joint messages.
    """

    def __post_init__(self) -> None:
        self.status = StreamStatus(topic=str(self.subscriber.topic_name))

    def update(self, current_time: float, maximum_age: float) -> None:
        """
        Refresh the last sample time and classify its age.
        """
        message = self.subscriber.latest_message
        if message is not None:
            stamp = message.header.stamp
            self.status.sample_time = float(Time.from_msg(stamp).nanoseconds) / 1e9
        self.status.update(current_time, maximum_age)


# %% observed digital twin


@dataclass
class RobotObserver:
    """
    An uncommanded digital twin following the single-arm TIAGo's inputs.
    """

    node: Node
    """
    The ROS node used only for subscriptions.
    """

    maximum_age: float = 2.0
    """
    The tolerated sample age, in seconds.
    """

    world: World = field(init=False)
    """
    The model receiving joint positions and laser odometry.
    """

    robot: TiagoSingleArm = field(init=False)
    """
    The semantic robot with all hardware interfaces disabled.
    """

    inputs: WorldStateInputs = field(init=False)
    """
    The existing framework synchronizers applied to this world.
    """

    observed_inputs: list[ObservedInput] = field(init=False, default_factory=list)
    """
    The subscribers and their freshness records.
    """

    lidar: SubscribedLidarSource = field(init=False)
    """
    The source of the real scanner's readings.
    """

    camera: CameraInformationSource = field(init=False)
    """
    The source of the camera intrinsics and optical frame.
    """

    transforms: Buffer = field(init=False)
    """
    The received transform tree used to compare model geometry.
    """

    listener: TransformListener = field(init=False)
    """
    The subscription owner populating the transform buffer.
    """

    lidar_transform: StreamStatus = field(init=False)
    """
    The freshness of the transform from odometry to the scanner.
    """

    def __post_init__(self) -> None:
        self.world = URDFParser.from_file(TiagoSingleArm.get_ros_file_path()).parse()
        self.robot = TiagoSingleArm.from_world(self.world)
        with self.world.modify_world():
            drive = DifferentialDrive.create_with_dofs(
                world=self.world,
                parent=Body(name=PrefixedName(MonitorName.ODOMETRY_FRAME)),
                child=self.world.root,
            )
            self.world.add_connection(drive)
        self.inputs = WorldStateInputs(world=self.world)
        joints = PendingJointStateSynchronizer(
            world=self.world, node=self.node, topic_name=RobotTopic.JOINT_STATES
        )
        base = SubscribedBasePoseSource(
            world=self.world,
            node=self.node,
            topic_name=TiagoTopic.ODOMETRY,
            connection=drive,
        )
        self.inputs.add_input(joints)
        self.inputs.add_input(base)
        self.robot.mobile_base.source = base
        self.lidar = SubscribedLidarSource(node=self.node, topic_name=TiagoTopic.LIDAR)
        self.robot.mobile_base.lidar.source = self.lidar
        self.camera = CameraInformationSource(
            node=self.node, topic_name=MonitorName.CAMERA_INFORMATION
        )
        self.observed_inputs = [
            ObservedInput(subscriber=source)
            for source in [joints, base, self.lidar, self.camera]
        ]
        self.transforms = Buffer()
        self.listener = TransformListener(self.transforms, self.node)
        self.lidar_transform = StreamStatus(
            topic=f"{MonitorName.ODOMETRY_FRAME} → {self.robot.mobile_base.lidar.root.name.name}"
        )

    def synchronize(self) -> None:
        """
        Apply received positions and sensor parameters without commanding hardware.
        """
        current_time = float(self.node.get_clock().now().nanoseconds) / 1e9
        for observed in self.observed_inputs:
            observed.update(current_time, self.maximum_age)
        self.inputs.synchronize()
        if self.lidar.has_reading:
            self.robot.mobile_base.lidar.scan_pattern = (
                self.robot.mobile_base.lidar.get_lidar_reading().scan_pattern
            )
        information = self.camera.latest_message
        if information is None:
            return
        if information.width <= 0 or information.height <= 0:
            return
        if information.k[0] <= 0 or information.k[4] <= 0:
            return
        camera = self.robot.get_default_camera()
        camera.root = self.world.get_body_by_name(information.header.frame_id)
        camera.field_of_view = FieldOfView(
            horizontal_angle=2 * math.atan(information.width / (2 * information.k[0])),
            vertical_angle=2 * math.atan(information.height / (2 * information.k[4])),
        )

    def compare_lidar_transform(self, current_time: float) -> float | None:
        """
        Compare the model scanner position to the latest received odometry TF.
        """
        scanner = self.robot.mobile_base.lidar.root
        if not self.transforms.can_transform(
            MonitorName.ODOMETRY_FRAME, scanner.name.name, Time()
        ):
            self.lidar_transform.update(current_time, self.maximum_age)
            return None
        transform = self.transforms.lookup_transform(
            MonitorName.ODOMETRY_FRAME, scanner.name.name, Time()
        )
        self.lidar_transform.sample_time = (
            float(Time.from_msg(transform.header.stamp).nanoseconds) / 1e9
        )
        self.lidar_transform.update(current_time, self.maximum_age)
        position = transform.transform.translation
        return float(
            np.linalg.norm(
                scanner.global_transform.to_np()[:3, 3]
                - np.array([position.x, position.y, position.z])
            )
        )

    def observe(self, view: RobotView) -> MonitorSnapshot:
        """
        Synchronize inputs and collect one browser snapshot of the observed world.
        """
        self.synchronize()
        current_time = float(self.node.get_clock().now().nanoseconds) / 1e9
        error = self.compare_lidar_transform(current_time)
        return MonitorSnapshot(
            generated_at=current_time,
            maximum_age=self.maximum_age,
            streams=[observed.status for observed in self.observed_inputs]
            + [self.lidar_transform],
            poses=view.get_mesh_poses(),
            lidar_translation_error=error,
            penetrating_contacts=self.count_penetrating_contacts(),
        )

    def count_penetrating_contacts(self) -> int | None:
        """
        Evaluate configured contacts only when joint and base samples are current.
        """
        if any(
            observed.status.condition != StreamCondition.CURRENT
            for observed in self.observed_inputs
            if isinstance(observed.subscriber, InputSynchronizer)
        ):
            return None
        manager = self.world.collision_manager
        manager.update_collision_matrix()
        contacts = manager.compute_collisions().contacts
        return sum(contact.distance <= 0 for contact in contacts)
