# %% imports

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass, field
from enum import StrEnum
from pathlib import Path

import numpy as np
from rclpy.time import Time
from typing_extensions import TYPE_CHECKING

from giskardpy.middleware.ros2 import rospy
from giskardpy.middleware.ros2.command_publishing import (
    DriveVelocityCommandPublisher,
    StateVelocityReader,
)
from giskardpy.middleware.ros2.giskard import Giskard
from giskardpy.middleware.ros2.robot_interface_config import RobotInterfaceConfig
from giskardpy.middleware.ros2.scripts.iai_robots.tiago.exceptions import (
    InvalidBaseSpeedLimits,
    NonFiniteBaseCommand,
)
from giskardpy.middleware.ros2.server_config import ExecutionMode, GiskardServerConfig
from giskardpy.model.world_config import WorldWithDiffDriveRobot
from semantic_digital_twin.adapters.ros.input_synchronization import (
    LatestJointStateSynchronizer,
    SubscribedBasePoseSource,
)
from semantic_digital_twin.robots.input_source import RobotTopic
from semantic_digital_twin.robots.tiago import TiagoTopic
from semantic_digital_twin.robots.tiago_single_arm import TiagoSingleArm
from semantic_digital_twin.world_description.connections import (
    Connection6DoF,
    DifferentialDrive,
)

if TYPE_CHECKING:
    from semantic_digital_twin.robots.robot_parts import AbstractRobot


# %% base limits and feedback


class BaseInterfaceName(StrEnum):
    """
    ROS and command-line names of the single-arm base interface.
    """

    COMMAND_TOPIC = "/cmd_vel"
    MAP_FRAME = "map"
    ODOMETRY_FRAME = "odom"
    NODE = "giskard"
    HARDWARE_OPTION = "--hardware"


@dataclass
class BaseVelocityLimits:
    """
    Configurable speed bounds shared by the solver and command output.
    """

    maximum_forward_velocity: float = 0.05
    """
    Maximum forward or reverse speed, in metres per second.
    """

    maximum_yaw_velocity: float = 0.1
    """
    Maximum yaw speed in either direction, in radians per second.
    """

    def __post_init__(self) -> None:
        """
        Reject speed bounds that cannot constrain the base command.
        """
        if any(
            not math.isfinite(speed) or speed <= 0
            for speed in [self.maximum_forward_velocity, self.maximum_yaw_velocity]
        ):
            raise InvalidBaseSpeedLimits(
                self.maximum_forward_velocity, self.maximum_yaw_velocity
            )

    def apply_to(self, drive: DifferentialDrive) -> None:
        """
        Set matching solver speed bounds on the two drive axes.
        """
        drive.x_velocity.limits.lower.velocity = -self.maximum_forward_velocity
        drive.x_velocity.limits.upper.velocity = self.maximum_forward_velocity
        drive.yaw.limits.lower.velocity = -self.maximum_yaw_velocity
        drive.yaw.limits.upper.velocity = self.maximum_yaw_velocity


@dataclass
class BaseFeedback:
    """
    The real positions required before allowing a nonzero base command.
    """

    odometry: SubscribedBasePoseSource
    """
    The laser odometry feedback used by the controller.
    """

    joints: LatestJointStateSynchronizer
    """
    The joint positions required to locate the robot's collision geometry.
    """

    maximum_age: float = 2.0
    """
    The tolerated timestamp offset from the local ROS clock, in seconds.
    """

    def is_current(self) -> bool:
        """
        Require joint and odometry samples within the tolerated timestamp offset.
        """
        current_time = float(self.odometry.node.get_clock().now().nanoseconds) / 1e9
        for source in [self.odometry, self.joints]:
            message = source.latest_message
            if message is None:
                return False
            sample_time = float(Time.from_msg(message.header.stamp).nanoseconds) / 1e9
            if abs(current_time - sample_time) > self.maximum_age:
                return False
        return True


# %% bounded command publication


@dataclass
class LimitedDriveVelocityReader(StateVelocityReader):
    """
    Read differential-drive velocities with the configured output saturation.
    """

    velocity_limits: BaseVelocityLimits = field(default_factory=BaseVelocityLimits)
    """
    The output speed bounds for forward translation and yaw.
    """

    def velocities(self) -> list[float]:
        """
        Saturate the velocities returned by :meth:`StateVelocityReader.velocities`.
        """
        velocities = super().velocities()
        if any(not math.isfinite(velocity) for velocity in velocities):
            return velocities
        maxima = [
            self.velocity_limits.maximum_forward_velocity,
            self.velocity_limits.maximum_yaw_velocity,
        ]
        return [
            float(np.clip(velocity, -maximum, maximum))
            for velocity, maximum in zip(velocities, maxima)
        ]


@dataclass
class TiagoBaseCommandPublisher(DriveVelocityCommandPublisher):
    """
    Publish bounded twists only while the real robot feedback is current.
    """

    connection: DifferentialDrive
    """
    The differential drive whose forward and yaw velocities are published.
    """

    feedback: BaseFeedback = field(kw_only=True)
    """
    The feedback condition required for nonzero commands.
    """

    velocity_limits: BaseVelocityLimits = field(default_factory=BaseVelocityLimits)
    """
    The bounds applied both to the solver and outgoing commands.
    """

    def __post_init__(self) -> None:
        """
        Apply shared speed bounds to
        :meth:`DriveVelocityCommandPublisher.__post_init__`.
        """
        super().__post_init__()
        self.velocity_limits.apply_to(self.connection)
        self.velocity_reader = LimitedDriveVelocityReader(
            world=self.world,
            degrees_of_freedom=[self.connection.x_velocity, self.connection.yaw],
            velocity_limits=self.velocity_limits,
        )

    def publish(self) -> None:
        """
        Stop on missing feedback or invalid output before
        :meth:`DriveVelocityCommandPublisher.publish`.
        """
        if not self.feedback.is_current():
            self.stop()
            return
        velocities = self.velocity_reader.velocities()
        if any(not math.isfinite(velocity) for velocity in velocities):
            self.stop()
            raise NonFiniteBaseCommand(velocities)
        super().publish()


# %% framework configuration


@dataclass
class WorldWithSingleArmTiagoConfig(WorldWithDiffDriveRobot):
    """
    Create the single-arm robot and its bounded differential drive.
    """

    urdf_view: type[AbstractRobot] = field(
        default=TiagoSingleArm, kw_only=True, init=False
    )
    """
    The semantic description matching the captured real robot.
    """

    velocity_limits: BaseVelocityLimits = field(default_factory=BaseVelocityLimits)
    """
    The base speed bounds of this world.
    """

    def setup_world(self) -> None:
        """
        Bind the merged robot and limit the drive from
        :meth:`WorldWithDiffDriveRobot.setup_world`.
        """
        super().setup_world()
        self.robot = self.world.get_semantic_annotations_by_type(TiagoSingleArm)[0]
        drive = self.world.get_connections_by_type(DifferentialDrive)[0]
        self.velocity_limits.apply_to(drive)


@dataclass
class SingleArmTiagoBaseInterface(RobotInterfaceConfig):
    """
    Read the whole robot state and command only its mobile base.
    """

    velocity_limits: BaseVelocityLimits = field(default_factory=BaseVelocityLimits)
    """
    The limits shared with the real base command sender.
    """

    def setup(self) -> None:
        drive = self.world.get_connections_by_type(DifferentialDrive)[0]
        self.velocity_limits.apply_to(drive)
        if self.server_config.is_standalone:
            self.register_controlled_joints([drive.name])
            return
        self.sync_6dof_joint_with_tf_frame(
            joint=self.world.get_connections_by_type(Connection6DoF)[0],
            tf_parent_frame=BaseInterfaceName.MAP_FRAME,
            tf_child_frame=BaseInterfaceName.ODOMETRY_FRAME,
        )
        feedback = BaseFeedback(
            odometry=SubscribedBasePoseSource(
                world=self.world,
                node=rospy.get_node(),
                topic_name=TiagoTopic.ODOMETRY,
                connection=drive,
            ),
            joints=LatestJointStateSynchronizer(
                world=self.world,
                node=rospy.get_node(),
                topic_name=RobotTopic.JOINT_STATES,
            ),
        )
        for source in [feedback.odometry, feedback.joints]:
            self.motion_server.inputs.add_input(source)
            self.control_loop.inputs.add_input(source)
        self.robot.mobile_base.source = feedback.odometry
        self.robot.mobile_base.lidar.use_real_source(rospy.get_node())
        self.control_loop.command_publishers.append(
            TiagoBaseCommandPublisher(
                world=self.world,
                command_topic=BaseInterfaceName.COMMAND_TOPIC,
                connection=drive,
                feedback=feedback,
                velocity_limits=self.velocity_limits,
            )
        )


# %% explicit process entry point


def main() -> None:
    """
    Start standalone by default; hardware mode requires an explicit option.
    """
    parser = argparse.ArgumentParser()
    parser.add_argument(BaseInterfaceName.HARDWARE_OPTION, action="store_true")
    options = parser.parse_args()
    mode = ExecutionMode.CLOSED_LOOP if options.hardware else ExecutionMode.STANDALONE
    limits = BaseVelocityLimits()
    rospy.init_node(BaseInterfaceName.NODE)
    giskard = Giskard(
        world_config=WorldWithSingleArmTiagoConfig(
            urdf=Path(TiagoSingleArm.get_ros_file_path()).read_text(),
            velocity_limits=limits,
        ),
        robot_interface_config=SingleArmTiagoBaseInterface(velocity_limits=limits),
        server_config=GiskardServerConfig(execution_mode=mode),
    )
    giskard.live()


if __name__ == "__main__":
    main()
