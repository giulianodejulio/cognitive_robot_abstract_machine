# %% imports
from __future__ import annotations

from dataclasses import dataclass, field

from rclpy.qos import QoSProfile, LivelinessPolicy, ReliabilityPolicy, DurabilityPolicy
from krrood.ormatic.utils import classproperty

from experiments.robocat_safety import RobocatFeedback, RobocatTopic
from giskardpy.middleware.ros2 import rospy
from giskardpy.middleware.ros2.robot_interface_config import RobotInterfaceConfig
from giskardpy.middleware.ros2.scripts.iai_robots.tiago.single_arm import (
    BaseFeedback,
    BaseVelocityLimits,
    TiagoBaseCommandPublisher,
)
from giskardpy.model.world_config import WorldWithDiffDriveRobot
from semantic_digital_twin.adapters.ros.input_synchronization import (
    LatestJointStateSynchronizer,
    SubscribedBasePoseSource,
)
from semantic_digital_twin.adapters.ros.lidar import SubscribedLidarSource
from semantic_digital_twin.robots.robocat import Robocat
from semantic_digital_twin.robots.robot_parts import AbstractRobot
from semantic_digital_twin.world_description.connections import DifferentialDrive


# %% bridge-compatible sensor subscriptions
@dataclass
class RobocatSensorPolicy:
    """
    The concrete best-effort policy of the bridge sensor endpoints.
    """

    @classproperty
    def qos_profile(cls) -> QoSProfile:
        """
        Request volatile samples with automatic liveliness.
        """
        return QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            liveliness=LivelinessPolicy.AUTOMATIC,
        )


@dataclass
class RobocatOdometrySource(SubscribedBasePoseSource):
    """
    Read odometry from the bridge's best-effort sensor stream.
    """

    @classproperty
    def qos_profile(cls) -> QoSProfile:
        return RobocatSensorPolicy.qos_profile


@dataclass
class RobocatJointStateSource(LatestJointStateSynchronizer):
    """
    Read the latest whole-robot joint state with sensor-compatible QoS.
    """

    @classproperty
    def qos_profile(cls) -> QoSProfile:
        return RobocatSensorPolicy.qos_profile


@dataclass
class RobocatLidarSource(SubscribedLidarSource):
    """
    Read the real lidar scan with sensor-compatible QoS.
    """

    @classproperty
    def qos_profile(cls) -> QoSProfile:
        return RobocatSensorPolicy.qos_profile


# %% bounded real feedback
@dataclass
class RobocatCommandFeedback(BaseFeedback):
    """
    Require fresh measured geometry and operator interlocks for every command.
    """

    interlocks: RobocatFeedback = field(kw_only=True)
    """
    Periodic emergency, mux and odometry feedback from the robot.
    """

    def is_current(self) -> bool:
        return super().is_current() and self.interlocks.ready(require_free_mux=False)


# %% world and hardware interface
@dataclass
class WorldWithRobocatConfig(WorldWithDiffDriveRobot):
    """
    Load the acquired Robocat geometry with bounded differential kinematics.
    """

    urdf_view: type[AbstractRobot] = field(default=Robocat, kw_only=True, init=False)
    """
    The semantic robot matching the acquired description.
    """

    velocity_limits: BaseVelocityLimits = field(default_factory=BaseVelocityLimits)
    """
    The speed limits shared with the real command publisher.
    """

    def setup_world(self) -> None:
        super().setup_world()
        self.robot = self.world.get_semantic_annotations_by_type(Robocat)[0]
        self.velocity_limits.apply_to(
            self.world.get_connections_by_type(DifferentialDrive)[0]
        )


@dataclass
class RobocatBaseInterface(RobotInterfaceConfig):
    """
    Read real Robocat state and command its base through the native twist mux.
    """

    velocity_limits: BaseVelocityLimits = field(default_factory=BaseVelocityLimits)
    """
    The solver and output limits of this controller.
    """

    def setup(self) -> None:
        drive = self.world.get_connections_by_type(DifferentialDrive)[0]
        self.velocity_limits.apply_to(drive)
        if self.server_config.is_standalone:
            self.register_controlled_joints([drive.name])
            return
        feedback = RobocatCommandFeedback(
            odometry=RobocatOdometrySource(
                world=self.world,
                node=rospy.get_node(),
                topic_name=RobocatTopic.ODOMETRY,
                connection=drive,
            ),
            joints=RobocatJointStateSource(
                world=self.world, node=rospy.get_node(), topic_name=RobocatTopic.JOINTS
            ),
            interlocks=RobocatFeedback(rospy.get_node()),
        )
        for source in (feedback.odometry, feedback.joints):
            self.motion_server.inputs.add_input(source)
            self.control_loop.inputs.add_input(source)
        self.robot.mobile_base.source = feedback.odometry
        self.robot.mobile_base.lidar.source = RobocatLidarSource(
            node=rospy.get_node(), topic_name="/scan"
        )
        self.control_loop.command_publishers.append(
            TiagoBaseCommandPublisher(
                world=self.world,
                command_topic="/cmd_vel",
                connection=drive,
                feedback=feedback,
                velocity_limits=self.velocity_limits,
            )
        )
