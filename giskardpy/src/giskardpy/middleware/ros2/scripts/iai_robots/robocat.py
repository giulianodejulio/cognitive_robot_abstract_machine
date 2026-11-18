# %% imports
from __future__ import annotations

from dataclasses import dataclass, field, replace
from datetime import timedelta
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from rclpy.qos import QoSProfile, LivelinessPolicy, ReliabilityPolicy, DurabilityPolicy
from krrood.ormatic.utils import classproperty

from experiments.robocat_safety import RobocatFeedback, RobocatTopic
from experiments.robocat_completion import MeasuredStopContext
from experiments.robocat_snm.lidar_entry import WallControlContext
from giskardpy.middleware.ros2 import rospy
from giskardpy.middleware.ros2.robot_interface_config import RobotInterfaceConfig
from giskardpy.middleware.ros2.scripts.iai_robots.tiago.single_arm import (
    BaseFeedback,
    BaseVelocityLimits,
    TiagoBaseCommandPublisher,
)
from giskardpy.model.world_config import WorldWithDiffDriveRobot
from giskardpy.qp.jerk_limited_braking import JerkLimitedBraking
from giskardpy.qp.qp_controller_config import QPControllerConfig
from semantic_digital_twin.adapters.ros.input_synchronization import (
    LatestJointStateSynchronizer,
    SubscribedBasePoseSource,
)
from semantic_digital_twin.adapters.ros.lidar import SubscribedLidarSource
from semantic_digital_twin.adapters.ros.ros2_to_semdt_converters import (
    LaserScanToSemDTConverter,
)
from semantic_digital_twin.adapters.sensors.lidar import Lidar
from semantic_digital_twin.datastructures.lidar_reading import LidarReading
from semantic_digital_twin.robots.robocat import Robocat
from semantic_digital_twin.robots.robot_parts import AbstractRobot
from semantic_digital_twin.world_description.connections import DifferentialDrive
from semantic_digital_twin.world_description.degree_of_freedom import DegreeOfFreedom


# %% PAL-compatible solver dynamics
@dataclass(frozen=True)
class RobocatAxisDynamics:
    """
    Symmetric acceleration and jerk bounds for one real base axis.
    """

    acceleration: float
    """
    Maximum acceleration and deceleration magnitude in SI units.
    """

    jerk: float
    """
    Maximum magnitude of acceleration change per second in SI units.
    """

    def apply_to(self, degree: DegreeOfFreedom) -> None:
        """
        Constrain both directions of the solver degree of freedom.
        """
        degree.limits.lower.acceleration = -self.acceleration
        degree.limits.upper.acceleration = self.acceleration
        degree.limits.lower.jerk = -self.jerk
        degree.limits.upper.jerk = self.jerk

    def braking(self, velocity: float, time_step: timedelta) -> JerkLimitedBraking:
        """
        Use the native solver's acceleration-safe braking bound.
        """
        return JerkLimitedBraking(
            velocity_limit=velocity, jerk_limit=self.jerk, time_step=time_step
        ).limited_to_acceleration(self.acceleration)


@dataclass
class RobocatMotionLimits(BaseVelocityLimits):
    """
    Speed and dynamics bounds matching the observed PAL base controller.
    """

    linear: RobocatAxisDynamics = field(
        default_factory=lambda: RobocatAxisDynamics(acceleration=0.6, jerk=2.0)
    )
    """
    PAL linear.x acceleration/deceleration and jerk bounds.
    """

    angular: RobocatAxisDynamics = field(
        default_factory=lambda: RobocatAxisDynamics(acceleration=1.0, jerk=5.0)
    )
    """
    PAL angular.z acceleration/deceleration and jerk bounds.
    """

    def apply_to(self, drive: DifferentialDrive) -> None:
        """
        Apply velocity bounds and PAL dynamics to the two controlled axes.
        """
        super().apply_to(drive)
        self.linear.apply_to(drive.x_velocity)
        self.angular.apply_to(drive.yaw)

    def controller_configuration(
        self, target_frequency: float = 20
    ) -> QPControllerConfig:
        """
        Size the native QP horizon to contain braking of both controlled axes.
        """
        configuration = QPControllerConfig(target_frequency=target_frequency)
        braking_steps = max(
            self.linear.braking(
                self.maximum_forward_velocity, configuration.control_time_step
            ).number_of_steps,
            self.angular.braking(
                self.maximum_yaw_velocity, configuration.control_time_step
            ).number_of_steps,
        )
        return replace(
            configuration,
            prediction_horizon=max(
                configuration.prediction_horizon,
                braking_steps + configuration.number_of_resting_steps,
            ),
        )


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

    applied_message: Odometry | None = field(default=None, init=False, repr=False)
    """
    Last odometry sample successfully written into the controller world.
    """

    def apply_message(self, message: Odometry) -> None:
        """
        Retain the sample underlying the measured model pose for diagnostics.
        """
        super().apply_message(message)
        self.applied_message = message

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

    def get_lidar_reading(
        self, lidar: Lidar, scan: LaserScan | None = None
    ) -> LidarReading:
        """
        Convert a captured scan or the latest available sensor message.
        """
        return LaserScanToSemDTConverter.convert(
            self.received_scan if scan is None else scan, lidar.root._world
        )


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
        node = rospy.get_node()
        command_topic = node.resolve_topic_name("/cmd_vel")
        node.declare_parameter(
            "base_feedback_topic", node.resolve_topic_name(RobocatTopic.ODOMETRY)
        )
        node.declare_parameter("base_command_topic", command_topic)
        node.declare_parameter(
            "wall_stop_supported", not self.server_config.is_standalone
        )
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
            interlocks=RobocatFeedback(
                node,
                active_priorities=(
                    ("0", "20") if command_topic == "/snm_vel" else ("0", "10")
                ),
            ),
        )
        self.giskard.executor.context.add_extension(
            MeasuredStopContext(feedback.interlocks)
        )
        for source in (feedback.odometry, feedback.joints):
            self.motion_server.inputs.add_input(source)
            self.control_loop.inputs.add_input(source)
        self.robot.mobile_base.source = feedback.odometry
        self.robot.mobile_base.lidar.source = RobocatLidarSource(
            node=rospy.get_node(), topic_name="/scan"
        )
        self.giskard.executor.context.add_extension(
            WallControlContext(
                self.robot.mobile_base.lidar.source,
                feedback.interlocks,
                self.world.compute_forward_kinematics(
                    self.robot.root, self.robot.mobile_base.lidar.root
                ).to_np(),
            )
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
