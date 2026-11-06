# %% imports

from __future__ import annotations

from dataclasses import dataclass
from contextlib import ExitStack
from pathlib import Path
from unittest.mock import Mock

import pytest
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

from giskardpy.middleware.ros2 import rospy
from giskardpy.middleware.ros2.giskard import Giskard
from giskardpy.data_types.exceptions import NoControlledJointsError
from giskardpy.middleware.ros2.server_config import ExecutionMode, GiskardServerConfig
from giskardpy.middleware.ros2.scripts.iai_robots.tiago.exceptions import (
    InvalidBaseSpeedLimits,
    NonFiniteBaseCommand,
)
from giskardpy.middleware.ros2.scripts.iai_robots.tiago.single_arm import (
    BaseInterfaceName,
    BaseFeedback,
    BaseVelocityLimits,
    TiagoBaseCommandPublisher,
    SingleArmTiagoBaseInterface,
    WorldWithSingleArmTiagoConfig,
)
from semantic_digital_twin.adapters.ros.input_synchronization import (
    LatestJointStateSynchronizer,
    SubscribedBasePoseSource,
)
from semantic_digital_twin.world_description.connections import DifferentialDrive
from semantic_digital_twin.input_synchronization import WorldStateInputs
from semantic_digital_twin.robots.tiago_single_arm import TiagoSingleArm

from .test_command_publishing import RecordingPublisher, build_drive_world
from ...semantic_digital_twin_test.test_robots.test_tiago_single_arm import (
    single_arm_robot,
)

# %% recorded commands


@dataclass
class FeedbackControlledCommands:
    """
    A base command sender with mocked transport and explicitly supplied feedback.
    """

    sender: TiagoBaseCommandPublisher
    """The command sender under test."""

    recorder: RecordingPublisher
    """
    The transport recording messages instead of sending them to a robot.
    """


@pytest.fixture
def commands(
    init_rospy: None, monkeypatch: pytest.MonkeyPatch
) -> FeedbackControlledCommands:
    """
    Create a base publisher whose transport never reaches ROS.
    """
    node = rospy.get_node()
    world = build_drive_world(DifferentialDrive)
    drive = world.get_connection_by_name("brumbrum")
    feedback = BaseFeedback(
        odometry=SubscribedBasePoseSource(
            world=world, node=node, topic_name="test_odom", connection=drive
        ),
        joints=LatestJointStateSynchronizer(
            world=world, node=node, topic_name="test_joint_states"
        ),
    )
    recorder = RecordingPublisher()
    monkeypatch.setattr(node, "create_publisher", Mock(return_value=recorder))
    sender = TiagoBaseCommandPublisher(
        world=world, command_topic="test_cmd_vel", connection=drive, feedback=feedback
    )
    stamp = node.get_clock().now().to_msg()
    odometry = Odometry()
    odometry.header.stamp = stamp
    joints = JointState()
    joints.header.stamp = stamp
    feedback.odometry.buffer_message(odometry)
    feedback.joints.buffer_message(joints)
    return FeedbackControlledCommands(sender=sender, recorder=recorder)


# %% speed bounds and stop


@pytest.mark.parametrize("direction", [-1.0, 1.0])
def test_base_commands_are_bounded(
    commands: FeedbackControlledCommands, direction: float
) -> None:
    """
    Forward and yaw commands are clipped independently in both directions.
    """
    sender = commands.sender
    sender.world.state[sender.connection.x_velocity.id].velocity = direction
    sender.world.state[sender.connection.yaw.id].velocity = direction

    sender.publish()

    message = commands.recorder.published_message
    assert (
        message.linear.x == direction * sender.velocity_limits.maximum_forward_velocity
    )
    assert message.linear.y == 0.0
    assert message.angular.z == direction * sender.velocity_limits.maximum_yaw_velocity


def test_base_stop_overrides_a_previous_command(
    commands: FeedbackControlledCommands,
) -> None:
    """
    Stopping replaces the previous nonzero command with a zero twist.
    """
    sender = commands.sender
    sender.world.state[sender.connection.x_velocity.id].velocity = 0.01
    sender.publish()

    sender.stop()

    message = commands.recorder.published_message
    assert message.linear.x == 0.0
    assert message.linear.y == 0.0
    assert message.angular.z == 0.0


def test_missing_feedback_sends_zero(commands: FeedbackControlledCommands) -> None:
    """
    No odometry sample means no nonzero base command is emitted.
    """
    sender = commands.sender
    sender.world.state[sender.connection.x_velocity.id].velocity = 0.1
    sender.feedback.odometry.latest_message = None

    sender.publish()

    assert commands.recorder.published_message.linear.x == 0.0
    assert commands.recorder.published_message.angular.z == 0.0


def test_stale_feedback_sends_zero(commands: FeedbackControlledCommands) -> None:
    """
    An old odometry timestamp prevents motion despite a nonzero solver output.
    """
    sender = commands.sender
    sender.world.state[sender.connection.x_velocity.id].velocity = 0.1
    sender.feedback.odometry.latest_message.header.stamp.sec = 0

    sender.publish()

    assert commands.recorder.published_message.linear.x == 0.0
    assert commands.recorder.published_message.angular.z == 0.0


def test_velocity_limits_reach_the_solver(commands: FeedbackControlledCommands) -> None:
    """
    The drive's solver bounds agree with the command saturation bounds.
    """
    sender = commands.sender

    assert (
        sender.connection.x_velocity.limits.upper.velocity
        == sender.velocity_limits.maximum_forward_velocity
    )
    assert (
        sender.connection.x_velocity.limits.lower.velocity
        == -sender.velocity_limits.maximum_forward_velocity
    )
    assert (
        sender.connection.yaw.limits.upper.velocity
        == sender.velocity_limits.maximum_yaw_velocity
    )
    assert (
        sender.connection.yaw.limits.lower.velocity
        == -sender.velocity_limits.maximum_yaw_velocity
    )


@pytest.mark.parametrize("velocity", [float("nan"), float("inf")])
def test_non_finite_commands_stop_before_rejection(
    commands: FeedbackControlledCommands, velocity: float
) -> None:
    """
    Non-finite solver output is rejected after sending a zero command.
    """
    commands.sender.world.state[commands.sender.connection.x_velocity.id].velocity = (
        velocity
    )

    with pytest.raises(NonFiniteBaseCommand):
        commands.sender.publish()

    assert commands.recorder.published_message.linear.x == 0.0
    assert commands.recorder.published_message.angular.z == 0.0


@pytest.mark.parametrize("maximum_speed", [0.0, -0.1, float("inf")])
def test_invalid_speed_bounds_are_rejected(maximum_speed: float) -> None:
    """
    Speed bounds must be finite positive magnitudes.
    """
    with pytest.raises(InvalidBaseSpeedLimits):
        BaseVelocityLimits(maximum_forward_velocity=maximum_speed)


# %% framework wiring


def test_closed_loop_interface_commands_only_the_base(
    init_rospy: None, monkeypatch: pytest.MonkeyPatch
) -> None:
    """
    Closed-loop configuration wires feedback and a single mocked base publisher.
    """
    packages = pytest.importorskip("ament_index_python.packages")
    required = {
        "tiago_description",
        "pmb2_description",
        "pal_gripper_description",
        "pal_urdf_utils",
    }
    if not required.issubset(packages.get_packages_with_prefixes()):
        pytest.skip("PAL description packages are not installed")
    world_config = WorldWithSingleArmTiagoConfig(
        urdf=Path(TiagoSingleArm.get_ros_file_path()).read_text()
    )
    with world_config.world.modify_world():
        world_config.setup_world()
    server = Mock(spec=Giskard)
    server.executor = Mock()
    server.executor.context.world = world_config.world
    server.robot = world_config.robot
    server.server_config = GiskardServerConfig(execution_mode=ExecutionMode.CLOSED_LOOP)
    server.motion_server = Mock()
    server.motion_server.inputs = WorldStateInputs(world=world_config.world)
    server.motion_server.control_loop.inputs = WorldStateInputs(
        world=world_config.world, reapplies_inputs=True
    )
    server.motion_server.control_loop.command_publishers = []
    recorder = RecordingPublisher()
    monkeypatch.setattr(
        rospy.get_node(), "create_publisher", Mock(return_value=recorder)
    )
    interface = SingleArmTiagoBaseInterface()
    interface.attach(server)

    interface.setup()

    senders = server.motion_server.control_loop.command_publishers
    assert [type(sender) for sender in senders] == [TiagoBaseCommandPublisher]
    assert senders[0].command_topic == BaseInterfaceName.COMMAND_TOPIC
    assert world_config.robot.controlled_connections == []
    assert world_config.robot.drive.has_hardware_interface is True
    assert recorder.published_messages == []


def test_giskard_accepts_a_controlled_drive_without_controlled_arm_joints(
    commands: FeedbackControlledCommands,
) -> None:
    """
    A controlled drive is sufficient for a base-only Giskard controller.
    """
    server = Mock(spec=Giskard)
    server.world_config = Mock()
    server.world_config.world = commands.sender.world
    server.robot = Mock()
    server.robot.controlled_connections = []

    Giskard._controlled_joints_sanity_check(server)

    assert commands.sender.connection.is_controlled is True
    assert commands.recorder.published_messages == []


def test_giskard_rejects_a_world_without_any_controlled_connection(
    commands: FeedbackControlledCommands,
) -> None:
    """
    Base-only support does not accept a drive whose control flag is disabled.
    """
    commands.sender.connection.has_hardware_interface = False
    server = Mock(spec=Giskard)
    server.world_config = Mock()
    server.world_config.world = commands.sender.world
    server.robot = Mock()
    server.robot.controlled_connections = []

    with pytest.raises(NoControlledJointsError):
        Giskard._controlled_joints_sanity_check(server)


def test_standalone_server_initializes_without_base_command_publishers(
    init_rospy: None, single_arm_robot: TiagoSingleArm
) -> None:
    """
    The real Giskard setup succeeds in its default non-actuating mode.
    """
    server = Giskard(
        world_config=WorldWithSingleArmTiagoConfig(
            urdf=Path(single_arm_robot.get_ros_file_path()).read_text()
        ),
        robot_interface_config=SingleArmTiagoBaseInterface(),
        server_config=GiskardServerConfig(execution_mode=ExecutionMode.STANDALONE),
    )
    with ExitStack() as cleanup:
        server.setup()
        cleanup.callback(server.close_world_model_ros_interface)

        assert type(server.robot) is TiagoSingleArm
        assert server.motion_server.control_loop.command_publishers == []
