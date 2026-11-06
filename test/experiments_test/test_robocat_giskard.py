# %% imports
from __future__ import annotations

import math
import time
import xml.etree.ElementTree as xml
from pathlib import Path
from unittest.mock import Mock

import pytest
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from scripts.robocat_giskard_goal import RobocatStraightMotion
from experiments.robocat_safety import RobocatFeedback, RobocatTopic
from semantic_digital_twin.adapters.urdf import URDFParser
from semantic_digital_twin.robots.robocat import Robocat
from semantic_digital_twin.world_description.connections import DifferentialDrive
from test.experiments_test.test_tiago_goal_cancel_session import (
    test_cancel_then_new_request_keeps_client_alive as check_cancellation_sequence,
)
from scripts.tiago_giskard_goal import GoalInbox, MotionTolerances


# %% real description without hardware
@pytest.fixture(scope="module")
def robocat() -> Robocat:
    """
    Load the acquired description without connecting to ROS.
    """
    return Robocat.from_world(URDFParser.from_file(Robocat.get_ros_file_path()).parse())


def test_robocat_description_resolves_all_meshes() -> None:
    """
    Resolve every acquired mesh without requiring PAL packages in ament.
    """
    description = xml.parse(Robocat.get_ros_file_path()).getroot()
    assert all(
        Path(mesh.attrib["filename"].removeprefix("file://")).is_file()
        for mesh in description.findall(".//mesh")
    )


def test_robocat_model_keeps_hardware_disabled(robocat: Robocat) -> None:
    """
    Loading the robot does not enable wheels or head command interfaces.
    """
    assert Robocat.get_drive_connection_type() is DifferentialDrive
    assert robocat.degrees_of_freedom_with_hardware_interface == []
    assert robocat.root.name.name == "base_footprint"
    assert len(robocat.bodies) == 17


def test_straight_mapping_is_bound_to_robocat() -> None:
    """
    Select the established Cartesian task implementation for Robocat.
    """
    assert RobocatStraightMotion.original_class() is Robocat
    assert RobocatStraightMotion.tolerances == MotionTolerances()


# %% safety interlocks and stream freshness
@pytest.fixture
def feedback() -> RobocatFeedback:
    """
    Provide stationary, fresh feedback with every interlock released.
    """
    node = Mock()
    node.get_clock.return_value.now.return_value.nanoseconds = 10_000_000_000
    state = RobocatFeedback(node)
    state.receive_emergency(Bool(data=False))
    state.receive_joy(Bool(data=False))
    odometry = Odometry()
    odometry.header.stamp.sec = 10
    odometry.header.frame_id = "odom"
    odometry.child_frame_id = "base_footprint"
    odometry.pose.pose.orientation.w = 1.0
    state.receive_odometry(odometry)
    state.receive_wheels(odometry)
    joints = JointState(name=["head_joint"], position=[0.0])
    joints.header.stamp.sec = 10
    state.receive_joints(joints)
    state.receive_diagnostics(
        DiagnosticArray(
            status=[
                DiagnosticStatus(
                    name="twist_mux: Twist mux status",
                    values=[KeyValue(key="current priority", value="0")],
                )
            ]
        )
    )
    return state


def test_current_robocat_feedback_allows_preflight(feedback: RobocatFeedback) -> None:
    """
    Permit a stationary robot with fresh feedback and released interlocks.
    """
    assert feedback.ready() is True


@pytest.mark.parametrize("topic", [RobocatTopic.EMERGENCY, RobocatTopic.DIAGNOSTICS])
def test_lost_safety_stream_blocks_running_goal(
    feedback: RobocatFeedback, topic: RobocatTopic
) -> None:
    """
    Stop trusting cached safety values when their stream disappears.
    """
    feedback.received_at[topic] = time.monotonic() - 3
    assert feedback.ready(require_free_mux=False) is False


def test_emergency_blocks_running_goal(feedback: RobocatFeedback) -> None:
    """
    An asserted hardware emergency invalidates ongoing execution.
    """
    feedback.receive_emergency(Bool(data=True))
    assert feedback.ready(require_free_mux=False) is False


def test_navigation_priority_can_continue_its_own_goal(
    feedback: RobocatFeedback,
) -> None:
    """
    Allow navigation ownership only while a goal is already executing.
    """
    feedback.mux["current priority"] = "10"
    assert feedback.ready() is False
    assert feedback.ready(require_free_mux=False) is True


def test_competing_mux_source_blocks_running_goal(feedback: RobocatFeedback) -> None:
    """
    Reject takeover by an operator or higher priority command source.
    """
    feedback.mux["current priority"] = "100"
    assert feedback.ready(require_free_mux=False) is False


@pytest.mark.parametrize("value", [0.0, math.nan])
def test_invalid_orientation_blocks_execution(
    feedback: RobocatFeedback, value: float
) -> None:
    """
    Reject zero or nonfinite odometry quaternions.
    """
    feedback.odometry.pose.pose.orientation.w = value
    assert feedback.ready(require_free_mux=False) is False


# %% local CLI cancellation without robot motion
def test_robocat_cli_cancellation_keeps_client_alive(tmp_path: Path) -> None:
    """
    Reuse the established protocol test with the dedicated Robocat command.
    """
    check_cancellation_sequence(
        client_script={"GoalInbox": GoalInbox},
        command=Path(__file__).parents[2] / "scripts/robocat-goal",
        tmp_path=tmp_path,
    )
