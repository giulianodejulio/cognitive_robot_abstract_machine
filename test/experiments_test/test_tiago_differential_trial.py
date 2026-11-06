# %% imports
from __future__ import annotations

import math
import os
import select
import subprocess
from concurrent.futures import Future
from contextlib import ExitStack, nullcontext
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from nav_msgs.msg import Odometry

from giskardpy.motion_statechart.goals.cartesian_goals import DifferentialDriveBaseGoal
from giskardpy.motion_statechart.goals.templates import Sequence
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.spatial_types import Pose
from semantic_digital_twin.world_description.world_entity import Body
from test.experiments_test.test_tiago_giskard_safety import client_script
from test.experiments_test.test_tiago_goal_command import command
from test.experiments_test.test_tiago_goal_foreground import request_pipe


# %% expansion of the native goal without a solver
@pytest.fixture
def expansion_context(monkeypatch):
    """
    Provide numeric poses while inspecting the native three-phase goal.
    """
    root = Body(name=PrefixedName("odom"))
    tip = Body(name=PrefixedName("base_footprint"))
    world = Mock(root=root)
    world.transform.return_value = Pose.from_xyz_rpy(x=0.4, reference_frame=root)
    world.compose_forward_kinematics_expression.return_value = Pose(
        reference_frame=root
    )
    monkeypatch.setattr(Sequence, "expand", Mock())
    return SimpleNamespace(world=world, drive=SimpleNamespace(child=tip))


def test_native_goal_applies_distinct_thresholds_to_all_phases(
    expansion_context: SimpleNamespace,
) -> None:
    """
    Heading alignment is capped separately from the two final pose tolerances.
    """
    position, orientation = 0.02, math.radians(5)
    goal = DifferentialDriveBaseGoal(
        goal_pose=expansion_context.world.transform.return_value,
        diff_drive_connection=expansion_context.drive,
        threshold=position,
        orientation_threshold=orientation,
    )
    goal.expand(expansion_context)
    assert goal.nodes[0].nodes[0].threshold == min(orientation, goal.heading_threshold)
    assert [
        (node.translation_threshold, node.orientation_threshold)
        for node in goal.nodes[1:]
    ] == [
        (position, orientation),
        (position, orientation),
    ]


def test_native_goal_uses_single_threshold_as_final_pose_fallback(
    expansion_context: SimpleNamespace,
) -> None:
    """
    Without an angular override, both pose phases retain the shared tolerance.
    """
    goal = DifferentialDriveBaseGoal(
        goal_pose=expansion_context.world.transform.return_value,
        diff_drive_connection=expansion_context.drive,
        threshold=0.3,
    )
    goal.expand(expansion_context)
    assert goal.nodes[0].nodes[0].threshold == min(
        goal.threshold, goal.heading_threshold
    )
    assert [
        (node.translation_threshold, node.orientation_threshold)
        for node in goal.nodes[1:]
    ] == [
        (goal.threshold, goal.threshold),
        (goal.threshold, goal.threshold),
    ]


# %% request selection through the existing foreground CLI
def test_differential_request_reaches_parser(
    command: Path, request_pipe, client_script: dict
) -> None:
    """
    The selected goal type travels with the distance through the warm client.
    """
    path, descriptor = request_pipe
    sender = subprocess.Popen(
        ["bash", str(command), "0.4", "--differential-drive", "--fifo", str(path)],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    with ExitStack() as cleanup:
        cleanup.callback(sender.kill)
        assert select.select([descriptor], [], [], 3)[0] == [descriptor]
        request = client_script["GoalRequest"].parse(
            os.read(descriptor, 4096).decode().strip()
        )
        assert request.distance == 0.4
        assert request.differential_drive is True
        request.publish("succeeded")
        sender.communicate(timeout=5)
        assert sender.returncode == 0


def test_differential_motion_uses_shared_tolerances(client_script: dict) -> None:
    """
    The native chart must use the same tolerance configuration as Cartesian goals.
    """
    motion = SimpleNamespace(target=Mock())
    chart = client_script["DifferentialBaseMotion"]._motion_chart.fget(motion)
    tolerances = client_script["StraightBaseMotion"].tolerances
    assert type(chart) is DifferentialDriveBaseGoal
    assert chart.goal_pose is motion.target
    assert chart.threshold == tolerances.position
    assert chart.orientation_threshold == math.radians(tolerances.orientation_degrees)


def test_differential_target_is_forward_with_initial_heading(
    client_script: dict, monkeypatch
) -> None:
    """
    A 40 cm request retains the measured initial heading as its final yaw.
    """
    session_type = client_script["GoalSession"]
    namespace = session_type.send.__globals__
    factory = Mock()
    monkeypatch.setitem(namespace, "execute_single", factory)
    monkeypatch.setitem(namespace, "real_robot", lambda **kwargs: nullcontext())
    monkeypatch.setattr(namespace["time"], "sleep", Mock())
    feedback = Mock()
    feedback.odometry = Odometry()
    initial = feedback.odometry.pose.pose
    initial.position.x, initial.position.y = 2.0, -1.0
    yaw = math.radians(60)
    initial.orientation.z, initial.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    completed = Future()
    completed.set_result(None)
    pool = Mock()
    pool.submit.return_value = completed
    session = session_type(context=Mock(), feedback=feedback, pool=pool)
    assert session.send(0.4, differential_drive=True) is True
    motion = factory.call_args.args[0]
    assert type(motion) is client_script["DifferentialBaseMotion"]
    assert float(motion.target.x) == pytest.approx(
        initial.position.x + 0.4 * math.cos(yaw)
    )
    assert float(motion.target.y) == pytest.approx(
        initial.position.y + 0.4 * math.sin(yaw)
    )
    assert float(motion.target.yaw) == pytest.approx(yaw)
