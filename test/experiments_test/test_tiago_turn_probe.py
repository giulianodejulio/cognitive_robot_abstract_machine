# %% imports
from __future__ import annotations

import importlib
import math

import pytest
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


# %% bounded measurement without ROS nodes
@pytest.fixture
def probe():
    """
    Load the measurement classes without creating subscriptions.
    """
    return importlib.import_module("experiments.tiago_turn_probe")


def test_odometry_translation_is_relative_to_its_initial_pose(probe) -> None:
    """
    Different global origins must not look like disagreement in displacement.
    """
    trace = probe.OdometryTrace()
    first = Odometry()
    first.pose.pose.position.x = 10.0
    first.pose.pose.orientation.z = math.sin(math.pi / 4)
    first.pose.pose.orientation.w = math.cos(math.pi / 4)
    trace.receive(first)
    second = Odometry()
    second.pose.pose.position.x = 10.0
    second.pose.pose.position.y = 0.04
    second.pose.pose.orientation.z = math.sin(math.pi / 3)
    second.pose.pose.orientation.w = math.cos(math.pi / 3)
    trace.receive(second)
    result = trace.result()
    assert result.forward == pytest.approx(0.04)
    assert result.lateral == pytest.approx(0)
    assert result.displacement == pytest.approx(0.04)
    assert result.yaw_degrees == pytest.approx(30)


def test_reset_starts_a_new_measurement_window(probe) -> None:
    """
    Idle baseline drift must not be included in the rotation displacement.
    """
    trace = probe.OdometryTrace()
    message = Odometry()
    message.pose.pose.orientation.w = 1.0
    trace.receive(message)
    next_message = Odometry()
    next_message.pose.pose.position.x = 0.02
    next_message.pose.pose.orientation.w = 1.0
    trace.receive(next_message)
    trace.reset()
    final = Odometry()
    final.pose.pose.position.x = 0.05
    final.pose.pose.orientation.w = 1.0
    trace.receive(final)
    result = trace.result()
    assert result.samples == 1
    assert result.displacement == pytest.approx(0.03)


def test_command_peak_detects_both_signs(probe) -> None:
    """
    A zero mean must not conceal alternating nonzero linear commands.
    """
    trace = probe.CommandTrace()
    for velocity in (-0.001, 0.002):
        message = Twist()
        message.linear.x = velocity
        trace.receive(message)
    assert trace.result().samples == 2
    assert trace.result().maximum_absolute_linear_x == 0.002
    trace.reset()
    assert trace.result().samples == 0


def test_execution_requires_explicit_mode(probe) -> None:
    """
    Importing or requesting no mode cannot dispatch a real turn.
    """
    with pytest.raises(SystemExit):
        probe.ProbeOptions.parse([])
    assert probe.ProbeOptions.parse(["--check"]).execute is False
    assert probe.ProbeOptions.parse(["--execute", "--angle", "30"]).angle == 30
