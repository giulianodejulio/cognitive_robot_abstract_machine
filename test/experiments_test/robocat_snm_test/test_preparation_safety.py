"""
Verify measured stop and source ownership before controller changes.
"""

# %% imports
import time
from unittest.mock import Mock

import pytest
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Twist
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

from experiments.robocat_snm.native_preparation import PreparationSafety


# %% native stop evidence
@pytest.fixture
def stopped_robot() -> PreparationSafety:
    """
    Provide current native observations with a completed stationary interval.
    """
    node = Mock()
    node.get_clock.return_value.now.return_value.nanoseconds = 10_000_000_000
    safety = PreparationSafety(node)
    safety.receive_emergency(Bool(data=False))
    safety.receive_charging(Bool(data=False))
    safety.receive_joystick(Bool(data=False))
    safety.receive_status("/navigate_to_pose", GoalStatusArray())
    safety.receive_diagnostics(
        DiagnosticArray(
            status=[
                DiagnosticStatus(
                    name="twist_mux: Twist mux status",
                    values=[KeyValue(key="current priority", value="0")],
                )
            ]
        )
    )
    wheel = Odometry()
    wheel.header.stamp.sec = 10
    safety.receive_wheels(wheel)
    safety.stopped_since = time.monotonic() - safety.stationary_seconds - 0.1
    return safety


def test_current_measured_stop_allows_preparation(
    stopped_robot: PreparationSafety,
) -> None:
    """
    Fresh quiet observations and idle navigation satisfy the preparation gate.
    """
    assert stopped_robot.blockers(require_wheels=True) == []


def test_active_navigation_blocks_controller_change(
    stopped_robot: PreparationSafety,
) -> None:
    """
    A stationary robot with a still-active goal must not be reconfigured.
    """
    stopped_robot.receive_status(
        "/navigate_to_pose",
        GoalStatusArray(status_list=[GoalStatus(status=GoalStatus.STATUS_EXECUTING)]),
    )
    assert len(stopped_robot.blockers(require_wheels=True)) == 1


def test_nonzero_command_blocks_stationary_robot(
    stopped_robot: PreparationSafety,
) -> None:
    """
    A driver not moving despite a current command is not an eligible stop.
    """
    command = Twist()
    command.linear.x = 0.1
    stopped_robot.receive_command("/snm_vel", command)
    assert len(stopped_robot.blockers(require_wheels=True)) == 1


def test_unknown_navigation_never_counts_as_idle(
    stopped_robot: PreparationSafety,
) -> None:
    """
    Missing action-state evidence is explicitly unavailable.
    """
    stopped_robot.active.clear()
    assert len(stopped_robot.blockers(require_wheels=True)) == 1


def test_assisted_teleop_lock_blocks_preparation(
    stopped_robot: PreparationSafety,
) -> None:
    """
    An active mux lock blocks preparation even with zero wheel velocities.
    """
    stopped_robot.receive_diagnostics(
        DiagnosticArray(
            status=[
                DiagnosticStatus(
                    name="twist_mux: Twist mux status",
                    values=[
                        KeyValue(key="current priority", value="0"),
                        KeyValue(key="lock locks.assisted_teleop", value="locked"),
                    ],
                )
            ]
        )
    )
    assert len(stopped_robot.blockers(require_wheels=True)) == 1


def test_operator_idle_confirmation_allows_silent_navigation(
    stopped_robot: PreparationSafety,
) -> None:
    """
    A present silent navigator can use explicit operator idle confirmation.
    """
    stopped_robot.active.clear()
    stopped_robot.navigation_idle_confirmed = True
    stopped_robot.node.count_publishers.return_value = 1
    assert stopped_robot.blockers(require_wheels=True) == []


def test_operator_confirmation_does_not_override_active_goal(
    stopped_robot: PreparationSafety,
) -> None:
    """
    An observed active goal overrides the operator idle declaration.
    """
    stopped_robot.navigation_idle_confirmed = True
    stopped_robot.receive_status(
        "/navigate_to_pose",
        GoalStatusArray(status_list=[GoalStatus(status=GoalStatus.STATUS_EXECUTING)]),
    )
    assert len(stopped_robot.blockers(require_wheels=True)) == 1


def test_operator_confirmation_requires_navigation_publisher(
    stopped_robot: PreparationSafety,
) -> None:
    """
    An absent navigator remains unavailable despite idle confirmation.
    """
    stopped_robot.active.clear()
    stopped_robot.navigation_idle_confirmed = True
    stopped_robot.node.count_publishers.return_value = 0
    assert len(stopped_robot.blockers(require_wheels=True)) == 1
