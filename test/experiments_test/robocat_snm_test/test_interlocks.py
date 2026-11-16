"""
Check calibrated native parameters and the selected command priority.
"""

# %% imports
from unittest.mock import Mock

import pytest
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rcl_interfaces.msg import ParameterValue
from rclpy.task import Future

from experiments.robocat_snm import native_navigation
from experiments.robocat_snm.feedback import EntryFeedback
from experiments.robocat_snm.native_navigation import WheelCalibration
from test.experiments_test.test_robocat_giskard import feedback


# %% native calibration
@pytest.mark.parametrize("radius, expected", [(2.0, True), (1.0, False)])
def test_native_calibration_must_match_validated_distance_scale(
    monkeypatch: pytest.MonkeyPatch, radius: float, expected: bool
) -> None:
    """
    Controller defaults after a restart cannot silently authorize a wheel-based entry.
    """
    node = Mock()
    future = Future()
    future.set_result(
        Mock(
            values=[
                ParameterValue(double_value=radius),
                ParameterValue(double_value=radius),
                ParameterValue(double_value=1.22),
            ]
        )
    )
    node.create_client.return_value.call_async.return_value = future
    monkeypatch.setattr(native_navigation.rclpy, "spin_until_future_complete", Mock())
    assert WheelCalibration(node).verify() is expected


# %% preserved source arbitration
def test_selected_entry_priority_is_allowed_only_while_active(feedback) -> None:
    """
    Priority twenty is compatible with entry but cannot pass stationary preflight.
    """
    feedback.active_priorities = ("0", "20")
    feedback.mux["current priority"] = "20"
    assert feedback.ready(require_free_mux=False) is True
    assert feedback.ready(require_free_mux=True) is False


def test_unrelated_source_remains_blocked(feedback) -> None:
    """
    An unknown priority cannot be accepted as the entry controller.
    """
    feedback.active_priorities = ("0", "20")
    feedback.mux["current priority"] = "100"
    assert feedback.ready(require_free_mux=False) is False


def test_wheel_driver_fault_is_an_entry_blocker() -> None:
    """
    A wheel hardware diagnostic is retained as an actionable blocker.
    """
    state = EntryFeedback(Mock())
    state.receive_diagnostics(
        DiagnosticArray(
            status=[
                DiagnosticStatus(
                    name="/Hardware/Actuators/wheel_left_actuator",
                    level=DiagnosticStatus.ERROR,
                    message="MotorHighDiscrepancyError",
                )
            ]
        )
    )
    assert state.motor_faults == {
        "/Hardware/Actuators/wheel_left_actuator": "MotorHighDiscrepancyError"
    }
