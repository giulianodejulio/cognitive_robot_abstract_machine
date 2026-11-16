"""
Recovery retains the operator's navigation tolerances with verified readback.
"""

# %% imports
from unittest.mock import Mock

import pytest
from rcl_interfaces.msg import ParameterType, ParameterValue

from experiments.robocat_snm.motor_recovery import RecoverySettings


# %% parameter preservation
@pytest.mark.parametrize("invalid", [float("nan"), float("inf"), 0.0, -1.0])
def test_recovery_does_not_restore_invalid_tolerances(invalid) -> None:
    """
    Invalid saved input must not generate a native parameter write.
    """
    node = Mock()
    assert RecoverySettings(node).restore(invalid, 0.1) is False
    node.create_client.assert_not_called()


def test_matching_tolerances_are_preserved_without_parameter_writes(
    monkeypatch,
) -> None:
    """
    Recovery leaves a matching navigation configuration untouched.
    """
    node = Mock()
    future = node.create_client.return_value.call_async.return_value
    future.done.return_value = True
    future.exception.return_value = None
    values = [0.03, 0.0872665]
    future.result.return_value.values = [
        ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value)
        for value in values
    ]
    monkeypatch.setattr(
        "experiments.robocat_snm.motor_recovery.rclpy.spin_until_future_complete",
        Mock(),
    )
    settings = RecoverySettings(node)
    assert settings.restore(*values) is True
    assert node.create_client.call_count == 1
    request = node.create_client.return_value.call_async.call_args.args[0]
    assert request.names == list(settings.names)
