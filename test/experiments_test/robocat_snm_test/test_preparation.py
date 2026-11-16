"""
Verify controller reconfiguration order and aborts without hardware services.
"""

# %% imports
from unittest.mock import Mock, call

import pytest

from experiments.robocat_snm import native_preparation
from experiments.robocat_snm.native_preparation import (
    ControllerPreparation,
    ControllerState,
)


# %% controller preparation
@pytest.fixture
def preparation() -> ControllerPreparation:
    """
    Provide only an explicitly stopped, authorized preparation context.
    """
    subject = ControllerPreparation(Mock(), Mock())
    subject.safety.wait.return_value = True
    subject.calibration.verify = Mock(return_value=True)
    return subject


def test_unsafe_preparation_does_not_reconfigure(
    preparation: ControllerPreparation, monkeypatch: pytest.MonkeyPatch
) -> None:
    """
    Missing stop/interlock evidence blocks every controller transition.
    """
    preparation.safety.wait.return_value = False
    command = Mock()
    monkeypatch.setattr(native_preparation.subprocess, "run", command)
    assert preparation.apply() is False
    command.assert_not_called()


def test_failed_parameter_write_never_reactivates_controller(
    preparation: ControllerPreparation,
) -> None:
    """
    A rejected atomic update leaves activation for explicit operator recovery.
    """
    preparation.transition = Mock(return_value=True)
    preparation.write_values = Mock(return_value=False)
    assert preparation.apply() is False
    assert preparation.transition.call_args_list == [
        call(ControllerState.INACTIVE),
        call(ControllerState.UNCONFIGURED),
    ]


def test_success_requires_cleanup_configure_activate_and_readback(
    preparation: ControllerPreparation,
) -> None:
    """
    Reading new values alone is insufficient without a configure transition.
    """
    preparation.transition = Mock(return_value=True)
    preparation.write_values = Mock(return_value=True)
    preparation.controller_state = Mock(return_value=ControllerState.ACTIVE)
    assert preparation.apply() is True
    assert preparation.transition.call_args_list == [
        call(ControllerState.INACTIVE),
        call(ControllerState.UNCONFIGURED),
        call(ControllerState.INACTIVE),
        call(ControllerState.ACTIVE),
    ]
    preparation.calibration.verify.assert_called_once_with()


def test_cli_success_is_checked_against_actual_controller_state(
    preparation: ControllerPreparation, monkeypatch: pytest.MonkeyPatch
) -> None:
    """
    A successful subprocess return cannot conceal a failed state transition.
    """
    preparation.controller_state = Mock(return_value=ControllerState.ACTIVE)
    monkeypatch.setattr(
        native_preparation.subprocess,
        "run",
        Mock(return_value=Mock(returncode=0, stdout="", stderr="")),
    )
    assert preparation.transition(ControllerState.UNCONFIGURED) is False
