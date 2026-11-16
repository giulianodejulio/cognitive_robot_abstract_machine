"""
Controller startup may complete after PAL reports its module started.
"""

# %% imports
from unittest.mock import Mock

import pytest
from rcl_interfaces.msg import ParameterType, ParameterValue

from experiments.robocat_snm import native_preparation
from experiments.robocat_snm.native_preparation import (
    ControllerPreparation,
    ControllerState,
)


# %% bounded discovery
def test_preparation_waits_for_controller_and_parameter_services(monkeypatch) -> None:
    """
    Late controller discovery must not abort a valid recovery immediately.
    """
    preparation = ControllerPreparation(Mock(), Mock())
    preparation.controller_state = Mock(side_effect=[None, ControllerState.ACTIVE])
    values = [
        ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=1.0)
        for setting in preparation.calibration.settings
    ]
    preparation.calibration.read = Mock(side_effect=[None, values])
    monkeypatch.setattr(native_preparation.time, "sleep", Mock())
    configuration = preparation.read_configuration()
    assert configuration.state is ControllerState.ACTIVE
    assert configuration.values == values
    assert preparation.controller_state.call_count == 2


def test_unavailable_controller_discovery_has_a_deadline(monkeypatch) -> None:
    """
    Missing hardware services stop recovery instead of waiting indefinitely.
    """
    preparation = ControllerPreparation(Mock(), Mock())
    preparation.controller_state = Mock(return_value=None)
    preparation.calibration.read = Mock(return_value=None)
    clock = Mock(side_effect=[0.0, 0.0, 2.0])
    monkeypatch.setattr(native_preparation.time, "monotonic", clock)
    monkeypatch.setattr(native_preparation.time, "sleep", Mock())
    assert preparation.read_configuration(timeout=1.0) is None
    assert preparation.controller_state.call_count == 1
