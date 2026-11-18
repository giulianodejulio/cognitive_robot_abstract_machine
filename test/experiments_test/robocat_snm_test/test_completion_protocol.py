"""
The measured-stop gate survives the native goal protocol and fails closed.
"""

# %% imports
import json
from unittest.mock import Mock

import pytest

from experiments.robocat_completion import MeasuredBaseStop, MeasuredStopContext
from giskardpy.motion_statechart.context import MotionStatechartContext
from giskardpy.motion_statechart.data_types import ObservationStateValues
from giskardpy.motion_statechart.exceptions import MissingContextExtensionError
from giskardpy.motion_statechart.motion_statechart import MotionStatechart


# %% request serialization
def test_stop_gate_survives_native_chart_serialization() -> None:
    """
    A server receives the stop requirement rather than a client-only condition.
    """
    chart = MotionStatechart()
    chart.add_node(MeasuredBaseStop(name="measured_base_stop"))
    restored = MotionStatechart.from_json(json.loads(json.dumps(chart.to_json())))
    assert len(restored.nodes) == 1
    assert isinstance(restored.nodes[0], MeasuredBaseStop)
    assert restored.nodes[0].maximum_age == 0.5


# %% unavailable feedback
def test_server_without_measured_feedback_rejects_gate() -> None:
    """
    An old or unconfigured server cannot silently accept estimated settling.
    """
    with pytest.raises(MissingContextExtensionError):
        MeasuredBaseStop().build_artifacts(MotionStatechartContext.empty())


def test_missing_wheel_sample_is_unknown() -> None:
    """
    Absence of measured motion is not evidence of a stationary base.
    """
    context = MotionStatechartContext.empty()
    context.add_extension(MeasuredStopContext(Mock(wheel_odometry=None)))
    assert MeasuredBaseStop().on_tick(context) == ObservationStateValues.UNKNOWN
