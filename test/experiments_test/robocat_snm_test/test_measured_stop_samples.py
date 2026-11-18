"""
Measured completion reuses sustained wheel-stop checks without new thresholds.
"""

# %% imports
from unittest.mock import Mock

import pytest
from nav_msgs.msg import Odometry

from experiments.robocat_completion import MeasuredBaseStop, MeasuredStopContext
from experiments.robocat_goal_diagnostics import GoalDecisionObserver, DecisionStage
from experiments.robocat_safety import RobocatFeedback
from giskardpy.motion_statechart.context import MotionStatechartContext
from giskardpy.motion_statechart.data_types import ObservationStateValues
from giskardpy.motion_statechart.motion_statechart import MotionStatechart
from scripts import tiago_giskard_goal


# %% stable rest and restart
def test_stop_requires_existing_sample_count_and_dwell(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """
    Brief zero velocity does not terminate entry and renewed motion clears rest.
    """
    clock = Mock(return_value=100.0)
    monkeypatch.setattr(tiago_giskard_goal.time, "monotonic", clock)
    node = Mock()
    node.get_clock.return_value.now.return_value.nanoseconds = 10_000_000_000
    feedback = RobocatFeedback(node)
    context = MotionStatechartContext.empty()
    context.add_extension(MeasuredStopContext(feedback))
    monitor = MeasuredBaseStop()
    sample = Odometry()
    sample.header.stamp.sec = 10
    for moment in (100.0, 100.1):
        clock.return_value = moment
        feedback.receive_wheels(sample)
        assert monitor.on_tick(context) == ObservationStateValues.FALSE
    clock.return_value = 100.21
    feedback.receive_wheels(sample)
    assert monitor.on_tick(context) == ObservationStateValues.TRUE
    moving = Odometry()
    moving.header.stamp.sec = 10
    moving.twist.twist.linear.x = 0.1
    feedback.receive_wheels(moving)
    assert monitor.on_tick(context) == ObservationStateValues.FALSE
    assert monitor.last_sample.linear_velocity == 0.1


def test_measured_stop_is_included_in_decision_bag_payload() -> None:
    """
    The recorded verdict carries its own wheel stamp and measured velocities.
    """
    feedback = Mock()
    feedback.node.get_clock.return_value.now.return_value.nanoseconds = 10_000_000_000
    feedback.wheel_odometry = Odometry()
    feedback.wheel_odometry.header.stamp.sec = 10
    feedback.wheel_odometry.twist.twist.linear.x = 0.25
    feedback.wheels_settled.return_value = False
    context = MotionStatechartContext.empty()
    context.add_extension(MeasuredStopContext(feedback))
    monitor = MeasuredBaseStop(name="measured_base_stop")
    monitor.on_tick(context)
    chart = MotionStatechart()
    chart.add_node(monitor)
    executor = Mock()
    executor.motion_statechart = chart
    observer = GoalDecisionObserver(
        Mock(), executor, Mock(goal_id=1), Mock(synchronizers=[])
    )
    observer.capture(DecisionStage.STATE_CHANGE)
    message = observer.publisher.publish.call_args.args[0]
    values = {item.key: item.value for item in message.status[0].values}
    assert values["measured_stop.linear_velocity"] == "0.25"
    assert values["measured_stop.stamp_sec"] == "10"
    assert values["measured_stop.settled"] == "False"
