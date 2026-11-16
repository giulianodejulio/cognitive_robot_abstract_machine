"""
Diagnostic subscriptions follow goal ownership without changing native cycles.
"""

# %% imports
from unittest.mock import Mock

import pytest

from experiments.robocat_goal_diagnostics import DiagnosticControlLoop
from giskardpy.middleware.ros2.control_loop import ControlLoop


# %% successive goals
def test_loop_rebinds_history_without_replacing_controller_components(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """
    A later chart releases the old subscription and reuses the native executor.
    """
    executor = Mock()
    first = Mock(nodes=[])
    executor.motion_statechart = first
    native = ControlLoop(
        executor=executor,
        action_server=Mock(goal_id=1),
        feedback_publisher=Mock(),
        inputs=Mock(synchronizers=[]),
        cycle_counter=Mock(),
        client_watchdog=Mock(),
        world_updates=Mock(),
    )
    tick = Mock()
    monkeypatch.setattr(ControlLoop, "run_cycle", tick)
    loop = DiagnosticControlLoop.from_loop(native, Mock())
    loop.run_cycle()
    loop.run_cycle()
    first.history.add_observer.assert_called_once_with(loop.decisions)
    assert loop.executor is executor
    assert loop.command_publishers is native.command_publishers
    second = Mock(nodes=[])
    executor.motion_statechart = second
    native.action_server.goal_id = 2
    loop.run_cycle()
    first.history.remove_observer.assert_called_once_with(loop.decisions)
    second.history.add_observer.assert_called_once_with(loop.decisions)
    assert tick.call_count == 3
    loop.close()
    second.history.remove_observer.assert_called_once_with(loop.decisions)


def test_compile_failure_does_not_publish_a_previous_goals_stop() -> None:
    """
    A failed new goal cannot acquire the previous chart's diagnostic verdict.
    """
    native = ControlLoop(
        executor=Mock(),
        action_server=Mock(goal_id=2),
        feedback_publisher=Mock(),
        inputs=Mock(synchronizers=[]),
        cycle_counter=Mock(),
        client_watchdog=Mock(),
        world_updates=Mock(),
    )
    loop = DiagnosticControlLoop.from_loop(native, Mock())
    loop.observed_chart = native.executor.motion_statechart
    loop.observed_goal_id = 1
    loop.stop()
    loop.decisions.publisher.publish.assert_not_called()
