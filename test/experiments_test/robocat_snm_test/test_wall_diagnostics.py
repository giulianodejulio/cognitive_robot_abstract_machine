"""
Native wall decisions and continuous SNM measurements remain recordable.
"""

# %% imports
from unittest.mock import Mock
from pathlib import Path
from builtin_interfaces.msg import Time
from experiments.robocat_goal_diagnostics import GoalDecisionObserver
from experiments.robocat_wall import WallSample, WallStopSettings, WallDiagnostic
from experiments.robocat_snm.lidar_entry import LidarLongitudinalTask
from giskardpy.motion_statechart.data_types import (
    ObservationStateValues,
    LifeCycleValues,
)


# %% synchronous native decision evidence
def test_native_decision_includes_wall_reference_and_endpoint() -> None:
    """
    The recorded success evidence contains the actual lidar stop target.
    """
    node = Mock()
    node.get_clock.return_value.now.return_value.to_msg.return_value = Time(sec=10)
    node.get_clock.return_value.now.return_value.nanoseconds = 10_000_000_000
    task = Mock(spec=LidarLongitudinalTask)
    task.name = "wall_longitudinal"
    task.tip_link = Mock()
    task.tip_link.name = "base_footprint"
    task.threshold = 0.05
    task.settings = WallStopSettings(0.936)
    task.last_sample = WallSample(True, "", 10.0, 10.0, 0.936, 0.0, 1.5, 0.2)
    task.observation_state = ObservationStateValues.TRUE
    task.life_cycle_state = LifeCycleValues.RUNNING
    executor = Mock()
    executor.control_cycles = 40
    executor.motion_statechart.nodes = [task]
    executor.motion_statechart.is_end_motion.return_value = True
    observer = GoalDecisionObserver(
        node, executor, Mock(goal_id=7), Mock(synchronizers=[])
    )
    observer.on_state_change(Mock())
    message = node.create_publisher.return_value.publish.call_args.args[0]
    values = {value.key: value.value for value in message.status[0].values}
    assert values["settings.reference_distance"] == "0.936"
    assert values["sample.target_progress"] == "1.5"
    assert values["sample.distance"] == "0.936"
    assert values["end_motion"] == "True"


def test_recorder_includes_continuous_wall_tracking() -> None:
    """
    Raw sensors and native decisions are supplemented by the live wall estimate.
    """
    recorder = Path(__file__).resolve().parents[3] / "scripts/robocat-record-entry"
    assert WallDiagnostic.TOPIC in recorder.read_text()
