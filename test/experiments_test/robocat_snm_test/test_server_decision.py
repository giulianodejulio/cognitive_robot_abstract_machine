"""
Server telemetry preserves the task state at native decision time.
"""

# %% imports
from unittest.mock import Mock

from experiments.robocat_goal_diagnostics import (
    GoalDecisionObserver,
    GoalDiagnosticTopic,
)
from giskardpy.motion_statechart.tasks.cartesian_tasks import CartesianPositionStraight
from giskardpy.motion_statechart.data_types import (
    ObservationStateValues,
    LifeCycleValues,
)
from semantic_digital_twin.spatial_types import Pose


# %% exact decision snapshot
def test_decision_contains_compiled_error_target_and_measurement() -> None:
    """
    The diagnostic stores numeric copies rather than a later mutable model pose.
    """
    node = Mock()
    node.get_clock.return_value.now.return_value.to_msg.return_value.sec = 123
    task = Mock(spec=CartesianPositionStraight)
    task.name = "position"
    task.root_link = Mock()
    task.tip_link = Mock()
    task.root_link.name = "odom"
    task.tip_link.name = "base_footprint"
    task.root_T_goal_reference_frame = Pose.from_xyz_rpy().homogeneous_matrix
    task.goal_point = Pose.from_xyz_rpy(x=1.3).position
    task.threshold = 0.02
    task.error_signal.evaluate.return_value = [0.019]
    task.observation_state = ObservationStateValues.TRUE
    task.life_cycle_state = LifeCycleValues.RUNNING
    executor = Mock()
    executor.control_cycles = 40
    executor.motion_statechart.nodes = [task]
    executor.motion_statechart.is_end_motion.return_value = True
    executor.context.world.compute_forward_kinematics.return_value = Pose.from_xyz_rpy(
        x=1.281
    ).homogeneous_matrix
    observer = GoalDecisionObserver(
        node, executor, Mock(goal_id=7), Mock(synchronizers=[])
    )
    observer.on_state_change(Mock())
    message = node.create_publisher.return_value.publish.call_args.args[0]
    status = next(item for item in message.status if item.name == "position")
    values = {item.key: item.value for item in status.values}
    assert values["task_error"] == "0.019"
    assert values["threshold"] == "0.02"
    assert values["target.x"] == "1.3"
    assert values["current.x"] == "1.281"
    assert values["goal_id"] == "7"
    assert values["end_motion"] == "True"
    assert status.hardware_id == "base_footprint"
    assert message.header.stamp.sec == 123
    assert node.create_publisher.call_args.args[1] == GoalDiagnosticTopic.DECISION
