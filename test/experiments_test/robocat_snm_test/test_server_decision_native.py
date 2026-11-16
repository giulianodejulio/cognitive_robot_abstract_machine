"""
Decision telemetry observes a real compiled chart before state prediction.
"""

# %% imports
from unittest.mock import Mock

import pytest

from experiments.robocat_goal_diagnostics import GoalDecisionObserver
from giskardpy.executor import Executor
from giskardpy.motion_statechart.context import MotionStatechartContext
from giskardpy.motion_statechart.graph_node import EndMotion
from giskardpy.motion_statechart.motion_statechart import MotionStatechart
from giskardpy.motion_statechart.tasks.cartesian_tasks import CartesianPositionStraight
from semantic_digital_twin.spatial_types import Pose, HomogeneousTransformationMatrix
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import DifferentialDrive


# %% native decision timing
def test_native_success_snapshot_precedes_command_integration(
    cylinder_bot_diff_world: World,
) -> None:
    """
    The event contains the actual compiled task error at native completion.
    """
    world = cylinder_bot_diff_world
    drive = world.get_connections_by_type(DifferentialDrive)[0]
    chart = MotionStatechart()
    task = CartesianPositionStraight(
        name="entry_position",
        root_link=world.root,
        tip_link=drive.child,
        goal_point=Pose.from_xyz_rpy(x=1.3, reference_frame=world.root).position,
        threshold=0.02,
    )
    chart.add_node(task)
    end = EndMotion()
    end.start_condition = task.goal_reached
    chart.add_node(end)
    executor = Executor(MotionStatechartContext(world=world))
    executor.compile(chart)
    executor.control_cycles = (
        2 / executor.context.qp_controller_config.control_time_step.total_seconds()
    )
    node = Mock()
    observer = GoalDecisionObserver(
        node, executor, Mock(goal_id=12), Mock(synchronizers=[])
    )
    chart.history.add_observer(observer)
    drive.origin = HomogeneousTransformationMatrix.from_xyz_quaternion(pos_x=1.281)
    world.notify_state_change()
    for _ in range(10):
        chart.tick(executor.context)
        if chart.is_end_motion():
            break
    assert chart.is_end_motion()
    messages = [call.args[0] for call in observer.publisher.publish.call_args_list]
    events = [
        item
        for message in messages
        for item in message.status
        if item.name == str(task.name)
    ]
    final = {value.key: value.value for value in events[-1].values}
    assert final["end_motion"] == "True"
    assert float(final["task_error"]) == pytest.approx(0.019)
    assert float(final["current.x"]) == pytest.approx(1.281)
    assert float(final["target.x"]) == pytest.approx(1.3)
    drive.origin = HomogeneousTransformationMatrix.from_xyz_quaternion(pos_x=1.4)
    world.notify_state_change()
    assert float(final["current.x"]) == pytest.approx(1.281)
