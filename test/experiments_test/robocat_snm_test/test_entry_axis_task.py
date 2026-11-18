"""
Native task completion with the recorded lateral offset.
"""

# %% imports
from pathlib import Path
from unittest.mock import Mock

from nav_msgs.msg import Odometry
from coraplex.datastructures.dataclasses import Context
from coraplex.plans.factories import execute_single
from experiments.robocat_completion import MeasuredStopContext
from experiments.robocat_goal_diagnostics import GoalDecisionObserver
from experiments.robocat_snm.entry_motion import (
    EntryPositionStraight,
    RobocatEntryMotion,
)
from giskardpy.executor import Executor
from giskardpy.middleware.ros2.scripts.iai_robots.robocat import WorldWithRobocatConfig
from giskardpy.motion_statechart.context import MotionStatechartContext
from semantic_digital_twin.robots.robocat import Robocat
from semantic_digital_twin.spatial_types import Pose
from semantic_digital_twin.world_description.connections import DifferentialDrive


# %% recorded stop accepted by native chart
def test_native_entry_finishes_at_rest_with_recorded_lateral_offset() -> None:
    """
    A 2.61 cm lateral residual cannot keep a stopped entry running.
    """
    configuration = WorldWithRobocatConfig(
        urdf=Path(Robocat.get_ros_file_path()).read_text()
    )
    world = configuration.world
    with world.modify_world():
        configuration.setup_world()
        drive = world.get_connections_by_type(DifferentialDrive)[0]
        drive.has_hardware_interface = True
    target = Pose.from_xyz_rpy(x=1.3, reference_frame=world.root)
    plan = execute_single(
        RobocatEntryMotion(target),
        context=Context(world=world, robot=configuration.robot),
    )
    executable = plan.parse()
    executable.prepare_for_execution()
    context = MotionStatechartContext(world=world)
    feedback = Mock()
    feedback.node.get_clock.return_value.now.return_value.nanoseconds = 10_000_000_000
    feedback.wheel_odometry = Odometry()
    feedback.wheel_odometry.header.stamp.sec = 10
    feedback.wheels_settled.return_value = True
    context.add_extension(MeasuredStopContext(feedback))
    executor = Executor(context)
    executor.compile(executable.motion_state_chart)
    executor.control_cycles = 100
    drive.origin = Pose.from_xyz_rpy(x=1.3, y=0.0261).homogeneous_matrix
    world.notify_state_change()
    for _ in range(10):
        executor.motion_statechart.tick(context)
        if executor.motion_statechart.is_end_motion():
            break
    assert executor.motion_statechart.is_end_motion()
    task = next(
        node
        for node in executor.motion_statechart.nodes
        if isinstance(node, EntryPositionStraight)
    )
    values = GoalDecisionObserver(
        node=Mock(), executor=executor, action_server=Mock(), inputs=Mock()
    ).task_values(task)
    assert float(values["lateral_tolerance"]) == task.lateral_tolerance
    assert float(values["longitudinal_error"]) == 0.0
