"""
Entry constraints remain active until the measured base has stopped at the goal.
"""

# %% imports
from unittest.mock import Mock

import pytest
from nav_msgs.msg import Odometry

from coraplex.datastructures.dataclasses import Context
from coraplex.plans.factories import execute_single
from experiments.robocat_completion import MeasuredBaseStop, MeasuredStopContext
from experiments.robocat_snm.entry_motion import RobocatEntryMotion
from giskardpy.executor import Executor
from giskardpy.middleware.ros2.scripts.iai_robots.robocat import WorldWithRobocatConfig
from giskardpy.motion_statechart.context import MotionStatechartContext
from giskardpy.motion_statechart.data_types import (
    LifeCycleValues,
    ObservationStateValues,
)
from giskardpy.motion_statechart.tasks.cartesian_tasks import CartesianPositionStraight
from semantic_digital_twin.robots.robocat import Robocat
from semantic_digital_twin.spatial_types import Pose, HomogeneousTransformationMatrix
from semantic_digital_twin.world_description.connections import DifferentialDrive
from pathlib import Path


# %% freshness at completion
@pytest.mark.parametrize(
    "age, settled, expected",
    [
        (0.05, False, ObservationStateValues.FALSE),
        (0.05, True, ObservationStateValues.TRUE),
        (0.6, True, ObservationStateValues.UNKNOWN),
    ],
)
def test_measured_stop_requires_fresh_stationary_feedback(
    age: float, settled: bool, expected: ObservationStateValues
) -> None:
    """
    A stopped internal model cannot substitute for fresh wheel observations.
    """
    feedback = Mock()
    feedback.node.get_clock.return_value.now.return_value.nanoseconds = 10_000_000_000
    feedback.wheel_odometry = Odometry()
    feedback.wheel_odometry.header.stamp.sec = 9
    feedback.wheel_odometry.header.stamp.nanosec = round((1 - age) * 1e9)
    feedback.wheels_settled.return_value = settled
    context = MotionStatechartContext.empty()
    context.add_extension(MeasuredStopContext(feedback))
    assert MeasuredBaseStop().on_tick(context) == expected


# %% transient reach must not latch success
def test_entry_keeps_constraints_after_crossing_goal_while_moving() -> None:
    """
    Overshoot remains correctable and only a stopped in-tolerance pose succeeds.
    """
    configuration = WorldWithRobocatConfig(
        urdf=Path(Robocat.get_ros_file_path()).read_text()
    )
    world = configuration.world
    with world.modify_world():
        configuration.setup_world()
        drive = world.get_connections_by_type(DifferentialDrive)[0]
        drive.has_hardware_interface = True
    context = Context(world=world, robot=configuration.robot)
    target = Pose.from_xyz_rpy(x=1.3, reference_frame=world.root)
    plan = execute_single(RobocatEntryMotion(target), context=context)
    executable = plan.parse()
    executable.prepare_for_execution()
    executor_context = MotionStatechartContext(world=world)
    feedback = Mock()
    feedback.node.get_clock.return_value.now.return_value.nanoseconds = 10_000_000_000
    feedback.wheel_odometry = Odometry()
    feedback.wheel_odometry.header.stamp.sec = 10
    feedback.wheels_settled.return_value = False
    executor_context.add_extension(MeasuredStopContext(feedback))
    executor = Executor(executor_context)
    executor.compile(executable.motion_state_chart)
    executor.control_cycles = 100
    chart = executor.motion_statechart
    position = next(n for n in chart.nodes if isinstance(n, CartesianPositionStraight))
    for x, yaw, settled in (
        (1.29, 0.0, False),
        (1.4, 0.0, False),
        (1.4, 0.0, True),
        (1.29, 0.2, True),
    ):
        drive.origin = Pose.from_xyz_rpy(x=x, yaw=yaw).homogeneous_matrix
        world.notify_state_change()
        feedback.wheels_settled.return_value = settled
        for _ in range(10):
            chart.tick(executor_context)
        assert not chart.is_end_motion()
        assert position.life_cycle_state == LifeCycleValues.RUNNING
    drive.origin = HomogeneousTransformationMatrix.from_xyz_quaternion(pos_x=1.29)
    world.notify_state_change()
    world.state[drive.x_velocity.id].velocity = 0.1
    for _ in range(10):
        chart.tick(executor_context)
    assert not chart.is_end_motion()
    assert position.life_cycle_state == LifeCycleValues.RUNNING
    world.state[drive.x_velocity.id].velocity = 0.0
    for _ in range(10):
        chart.tick(executor_context)
        if chart.is_end_motion():
            break
    assert chart.is_end_motion()
