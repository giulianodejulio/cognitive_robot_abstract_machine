"""
Native orientation constraints execute a half turn without forward travel.
"""

# %% imports
from unittest.mock import Mock
import math

import pytest

from nav_msgs.msg import Odometry

from experiments.robocat_completion import MeasuredStopContext
from experiments.robocat_snm.cabin_turn import RobocatCabinTurnMotion
from giskardpy.executor import Executor
from giskardpy.middleware.ros2.scripts.iai_robots.robocat import RobocatMotionLimits
from giskardpy.motion_statechart.context import MotionStatechartContext
from semantic_digital_twin.spatial_types import Pose
from semantic_digital_twin.world_description.connections import DifferentialDrive
from semantic_digital_twin.world import World
from giskardpy.motion_statechart.motion_statechart import MotionStatechart
from giskardpy.motion_statechart.graph_node import EndMotion


# %% ideal stationary-centre plant
@pytest.mark.parametrize("initial_yaw", [0.0, 2.1426351199754463, -2.8, 0.7])
@pytest.mark.parametrize("heading_offset", [-0.00058, 0.0, 0.00058])
def test_native_turn_reaches_opposite_heading_at_rest_without_forward_command(
    cylinder_bot_diff_world: World,
    initial_yaw: float,
    heading_offset: float,
) -> None:
    """
    The actual QP reaches 180 degrees within PAL bounds and retains the base centre.
    """
    world = cylinder_bot_diff_world
    drive = world.get_connections_by_type(DifferentialDrive)[0]
    world.state[drive.yaw.id].position = initial_yaw
    target_yaw = initial_yaw + math.pi + heading_offset
    limits = RobocatMotionLimits(maximum_forward_velocity=0.3, maximum_yaw_velocity=0.2)
    limits.apply_to(drive)
    motion = RobocatCabinTurnMotion(
        Pose.from_xyz_rpy(yaw=target_yaw, reference_frame=world.root),
        orientation_tolerance_deg=5.0,
    )
    motion.plan_node = Mock()
    motion.plan_node.plan.world = world
    motion.plan_node.plan.robot.root = drive.child
    goal = motion._motion_chart
    chart = MotionStatechart()
    chart.add_node(goal)
    chart.add_node(EndMotion.when_true(goal))
    context = MotionStatechartContext(
        world=world, qp_controller_config=limits.controller_configuration()
    )
    feedback = Mock()
    context.add_extension(MeasuredStopContext(feedback))
    executor = Executor(context)
    executor.compile(chart)
    interval = context.qp_controller_config.control_time_step.total_seconds()
    forward = []
    angular = []
    for index in range(500):
        now = 10.0 + index * interval
        feedback.node.get_clock.return_value.now.return_value.nanoseconds = round(
            now * 1e9
        )
        wheel = Odometry()
        wheel.header.stamp.sec, wheel.header.stamp.nanosec = divmod(
            round(now * 1e9), 1_000_000_000
        )
        wheel.twist.twist.angular.z = world.state[drive.yaw.id].velocity
        feedback.wheel_odometry = wheel
        feedback.wheels_settled.return_value = abs(wheel.twist.twist.angular.z) < 0.001
        executor.tick()
        forward.append(world.state[drive.x_velocity.id].velocity)
        angular.append(world.state[drive.yaw.id].velocity)
        if executor.motion_statechart.is_end_motion():
            break
    assert executor.motion_statechart.is_end_motion()
    final = world.compute_forward_kinematics(world.root, drive.child)
    yaw = float(Pose(position=final.position, orientation=final.quaternion).yaw)
    assert abs(
        math.atan2(math.sin(yaw - target_yaw), math.cos(yaw - target_yaw))
    ) <= math.radians(5.0)
    assert math.hypot(float(final.position.x), float(final.position.y)) < 1e-6
    assert max(abs(value) for value in forward) < 1e-6
    assert max(abs(value) for value in angular) <= limits.maximum_yaw_velocity + 1e-6
    assert abs(angular[-1]) < 0.001
