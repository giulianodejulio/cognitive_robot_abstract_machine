"""
The native QP can follow the wall endpoint under PAL dynamics.
"""

# %% imports
from unittest.mock import Mock
import math
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from experiments.robocat_completion import MeasuredBaseStop, MeasuredStopContext
from experiments.robocat_wall import WallStopSettings
from experiments.robocat_snm.lidar_entry import (
    LidarLongitudinalTask,
    WallControlContext,
)
from experiments.robocat_snm.odometry_history import OdometryHistory
from giskardpy.executor import Executor
from giskardpy.middleware.ros2.scripts.iai_robots.robocat import RobocatMotionLimits
from giskardpy.motion_statechart.context import MotionStatechartContext
from giskardpy.motion_statechart.goals.templates import Parallel
from giskardpy.motion_statechart.graph_node import EndMotion
from giskardpy.motion_statechart.motion_statechart import MotionStatechart
from giskardpy.motion_statechart.monitors.monitors import LocalMinimumReached
from semantic_digital_twin.spatial_types import Pose
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import DifferentialDrive
from .test_wall_stop import wall_scan, laser_transform


# %% ideal-plant offline command envelope
def test_wall_qp_reaches_lidar_target_with_pal_dynamics(
    cylinder_bot_diff_world: World, wall_scan: LaserScan, laser_transform: np.ndarray
) -> None:
    """
    Actual optimized commands reach the updated endpoint without bypassing PAL bounds.
    """
    world = cylinder_bot_diff_world
    drive = world.get_connections_by_type(DifferentialDrive)[0]
    limits = RobocatMotionLimits(maximum_forward_velocity=0.3, maximum_yaw_velocity=0.2)
    limits.apply_to(drive)
    reference, desired_x = 0.936, 0.7
    task = LidarLongitudinalTask(
        root_link=world.root,
        tip_link=drive.child,
        goal_pose=Pose.from_xyz_rpy(x=0.5, reference_frame=world.root),
        settings=WallStopSettings(reference),
        threshold=0.05,
    )
    chart = MotionStatechart()
    goal = Parallel(
        nodes=[
            task,
            MeasuredBaseStop(),
            LocalMinimumReached(minimum_time=0.0, measure_from_own_start=False),
        ]
    )
    chart.add_node(goal)
    chart.add_node(EndMotion.when_true(goal))
    context = MotionStatechartContext(
        world=world, qp_controller_config=limits.controller_configuration()
    )
    feedback = Mock()
    feedback.wheel_history = OdometryHistory()
    source = Mock(has_reading=True, received_scan=wall_scan)
    context.add_extension(MeasuredStopContext(feedback))
    context.add_extension(WallControlContext(source, feedback, laser_transform))
    interval = context.qp_controller_config.control_time_step.total_seconds()
    velocities = [0.0]
    settled_since = 10.0
    executor = Executor(context)
    for index in range(250):
        now = 10 + index * interval
        current = world.compute_forward_kinematics(world.root, drive.child)
        wheel = Odometry()
        nanos = round(now * 1e9)
        wheel.header.stamp.sec = nanos // 1_000_000_000
        wheel.header.stamp.nanosec = nanos % 1_000_000_000
        wheel.header.frame_id = "odom"
        wheel.child_frame_id = "base_footprint"
        wheel.pose.pose.position.x = float(current.position.x)
        wheel.pose.pose.position.y = float(current.position.y)
        wheel.pose.pose.orientation.w = 1.0
        velocity = world.state[drive.x_velocity.id].velocity
        wheel.twist.twist.linear.x = float(velocity)
        if abs(velocity) >= 0.005:
            settled_since = now
        feedback.wheel_odometry = wheel
        feedback.wheel_history.samples.append(wheel)
        feedback.wheels_settled.return_value = now - settled_since >= 0.2
        feedback.node.get_clock.return_value.now.return_value.nanoseconds = nanos
        wall_scan.header.stamp = wheel.header.stamp
        angles = (
            wall_scan.angle_min
            + np.arange(len(wall_scan.ranges)) * wall_scan.angle_increment
        )
        wall_scan.ranges = (
            (reference + desired_x - 0.113 - float(current.position.x)) / np.cos(angles)
        ).tolist()
        if index == 0:
            executor.compile(chart)
        executor.tick()
        velocities.append(world.state[drive.x_velocity.id].velocity)
        if chart.is_end_motion():
            break
    assert chart.is_end_motion()
    final = float(world.compute_forward_kinematics(world.root, drive.child).position.x)
    assert abs(final - desired_x) <= task.threshold
    assert final > 0.6
    acceleration = np.diff(velocities) / interval
    jerk = np.diff(np.r_[0.0, acceleration]) / interval
    assert max(abs(acceleration)) <= limits.linear.acceleration + 1e-5
    assert max(abs(jerk)) <= limits.linear.jerk + 1e-5
