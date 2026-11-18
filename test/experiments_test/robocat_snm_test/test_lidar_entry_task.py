"""
A compiled native motion stops at the lidar reference rather than the old waypoint.
"""

# %% imports
from pathlib import Path
from unittest.mock import Mock

import numpy as np
import pytest
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from coraplex.datastructures.dataclasses import Context
from coraplex.plans.factories import execute_single
from experiments.robocat_completion import MeasuredStopContext
from experiments.robocat_wall import WallStopSettings
from experiments.robocat_snm.entry_motion import RobocatEntryMotion
from experiments.robocat_snm.lidar_entry import (
    LidarLongitudinalTask,
    WallControlContext,
)
from experiments.robocat_snm.odometry_history import OdometryHistory
from giskardpy.executor import Executor
from giskardpy.middleware.ros2.scripts.iai_robots.robocat import WorldWithRobocatConfig
from giskardpy.motion_statechart.context import MotionStatechartContext
from semantic_digital_twin.robots.robocat import Robocat
from semantic_digital_twin.spatial_types import Pose
from semantic_digital_twin.world_description.connections import DifferentialDrive
from .test_wall_stop import wall_scan, laser_transform


# %% measured reference overrides nominal position
def test_native_motion_uses_updated_wall_target_and_measured_rest(
    wall_scan: LaserScan, laser_transform: np.ndarray
) -> None:
    """
    The nominal goal may be passed while the robot still owes travel to the wall
    reference.
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
    executable = execute_single(
        RobocatEntryMotion(target, wall_stop=WallStopSettings(0.936)),
        context=Context(world=world, robot=configuration.robot),
    ).parse()
    executable.prepare_for_execution()
    feedback = Mock()
    feedback.node.get_clock.return_value.now.return_value.nanoseconds = 10_000_000_000
    wheel = Odometry()
    wheel.header.frame_id = "odom"
    wheel.child_frame_id = "base_footprint"
    wheel.header.stamp.sec = 10
    wheel.pose.pose.orientation.w = 1.0
    feedback.wheel_odometry = wheel
    feedback.wheel_history = OdometryHistory()
    feedback.wheel_history.samples.append(wheel)
    feedback.wheels_settled.return_value = False
    angles = (
        wall_scan.angle_min
        + np.arange(len(wall_scan.ranges)) * wall_scan.angle_increment
    )
    wall_scan.ranges = ((1.5 + 0.936 - 0.113) / np.cos(angles)).tolist()
    source = Mock(has_reading=True, received_scan=wall_scan)
    context = MotionStatechartContext(world=world)
    context.add_extension(MeasuredStopContext(feedback))
    context.add_extension(WallControlContext(source, feedback, laser_transform))
    executor = Executor(context)
    executor.compile(executable.motion_state_chart)
    executor.control_cycles = 100
    task = next(
        node
        for node in executor.motion_statechart.nodes
        if isinstance(node, LidarLongitudinalTask)
    )
    assert task.last_sample.target_progress == pytest.approx(1.5, abs=1e-5)
    for index, position in enumerate((1.3, 1.5), start=1):
        drive.origin = Pose.from_xyz_rpy(x=position).homogeneous_matrix
        world.notify_state_change()
        current = Odometry()
        current.header.frame_id = "odom"
        current.child_frame_id = "base_footprint"
        current.header.stamp.sec = 10 + index
        current.pose.pose.orientation.w = 1.0
        current.pose.pose.position.x = position
        feedback.wheel_odometry = current
        feedback.wheel_history.samples.append(current)
        feedback.node.get_clock.return_value.now.return_value.nanoseconds = (
            10 + index
        ) * 1_000_000_000
        wall_scan.header.stamp.sec = 10 + index
        wall_scan.ranges = ((1.5 + 0.936 - 0.113 - position) / np.cos(angles)).tolist()
        for _ in range(5):
            executor.motion_statechart.tick(context)
        assert not executor.motion_statechart.is_end_motion()
    feedback.wheels_settled.return_value = True
    for _ in range(10):
        executor.motion_statechart.tick(context)
        if executor.motion_statechart.is_end_motion():
            break
    assert executor.motion_statechart.is_end_motion()
    assert task.last_sample.distance == pytest.approx(0.936, abs=1e-5)
