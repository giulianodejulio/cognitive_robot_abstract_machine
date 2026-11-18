"""
Wall identity, data validity and target-rate limits fail closed.
"""

# %% imports
import numpy as np
import pytest
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from experiments.robocat_wall import WallStopController, WallStopSettings
from experiments.robocat_snm.odometry_history import OdometryHistory
from experiments.tiago_goal_logging import PlanarPose
from .test_wall_stop import wall_scan, laser_transform


@pytest.fixture
def wheel_sample() -> Odometry:
    """
    A fresh stationary wheel pose in the configured native frame.
    """
    wheel = Odometry()
    wheel.header.stamp.sec = 10
    wheel.header.frame_id = "odom"
    wheel.child_frame_id = "base_footprint"
    wheel.pose.pose.orientation.w = 1.0
    return wheel


# %% continuity and target filtering
def test_wall_jump_is_rejected_without_moving_endpoint(
    wall_scan: LaserScan, laser_transform: np.ndarray, wheel_sample: Odometry
) -> None:
    controller = WallStopController(
        WallStopSettings(0.936), PlanarPose(0.18, 0, 0), laser_transform
    )
    history = OdometryHistory()
    history.samples.append(wheel_sample)
    first = controller.observe(wall_scan, wheel_sample, history, 10.0)
    assert first.valid is True
    wall_scan.header.stamp.nanosec = 100_000_000
    wheel_sample.header.stamp.nanosec = 100_000_000
    wall_scan.ranges = (np.asarray(wall_scan.ranges) * 1.12).tolist()
    rejected = controller.observe(wall_scan, wheel_sample, history, 10.1)
    assert rejected.valid is False
    assert controller.progress == first.target_progress


def test_target_updates_are_smooth_and_rate_bounded(
    wall_scan: LaserScan, laser_transform: np.ndarray, wheel_sample: Odometry
) -> None:
    controller = WallStopController(
        WallStopSettings(0.936), PlanarPose(0.18, 0, 0), laser_transform
    )
    history = OdometryHistory()
    history.samples.append(wheel_sample)
    first = controller.observe(wall_scan, wheel_sample, history, 10.0)
    wall_scan.header.stamp.nanosec = 100_000_000
    wheel_sample.header.stamp.nanosec = 100_000_000
    wall_scan.ranges = (np.asarray(wall_scan.ranges) * 1.02).tolist()
    updated = controller.observe(wall_scan, wheel_sample, history, 10.1)
    assert updated.valid is True
    assert (
        0
        < updated.target_progress - first.target_progress
        <= controller.settings.maximum_target_speed * 0.1
    )


def test_missing_scan_pose_match_does_not_create_target(
    wall_scan: LaserScan, laser_transform: np.ndarray, wheel_sample: Odometry
) -> None:
    controller = WallStopController(
        WallStopSettings(0.936), PlanarPose(0.18, 0, 0), laser_transform
    )
    assert (
        controller.observe(wall_scan, wheel_sample, OdometryHistory(), 10.0).valid
        is False
    )
    assert controller.progress is None


def test_invalid_wheel_pose_does_not_create_target(
    wall_scan: LaserScan, laser_transform: np.ndarray, wheel_sample: Odometry
) -> None:
    controller = WallStopController(
        WallStopSettings(0.936), PlanarPose(0.18, 0, 0), laser_transform
    )
    wheel_sample.pose.pose.orientation.w = 0.0
    history = OdometryHistory()
    history.samples.append(wheel_sample)
    assert controller.observe(wall_scan, wheel_sample, history, 10.0).valid is False
    assert controller.progress is None
