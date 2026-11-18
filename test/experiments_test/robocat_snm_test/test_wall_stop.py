"""Lidar stop geometry remains independent of map pose and wheel scale."""

# %% imports
import math
import numpy as np
import pytest
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from experiments.robocat_wall import CabinWallEstimator, WallStopController, WallStopSettings, WallMotionProgress
from experiments.robocat_snm.odometry_history import OdometryHistory
from experiments.tiago_goal_logging import PlanarPose


# %% geometric scan fixtures
@pytest.fixture
def wall_scan() -> LaserScan:
    """A wide wall one metre in front of a forward-mounted scanner."""
    scan = LaserScan()
    scan.header.frame_id = "laser_link"
    scan.header.stamp.sec = 10
    scan.angle_min = -math.pi / 4
    scan.angle_increment = math.pi / 720
    scan.range_min = 0.05
    scan.range_max = 10.0
    angles = scan.angle_min + np.arange(361) * scan.angle_increment
    scan.ranges = (1 / np.cos(angles)).tolist()
    return scan


@pytest.fixture
def laser_transform() -> np.ndarray:
    """URDF-derived forward lidar displacement in the base frame."""
    transform = np.eye(4)
    transform[0, 3] = 0.113
    return transform


def test_wall_distance_includes_lidar_offset(wall_scan: LaserScan, laser_transform: np.ndarray) -> None:
    fit = CabinWallEstimator().estimate(wall_scan, laser_transform)
    assert fit is not None
    assert fit.distance == pytest.approx(1.113, abs=1e-5)


def test_wall_fit_compensates_heading(wall_scan: LaserScan, laser_transform: np.ndarray) -> None:
    angle = 0.12
    angles = wall_scan.angle_min + np.arange(len(wall_scan.ranges)) * wall_scan.angle_increment
    wall_scan.ranges = (1 / np.cos(angles - angle)).tolist()
    fit = CabinWallEstimator().estimate(wall_scan, laser_transform)
    assert fit is not None
    assert fit.distance == pytest.approx(1 + 0.113 * math.cos(angle), abs=1e-5)


def test_no_return_cannot_define_stop_target(wall_scan: LaserScan, laser_transform: np.ndarray) -> None:
    wall_scan.ranges = [math.inf] * len(wall_scan.ranges)
    assert CabinWallEstimator().estimate(wall_scan, laser_transform) is None


def test_delayed_scan_is_compensated_with_wheel_pose(wall_scan: LaserScan, laser_transform: np.ndarray) -> None:
    history = OdometryHistory()
    acquisition = Odometry()
    acquisition.header.frame_id = "odom"
    acquisition.child_frame_id = "base_footprint"
    acquisition.header.stamp.sec = 10
    acquisition.pose.pose.orientation.w = 1.0
    history.samples.append(acquisition)
    current = Odometry()
    current.header.frame_id = "odom"
    current.child_frame_id = "base_footprint"
    current.header.stamp.sec = 10
    current.header.stamp.nanosec = 100_000_000
    current.pose.pose.orientation.w = 1.0
    current.pose.pose.position.x = 0.03
    controller = WallStopController(WallStopSettings(reference_distance=0.936), PlanarPose(0.18, 0, 0), laser_transform)
    sample = controller.observe(wall_scan, current, history, now=10.1)
    assert sample.valid is True
    assert sample.distance == pytest.approx(1.083, abs=1e-5)
    assert sample.target_progress == pytest.approx(1.113 - 0.936, abs=1e-5)
    stale = controller.observe(wall_scan, current, history, now=11.0)
    assert stale.valid is False


# %% bounded unavailable and stalled cases


def test_stationary_precision_attempts_are_bounded() -> None:
    guard = WallMotionProgress(limit=1.0)
    assert guard.blocked(0.0, stationary=True, reached=False) is False
    assert guard.blocked(1.0, stationary=False, reached=False) is False
    assert guard.blocked(2.0, stationary=True, reached=False) is False
    assert guard.blocked(3.0, stationary=True, reached=False) is True
    assert guard.blocked(3.1, stationary=True, reached=True) is False
