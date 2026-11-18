"""
SNM and the native task use the same wall-distance criterion.
"""

# %% imports
from dataclasses import replace
import numpy as np
import pytest
from experiments.robocat_snm.geometry import EntryGeometry
from experiments.robocat_snm.feedback import EntryInputs
from sensor_msgs.msg import LaserScan
from experiments.robocat_wall import WallStopController, WallStopSettings
from experiments.robocat_snm.odometry_history import OdometryHistory
from experiments.robocat_waypoints.model import WaypointSnapshot
from experiments.tiago_goal_logging import PlanarPose
from semantic_digital_twin.spatial_types import Pose
from .test_feedback import entry_inputs
from .test_wall_stop import wall_scan, laser_transform


# %% reference replaces map precision
@pytest.mark.parametrize(
    "distance,expected", [(0.936, True), (1.149, False), (0.80, False)]
)
def test_final_wall_reference_is_independent_of_nominal_map_error(
    distance: float, expected: bool
) -> None:
    geometry = EntryGeometry(Pose.from_xyz_rpy(), doorway_distance=0.86, distance=1.3)
    assessment = replace(
        geometry.assess(Pose.from_xyz_rpy(x=1.5, y=0.07, yaw=0.1)),
        wall_reference=0.936,
        wall_distance=distance,
    )
    assert assessment.target_reached is expected
    assert assessment.fully_inside is expected


def test_snm_accepts_measured_wall_reference_after_nominal_waypoint(
    entry_inputs: EntryInputs, wall_scan: LaserScan, laser_transform: np.ndarray
) -> None:
    settings = WallStopSettings(0.936)
    inputs = entry_inputs
    inputs.wall_settings = settings
    inputs.wall_controller = WallStopController(
        settings, PlanarPose(1.3, 0, 0), laser_transform
    )
    inputs.frozen_geometry = EntryGeometry(
        Pose.from_xyz_rpy(), doorway_distance=0.86, distance=1.3
    )
    wheel = inputs.feedback.wheel_odometry
    wheel.header.frame_id = "odom"
    wheel.child_frame_id = "base_footprint"
    wheel.pose.pose.position.x = 1.5
    inputs.feedback.wheel_history = OdometryHistory()
    inputs.feedback.wheel_history.samples.append(wheel)
    inputs.snapshot = WaypointSnapshot(
        Pose.from_xyz_rpy(),
        Pose.from_xyz_rpy(),
        Pose.from_xyz_rpy(x=1.3),
        feedback_start=Pose.from_xyz_rpy(),
    )
    inputs.scene.blockers.return_value = []
    angles = (
        wall_scan.angle_min
        + np.arange(len(wall_scan.ranges)) * wall_scan.angle_increment
    )
    wall_scan.ranges = ((0.936 - 0.113) / np.cos(angles)).tolist()
    inputs.source.received_scan = wall_scan
    inputs.context.robot.root.global_transform = Pose.from_xyz_rpy(
        x=1.5
    ).homogeneous_matrix
    observation = inputs.observe(navigation_idle=True)
    assert observation.healthy is True
    assert observation.target_reached is True
    assert observation.fully_inside is True
    assert inputs.last_assessment.wall_distance == pytest.approx(
        settings.reference_distance, abs=1e-5
    )
    assert inputs.frozen_geometry.distance == pytest.approx(1.5, abs=1e-5)
