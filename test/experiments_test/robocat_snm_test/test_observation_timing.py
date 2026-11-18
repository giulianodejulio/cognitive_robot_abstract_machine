"""
Long evaluations cannot mix an old clock reading with new callback samples.
"""

# %% imports
from copy import deepcopy
from unittest.mock import Mock

import numpy as np
import pytest
from sensor_msgs.msg import LaserScan

from experiments.robocat_snm.feedback import EntryInputs
from experiments.robocat_snm.geometry import EntryGeometry
from experiments.robocat_snm.odometry_history import OdometryHistory
from experiments.robocat_wall import WallStopController, WallStopSettings
from experiments.robocat_waypoints.model import WaypointSnapshot
from experiments.tiago_goal_logging import PlanarPose
from semantic_digital_twin.spatial_types import Pose

from .test_feedback import entry_inputs
from .test_wall_stop import wall_scan, laser_transform


# %% delayed observation fixture
@pytest.fixture
def wall_inputs(
    entry_inputs: EntryInputs, wall_scan: LaserScan, laser_transform: np.ndarray
) -> EntryInputs:
    """
    Provide a stationary cabin reference with production wall validation.
    """
    inputs = entry_inputs
    inputs.wall_settings = WallStopSettings(0.936)
    inputs.wall_controller = WallStopController(
        inputs.wall_settings, PlanarPose(1.3, 0, 0), laser_transform
    )
    inputs.wall_controller.observe = Mock(wraps=inputs.wall_controller.observe)
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
    wall_scan.ranges = (
        (inputs.wall_settings.reference_distance - 0.113) / np.cos(angles)
    ).tolist()
    inputs.source.received_scan = wall_scan
    inputs.context.robot.root.global_transform = Pose.from_xyz_rpy(
        x=1.5
    ).homogeneous_matrix
    return inputs


@pytest.fixture
def delayed_assessment(
    wall_inputs: EntryInputs, monkeypatch: pytest.MonkeyPatch
) -> Mock:
    """
    Advance the clock and publish fresh sensors during the first geometric assessment.
    """
    original = EntryGeometry.assess
    updates = Mock()

    def assess(geometry: EntryGeometry, pose: Pose):
        if updates.call_count == 0:
            updates()
            wall_inputs.node.get_clock.return_value.now.return_value.nanoseconds = (
                10_637_000_000
            )
            wheel = deepcopy(wall_inputs.feedback.wheel_odometry)
            wheel.header.stamp.nanosec = 637_000_000
            scan = deepcopy(wall_inputs.source.received_scan)
            scan.header.stamp.nanosec = 500_000_000
            acquisition = deepcopy(wheel)
            acquisition.header.stamp.nanosec = 500_000_000
            wall_inputs.feedback.wheel_history.samples.extend([acquisition, wheel])
            wall_inputs.feedback.wheel_odometry = wheel
            wall_inputs.source.received_scan = scan
        return original(geometry, pose)

    monkeypatch.setattr(EntryGeometry, "assess", assess)
    return updates


# %% callback race regression
def test_delayed_cycle_refreshes_evidence_instead_of_false_future_fault(
    wall_inputs: EntryInputs, delayed_assessment: Mock
) -> None:
    """
    Fresh callbacks after a 637 ms pause permit a newly evaluated valid observation.
    """
    observation = wall_inputs.observe(navigation_idle=True)
    assert observation.healthy is True
    assert observation.target_reached is True
    assert wall_inputs.last_reasons == []
    assert wall_inputs.trace.scan_stamp == 10.5
    assert wall_inputs.trace.wheel_stamp == 10.637


def test_wall_guard_uses_same_wheel_snapshot_as_pose_comparison(
    wall_inputs: EntryInputs, delayed_assessment: Mock
) -> None:
    """
    The wall guard never reads a later callback against the old observation time.
    """
    captured = wall_inputs.feedback.wheel_odometry
    wall_inputs.observe(navigation_idle=True)
    first = wall_inputs.wall_controller.observe.call_args_list[0]
    assert first.args[1] is captured
    assert first.args[3] == 10.0


def test_genuinely_stale_sources_remain_blocked(wall_inputs: EntryInputs) -> None:
    """
    Retrying evaluation cannot make an unchanged old scan or wheel sample fresh.
    """
    wall_inputs.node.get_clock.return_value.now.return_value.nanoseconds = (
        11_000_000_000
    )
    observation = wall_inputs.observe(navigation_idle=True)
    assert observation.healthy is False
    assert observation.passage_clear is None
    assert wall_inputs.maximum_age == 0.5
    assert wall_inputs.wall_settings.maximum_age == 0.5


def test_repeated_processing_stalls_cannot_authorize_stale_observation(
    wall_inputs: EntryInputs, monkeypatch: pytest.MonkeyPatch
) -> None:
    """
    Fresh callbacks do not authorize a decision whose own computation keeps expiring.
    """
    original = EntryInputs.assess_feedback
    completed = Mock()

    def assess(inputs: EntryInputs, navigation_idle: bool):
        observation = original(inputs, navigation_idle)
        completed()
        nanoseconds = (
            inputs.node.get_clock.return_value.now.return_value.nanoseconds
            + 637_000_000
        )
        inputs.node.get_clock.return_value.now.return_value.nanoseconds = nanoseconds
        wheel = deepcopy(inputs.feedback.wheel_odometry)
        wheel.header.stamp.sec, wheel.header.stamp.nanosec = divmod(
            nanoseconds, 1_000_000_000
        )
        scan = deepcopy(inputs.source.received_scan)
        scan.header.stamp = deepcopy(wheel.header.stamp)
        inputs.feedback.wheel_history.samples.append(wheel)
        inputs.feedback.wheel_odometry = wheel
        inputs.source.received_scan = scan
        return observation

    monkeypatch.setattr(EntryInputs, "assess_feedback", assess)
    observation = wall_inputs.observe(navigation_idle=True)
    assert observation.healthy is False
    assert observation.target_reached is False
    assert observation.passage_clear is None
    assert completed.call_count == wall_inputs.maximum_observation_attempts
    assert wall_inputs.last_assessment is None
    assert wall_inputs.trace.attempts == wall_inputs.maximum_observation_attempts


def test_genuinely_future_odometry_remains_blocked(wall_inputs: EntryInputs) -> None:
    """
    The consistent snapshot retains rejection of real clock/source disagreement.
    """
    wall_inputs.feedback.wheel_odometry.header.stamp.sec = 11
    observation = wall_inputs.observe(navigation_idle=True)
    assert observation.healthy is False
    assert observation.target_reached is False
