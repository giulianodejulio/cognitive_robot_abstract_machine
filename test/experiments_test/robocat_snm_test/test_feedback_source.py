"""
Consistent laser or wheel coordinates without remapping wheel safety checks.
"""

# %% imports
from unittest.mock import Mock

import pytest

from experiments.robocat_safety import FeedbackSource
from experiments.robocat_snm.application import EntryOptions
from experiments.robocat_snm.exceptions import EntryError
from experiments.robocat_snm.feedback import EntryInputs
from experiments.robocat_snm.odometry_history import OdometryHistory
from semantic_digital_twin.spatial_types import Pose

from .test_feedback import entry_inputs
from .test_scan_alignment import wheel_sample


# %% selected coordinates
def test_laser_observation_does_not_use_wheel_pose(entry_inputs: EntryInputs) -> None:
    """
    Independent wheel coordinates cannot change a laser-based final assessment.
    """
    entry_inputs.feedback_source = FeedbackSource.LASER
    entry_inputs.feedback.odometry = wheel_sample(10, x=0.1)
    entry_inputs.feedback.wheel_odometry = wheel_sample(10, x=5)
    entry_inputs.context.robot.root.global_transform = Pose.from_xyz_rpy(
        x=0.1
    ).homogeneous_matrix
    observation = entry_inputs.observe(navigation_idle=True)
    assert observation.healthy is True
    assert entry_inputs.last_assessment.position_error == pytest.approx(
        entry_inputs.frozen_geometry.distance - 0.1
    )
    assert entry_inputs.feedback.wheel_odometry.pose.pose.position.x == 5


def test_frozen_target_uses_laser_alignment_and_retains_real_wheels(
    entry_inputs: EntryInputs,
) -> None:
    """
    The target and staging use selected coordinates while the wheel snapshot stays real.
    """
    entry_inputs.feedback_source = FeedbackSource.LASER
    entry_inputs.feedback.odometry = wheel_sample(10, x=2)
    entry_inputs.feedback.wheel_odometry = wheel_sample(10, x=50)
    geometry = type(entry_inputs.frozen_geometry)(
        Pose.from_xyz_rpy(x=10), doorway_distance=0.7
    )
    entry_inputs.confirm_handover = Mock(return_value=Pose.from_xyz_rpy(x=10))
    entry_inputs.scene.blockers.return_value = []
    entry_inputs.freeze(geometry, entry_inputs.fingerprint)
    assert float(entry_inputs.snapshot.feedback_target.x) == pytest.approx(
        2 + geometry.distance
    )
    assert float(entry_inputs.snapshot.wheel_target.x) == pytest.approx(
        50 + geometry.distance
    )
    assert float(entry_inputs.frozen_geometry.staging.x) == pytest.approx(2)


def test_laser_scan_association_uses_laser_history(entry_inputs: EntryInputs) -> None:
    """
    Laser feedback is associated with acquisition time using its own history.
    """
    entry_inputs.feedback_source = FeedbackSource.LASER
    entry_inputs.node.get_clock.return_value.now.return_value.nanoseconds = (
        10_300_000_000
    )
    entry_inputs.feedback.odometry = wheel_sample(10.3)
    entry_inputs.feedback.laser_history = OdometryHistory()
    entry_inputs.feedback.laser_history.samples.extend(
        [wheel_sample(10), wheel_sample(10.3)]
    )
    assert entry_inputs.observe(navigation_idle=True).healthy is True


# %% explicit consistency check
def test_cli_selects_laser_and_preserves_wheel_default() -> None:
    """
    The default is backward compatible and laser requires an explicit option.
    """
    assert EntryOptions.parse([]).feedback is FeedbackSource.WHEEL
    assert EntryOptions.parse(["--feedback", "laser"]).feedback is FeedbackSource.LASER


def test_server_feedback_mismatch_is_rejected() -> None:
    """
    A laser request must not execute against a wheel-based Giskard server.
    """
    options = EntryOptions.parse(["--feedback", "laser"])
    options.verify_server_topics(str(FeedbackSource.LASER.topic), "/snm_vel")
    with pytest.raises(EntryError):
        options.verify_server_topics(str(FeedbackSource.WHEEL.topic), "/snm_vel")
