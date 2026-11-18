"""
Continuity diagnostics preserve rejection and the previously accepted target.
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
from .test_wall_continuity import wheel_sample
from .test_application import mission
from .test_feedback import entry_inputs
from .test_scan_alignment import (
    aligned_inputs,
    wheel_sample as timestamped_wheel_sample,
)
from experiments.robocat_snm.application import EntryMission
from experiments.robocat_snm.verbose import EntryTrace
from experiments.robocat_snm.geometry import EntryGeometry
from experiments.robocat_snm.policy import EntryPhase
from semantic_digital_twin.spatial_types import Pose


# %% recorded rejection evidence
def test_wall_rejection_exposes_candidate_jump(
    wall_scan: LaserScan, laser_transform: np.ndarray, wheel_sample: Odometry
) -> None:
    """
    A rejected new plane reports its displacement without moving the endpoint.
    """
    controller = WallStopController(
        WallStopSettings(0.936), PlanarPose(0.18, 0, 0), laser_transform
    )
    history = OdometryHistory()
    history.samples.append(wheel_sample)
    first = controller.observe(wall_scan, wheel_sample, history, 10.0)
    wall_scan.header.stamp.nanosec = 100_000_000
    wheel_sample.header.stamp.nanosec = 100_000_000
    wall_scan.ranges = (np.asarray(wall_scan.ranges) * 1.12).tolist()
    rejected = controller.observe(wall_scan, wheel_sample, history, 10.1)
    assert rejected.valid is False
    assert rejected.distance_jump == pytest.approx(0.12, abs=0.001)
    assert rejected.normal_change == pytest.approx(0.0, abs=0.001)
    assert controller.progress == first.target_progress


# %% mission wiring
def test_verbose_mission_preserves_success_and_motion_count(
    mission: EntryMission, capsys: pytest.CaptureFixture
) -> None:
    """
    Enabling terminal tracing retains the normal single-motion success path.
    """
    mission.options.verbose = True
    mission.inputs.trace = EntryTrace()
    assert (
        mission.run(EntryGeometry(Pose.from_xyz_rpy(), doorway_distance=0.7))
        is EntryPhase.SUCCEEDED
    )
    mission.pool.submit.assert_called_once()
    assert "VERBOSE TIMING:" in capsys.readouterr().out


# %% observation provenance
def test_verbose_evidence_uses_acquisition_match(aligned_inputs) -> None:
    """
    Trace ages and skew retain the samples used by the actual clearance guard.
    """
    aligned_inputs.node.get_clock.return_value.now.return_value.nanoseconds = (
        10_300_000_000
    )
    aligned_inputs.feedback.receive_wheels(timestamped_wheel_sample(10.0))
    aligned_inputs.feedback.receive_wheels(timestamped_wheel_sample(10.3))
    observation = aligned_inputs.observe(navigation_idle=True)
    assert observation.healthy is True
    assert aligned_inputs.trace.scan_stamp == 10.0
    assert aligned_inputs.trace.wheel_stamp == 10.3
    assert aligned_inputs.trace.association_stamp == 10.0
    assert len(aligned_inputs.trace.sections) > 0
    assert all(section.clear is True for section in aligned_inputs.trace.sections)
