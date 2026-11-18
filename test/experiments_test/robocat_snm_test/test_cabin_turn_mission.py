"""
Post-entry orchestration never turns on a merely reported or incomplete arrival.
"""

# %% imports
from concurrent.futures import Future
from dataclasses import replace
from unittest.mock import Mock
import math

import pytest
from nav_msgs.msg import Odometry

from experiments.robocat_snm import application
from experiments.robocat_snm.application import EntryMission
from experiments.robocat_snm.geometry import EntryGeometry
from experiments.robocat_snm.policy import EntryObservation, EntryPhase
from semantic_digital_twin.spatial_types import Pose
from semantic_digital_twin.world import World

from .test_application import mission


# %% certified stopped entry fixture
@pytest.fixture
def arrived_mission(mission: EntryMission) -> EntryMission:
    """
    An owned successful entry is followed by independently verified wall acceptance.
    """
    mission.options.wall_stop_distance = 0.936
    mission.options.position_tolerance = 0.05
    mission.options.orientation_tolerance_deg = 5.0
    mission.options.turn_after_entry = True
    mission.policy.phase = EntryPhase.SUCCEEDED
    mission.motion = Future()
    mission.motion.set_result(None)
    mission.inputs.context.world = World.create_with_root_body("odom")
    mission.inputs.node.get_clock.return_value.now.return_value.nanoseconds = (
        10_000_000_000
    )
    mission.inputs.scene.fingerprint = mission.options.map_fingerprint
    mission.inputs.feedback.blocking_reasons.return_value = []
    mission.inputs.observe.side_effect = None
    mission.inputs.observe.return_value = EntryObservation(
        True, True, True, True, True, 1, fully_inside=True, target_reached=True
    )
    geometry = EntryGeometry(Pose.from_xyz_rpy(), doorway_distance=0.86, distance=1.3)
    mission.inputs.frozen_geometry = geometry
    mission.inputs.last_assessment = replace(
        geometry.assess(Pose.from_xyz_rpy(x=1.3)),
        wall_distance=0.936,
        wall_reference=0.936,
        wall_tolerance=0.05,
    )
    wheel = Odometry()
    wheel.header.stamp.sec = 10
    wheel.pose.pose.position.x = 1.5
    wheel.pose.pose.orientation.w = 1.0
    mission.inputs.pose_feedback = wheel
    return mission


def test_verified_arrival_submits_one_pure_turn(arrived_mission: EntryMission) -> None:
    """
    The next action keeps the current position and points opposite the entry heading.
    """

    def finish_turn(operation):
        arrived_mission.inputs.pose_feedback.pose.pose.orientation.z = 1.0
        arrived_mission.inputs.pose_feedback.pose.pose.orientation.w = 0.0
        result = Future()
        result.set_result(None)
        return result

    arrived_mission.pool.submit.side_effect = finish_turn
    assert arrived_mission.turn_inside() is EntryPhase.SUCCEEDED
    arrived_mission.pool.submit.assert_called_once()
    motion = application.execute_single.call_args.args[0]
    assert float(motion.target.x) == 1.5
    assert abs(float(motion.target.yaw)) == pytest.approx(math.pi)


@pytest.mark.parametrize(
    "condition", ["short", "failed_entry", "moving", "stale", "canceled"]
)
def test_failed_arrival_gate_never_submits_turn(
    arrived_mission: EntryMission, condition: str
) -> None:
    """
    Missing longitudinal evidence or rest cannot be bypassed by a native success.
    """
    if condition == "short":
        arrived_mission.inputs.last_assessment = replace(
            arrived_mission.inputs.last_assessment, wall_distance=1.12
        )
    elif condition == "failed_entry":
        arrived_mission.policy.phase = EntryPhase.FAILED
    elif condition == "moving":
        arrived_mission.inputs.observe.return_value = replace(
            arrived_mission.inputs.observe.return_value, stationary=False
        )
    elif condition == "stale":
        arrived_mission.inputs.observe.return_value = replace(
            arrived_mission.inputs.observe.return_value, healthy=False
        )
    else:
        arrived_mission.stop.requested = True
    expected = EntryPhase.CANCELED if condition == "canceled" else EntryPhase.FAILED
    assert arrived_mission.turn_inside() is expected
    arrived_mission.pool.submit.assert_not_called()


def test_driver_interlock_cancels_turn_without_resubmission(
    arrived_mission: EntryMission, monkeypatch: pytest.MonkeyPatch
) -> None:
    """
    A driver fault while turning cancels the owned goal and never starts another.
    """
    pending = Future()
    arrived_mission.pool.submit.return_value = pending
    arrived_mission.inputs.feedback.blocking_reasons.return_value = [
        "wheel driver error"
    ]
    cancellation = Mock()
    cancellation.cancel.side_effect = lambda: pending.set_exception(
        RuntimeError("canceled")
    )
    monkeypatch.setattr(
        application, "GoalCancellation", Mock(return_value=cancellation)
    )
    assert arrived_mission.turn_inside() is EntryPhase.FAILED
    arrived_mission.pool.submit.assert_called_once()
    cancellation.cancel.assert_called_once()


# %% complete request sequencing
def test_entry_run_invokes_turn_only_after_success(mission: EntryMission) -> None:
    """
    The optional second phase follows entry completion rather than its submission.
    """
    mission.options.turn_after_entry = True
    mission.options.wall_stop_distance = 0.936
    mission.turn_inside = Mock(return_value=EntryPhase.SUCCEEDED)
    assert (
        mission.run(EntryGeometry(Pose.from_xyz_rpy(), doorway_distance=0.7))
        is EntryPhase.SUCCEEDED
    )
    mission.turn_inside.assert_called_once_with()
    assert mission.policy.phase is EntryPhase.SUCCEEDED


def test_failed_entry_run_never_invokes_turn(mission: EntryMission) -> None:
    """
    An interrupted crossing cannot reach the optional orientation phase.
    """
    mission.options.turn_after_entry = True
    mission.options.wall_stop_distance = 0.936
    mission.turn_inside = Mock(return_value=EntryPhase.SUCCEEDED)
    clear = EntryObservation(True, True, True, True, True, 1)
    mission.inputs.observe.side_effect = [clear, replace(clear, healthy=False), clear]
    assert (
        mission.run(EntryGeometry(Pose.from_xyz_rpy(), doorway_distance=0.7))
        is EntryPhase.FAILED
    )
    mission.turn_inside.assert_not_called()


# %% relative heading independent of map orientation
@pytest.mark.parametrize("initial_yaw", [2.1426351199754463, -2.8, 0.7])
def test_turn_target_is_half_turn_from_measured_heading(
    arrived_mission: EntryMission, initial_yaw: float
) -> None:
    """
    A stopped measured heading, rather than the nominal waypoint, anchors the turn.
    """
    wheel = arrived_mission.inputs.pose_feedback
    wheel.pose.pose.orientation.z = math.sin(initial_yaw / 2)
    wheel.pose.pose.orientation.w = math.cos(initial_yaw / 2)

    def finish_turn(operation):
        wheel.pose.pose.orientation.z = math.sin((initial_yaw + math.pi) / 2)
        wheel.pose.pose.orientation.w = math.cos((initial_yaw + math.pi) / 2)
        result = Future()
        result.set_result(None)
        return result

    arrived_mission.pool.submit.side_effect = finish_turn
    assert arrived_mission.turn_inside() is EntryPhase.SUCCEEDED
    motion = application.execute_single.call_args.args[0]
    actual = float(motion.target.yaw)
    assert abs(
        math.atan2(math.sin(actual - initial_yaw), math.cos(actual - initial_yaw))
    ) == pytest.approx(math.pi)
