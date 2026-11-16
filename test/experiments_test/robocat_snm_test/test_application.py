"""
Exercise complete supervised requests with controlled action results.
"""

# %% imports
from concurrent.futures import Future
from contextlib import nullcontext
from dataclasses import replace
from unittest.mock import Mock

import pytest

from experiments.robocat_snm import application
from experiments.robocat_snm.application import (
    EntryEvents,
    EntryMission,
    EntryOptions,
    EntryStop,
)
from experiments.robocat_snm.exceptions import EntryError
from experiments.robocat_snm.geometry import EntryGeometry
from experiments.robocat_snm.native_navigation import NavigationStatus
from experiments.robocat_snm.policy import EntryObservation, EntryPhase, EntryPolicy
from semantic_digital_twin.spatial_types import Pose


# %% executable request fixtures
@pytest.fixture
def mission(monkeypatch: pytest.MonkeyPatch) -> EntryMission:
    """
    Provide a successful native arrival and an immediately completed owned motion.
    """
    navigation = Mock(status=NavigationStatus.SUCCEEDED, idle=True, routing_ready=True)
    navigation.poll.return_value = True
    inputs = Mock()
    inputs.feedback.ready.return_value = True
    inputs.feedback.wheels_settled.return_value = True
    inputs.snapshot.wheel_target = Pose.from_xyz_rpy(x=1.5)
    clear = EntryObservation(True, True, True, True, True, 1)
    inputs.observe.side_effect = [
        clear,
        replace(clear, fully_inside=True, target_reached=True),
    ]
    inputs.last_reasons = []
    complete = Future()
    complete.set_result(None)
    pool = Mock()
    pool.submit.return_value = complete
    monkeypatch.setattr(application, "real_robot", Mock(return_value=nullcontext()))
    monkeypatch.setattr(application, "execute_single", Mock())
    monkeypatch.setattr(application, "RobocatStraightMotion", Mock())
    return EntryMission(
        EntryOptions(map_fingerprint="checked-map"),
        inputs,
        navigation,
        EntryStop(),
        EntryEvents(Mock()),
        pool,
        policy=EntryPolicy(stability_seconds=0, minimum_scans=1),
    )


def test_successful_handover_submits_one_owned_motion(mission: EntryMission) -> None:
    """
    Verified native success permits exactly one entry motion.
    """
    geometry = EntryGeometry(Pose.from_xyz_rpy(), doorway_distance=0.7)
    assert mission.run(geometry) is EntryPhase.SUCCEEDED
    mission.inputs.freeze.assert_called_once_with(
        geometry, mission.options.map_fingerprint
    )
    mission.pool.submit.assert_called_once()


def test_failed_navigation_never_submits_giskard_goal(mission: EntryMission) -> None:
    """
    Native failure terminates before preparing a Giskard motion.
    """
    mission.navigation.status = NavigationStatus.FAILED
    with pytest.raises(EntryError):
        mission.run(EntryGeometry(Pose.from_xyz_rpy(), doorway_distance=0.7))
    mission.pool.submit.assert_not_called()


def test_cancel_before_navigation_sends_no_entry_goal(mission: EntryMission) -> None:
    """
    An interrupted approach never turns into an entry request.
    """
    mission.stop.requested = True
    with pytest.raises(EntryError):
        mission.run(EntryGeometry(Pose.from_xyz_rpy(), doorway_distance=0.7))
    mission.pool.submit.assert_not_called()


def test_stale_clearance_times_out_without_motion(mission: EntryMission) -> None:
    """
    Unavailable perception can expire the wait but cannot submit motion.
    """
    mission.options.wait_timeout = 0.01
    mission.inputs.observe.side_effect = None
    mission.inputs.observe.return_value = EntryObservation(
        True, True, True, None, None, 1
    )
    assert (
        mission.run(EntryGeometry(Pose.from_xyz_rpy(), doorway_distance=0.7))
        is EntryPhase.FAILED
    )
    mission.pool.submit.assert_not_called()


def test_execution_requires_calibration_and_door_hold_opt_in() -> None:
    """
    Missing physical-trial prerequisites are rejected before creating ROS nodes.
    """
    with pytest.raises(SystemExit):
        EntryOptions.parse(["--execute"])


def test_check_mode_never_opts_in_to_motion() -> None:
    """
    Default invocation performs read-only checks.
    """
    assert EntryOptions.parse([]).execute is False


def test_obstacle_cancels_once_and_never_resubmits(
    mission: EntryMission, monkeypatch: pytest.MonkeyPatch
) -> None:
    """
    A newly blocked corridor cancels the owned goal and ends the request.
    """
    clear = EntryObservation(True, True, True, True, True, 1)
    mission.inputs.observe.side_effect = [
        clear,
        replace(clear, passage_clear=False),
        clear,
    ]
    pending = Future()
    mission.pool.submit.return_value = pending
    cancellation = Mock()
    cancellation.cancel.side_effect = lambda: pending.set_exception(
        EntryError("Owned motion canceled")
    )
    monkeypatch.setattr(
        application, "GoalCancellation", Mock(return_value=cancellation)
    )
    assert (
        mission.run(EntryGeometry(Pose.from_xyz_rpy(), doorway_distance=0.7))
        is EntryPhase.FAILED
    )
    cancellation.cancel.assert_called_once_with()
    mission.pool.submit.assert_called_once()

def test_nav2_busy_cancels_giskard(
    mission: EntryMission, monkeypatch: pytest.MonkeyPatch
) -> None:
    """A new Nav2 goal cancels the owned Giskard motion."""
    clear = EntryObservation(True, True, True, True, True, 1)
    mission.inputs.observe.side_effect = [
        clear,
        replace(clear, navigation_idle=False),
        clear,
    ]

    pending = Future()
    mission.pool.submit.return_value = pending

    cancellation = Mock()
    cancellation.cancel.side_effect = lambda: pending.set_exception(
        EntryError("Owned motion canceled")
    )
    monkeypatch.setattr(
        application, "GoalCancellation", Mock(return_value=cancellation)
    )

    assert (
        mission.run(EntryGeometry(Pose.from_xyz_rpy(), doorway_distance=0.7))
        is EntryPhase.FAILED
    )
    cancellation.cancel.assert_called_once_with()
    mission.pool.submit.assert_called_once()