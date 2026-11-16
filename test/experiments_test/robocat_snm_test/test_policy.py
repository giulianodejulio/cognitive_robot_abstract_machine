"""
Verify entry decisions without connecting to a robot.
"""

# %% imports
from dataclasses import replace

import pytest

from experiments.robocat_snm.policy import EntryObservation, EntryPhase, EntryPolicy


# %% observations
@pytest.fixture
def clear_entry() -> EntryObservation:
    """
    Provide a stationary robot with a measured clear entry corridor.
    """
    return EntryObservation(
        healthy=True,
        stationary=True,
        navigation_idle=True,
        door_open=True,
        passage_clear=True,
        sample=1,
    )


def test_entry_requires_stable_distinct_scans(clear_entry: EntryObservation) -> None:
    """
    Repeated evaluations of one scan cannot authorize entry.
    """
    policy = EntryPolicy()
    policy.update(clear_entry, now=0)
    policy.update(clear_entry, now=2)
    assert policy.phase is EntryPhase.WAITING
    policy.update(replace(clear_entry, sample=2), now=2.1)
    policy.update(replace(clear_entry, sample=3), now=2.2)
    assert policy.phase is EntryPhase.ENTERING


@pytest.mark.parametrize("door", [False, None])
def test_closed_or_unknown_door_resets_clear_interval(
    clear_entry: EntryObservation, door: bool | None
) -> None:
    """
    An unavailable door observation does not count as an open doorway.
    """
    policy = EntryPolicy()
    policy.update(clear_entry, now=0)
    policy.update(replace(clear_entry, sample=2, door_open=door), now=0.5)
    policy.update(replace(clear_entry, sample=3), now=2)
    assert policy.phase is EntryPhase.WAITING


@pytest.mark.parametrize("attribute", ["stationary", "navigation_idle", "healthy"])
def test_handover_requires_every_interlock(
    clear_entry: EntryObservation, attribute: str
) -> None:
    """
    An open door cannot override a moving or unavailable base.
    """
    policy = EntryPolicy(stability_seconds=0, minimum_scans=1)
    policy.update(replace(clear_entry, **{attribute: False}), now=0)
    assert policy.phase is EntryPhase.WAITING


def test_obstacle_during_entry_requests_cancel(clear_entry: EntryObservation) -> None:
    """
    An obstruction in the swept path cancels the owned motion.
    """
    policy = EntryPolicy(stability_seconds=0, minimum_scans=1)
    policy.update(clear_entry, now=0)
    policy.update(replace(clear_entry, passage_clear=False), now=0.1)
    assert policy.phase is EntryPhase.STOPPING

def test_nav2_becomes_busy_during_entry_requests_stop(
    clear_entry: EntryObservation,
) -> None:
    policy = EntryPolicy(stability_seconds=0, minimum_scans=1)

    policy.update(clear_entry, now=0)
    assert policy.phase is EntryPhase.ENTERING

    policy.update(
        replace(clear_entry, navigation_idle=False),
        now=0.1,
    )

    assert policy.phase is EntryPhase.STOPPING

def test_door_out_of_view_inside_does_not_cause_ordinary_stop(
    clear_entry: EntryObservation,
) -> None:
    """
    A door behind the robot is not a new obstruction in the remaining path.
    """
    policy = EntryPolicy(stability_seconds=0, minimum_scans=1)
    policy.update(clear_entry, now=0)
    policy.update(replace(clear_entry, fully_inside=True, door_open=None), now=0.1)
    assert policy.phase is EntryPhase.ENTERING


def test_success_requires_result_pose_and_settled_wheels(
    clear_entry: EntryObservation,
) -> None:
    """
    Action success alone cannot certify a completed physical entry.
    """
    policy = EntryPolicy(stability_seconds=0, minimum_scans=1)
    policy.update(clear_entry, now=0)
    policy.update(replace(clear_entry, motion_succeeded=True), now=0.1)
    assert policy.phase is EntryPhase.STOPPING


def test_verified_arrival_succeeds(clear_entry: EntryObservation) -> None:
    """
    Completion needs the robot fully inside and near the configured target.
    """
    policy = EntryPolicy(stability_seconds=0, minimum_scans=1)
    policy.update(clear_entry, now=0)
    policy.update(
        replace(
            clear_entry, motion_succeeded=True, fully_inside=True, target_reached=True
        ),
        now=0.1,
    )
    assert policy.phase is EntryPhase.SUCCEEDED


def test_cancel_waits_for_terminal_action_and_arrest(
    clear_entry: EntryObservation,
) -> None:
    """
    Cancellation stays pending until both action and base have stopped.
    """
    policy = EntryPolicy(stability_seconds=0, minimum_scans=1)
    policy.update(clear_entry, now=0)
    policy.cancel()
    policy.update(replace(clear_entry, stationary=False, motion_terminal=True), now=0.1)
    assert policy.phase is EntryPhase.STOPPING
    policy.update(replace(clear_entry, motion_terminal=True), now=0.2)
    assert policy.phase is EntryPhase.CANCELED


def test_canceled_entry_never_restarts(clear_entry: EntryObservation) -> None:
    """
    Fresh clear scans do not resume an interrupted crossing.
    """
    policy = EntryPolicy()
    policy.cancel()
    policy.update(replace(clear_entry, motion_terminal=True), now=0)
    policy.update(clear_entry, now=5)
    assert policy.phase is EntryPhase.CANCELED
