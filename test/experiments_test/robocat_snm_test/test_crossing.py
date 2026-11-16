"""
Distinguish an unseen door behind the chassis from a blocked forward path.
"""

# %% imports
from dataclasses import replace

from experiments.robocat_snm.policy import EntryObservation, EntryPhase, EntryPolicy


# %% threshold crossing
def test_door_visibility_may_end_during_held_open_crossing() -> None:
    """
    A clear remaining corridor permits completion when the doorway leaves view.
    """
    observation = EntryObservation(True, True, True, True, True, 1)
    policy = EntryPolicy(stability_seconds=0, minimum_scans=1)
    policy.update(observation, 0)
    policy.update(
        replace(observation, stationary=False, door_open=None, threshold_crossing=True),
        0.1,
    )
    assert policy.phase is EntryPhase.ENTERING


def test_measured_obstruction_at_threshold_still_stops() -> None:
    """
    Crossing commitment never overrides a measured doorway obstruction.
    """
    observation = EntryObservation(True, True, True, True, True, 1)
    policy = EntryPolicy(stability_seconds=0, minimum_scans=1)
    policy.update(observation, 0)
    policy.update(
        replace(
            observation, stationary=False, door_open=False, threshold_crossing=True
        ),
        0.1,
    )
    assert policy.phase is EntryPhase.STOPPING
