"""
Isolate retained doorway clearance from native plan and movement changes.
"""

# %% imports
from dataclasses import replace
from unittest.mock import Mock

import pytest

from experiments.robocat_snm import application
from experiments.robocat_snm.application import EntryMission, EntryOptions
from experiments.robocat_snm.geometry import EntryGeometry
from experiments.robocat_snm.policy import EntryObservation, EntryPhase, EntryPolicy
from semantic_digital_twin.spatial_types import Pose

from .test_application import mission


# %% authorization and baseline comparison
@pytest.mark.parametrize(
    "retain, expected", [(False, EntryPhase.STOPPING), (True, EntryPhase.ENTERING)]
)
def test_only_retained_mode_ignores_door_plane_after_authorization(
    retain: bool, expected: EntryPhase
) -> None:
    """
    Only the explicit comparison mode retains a previously clear doorway.
    """
    policy = EntryPolicy(
        stability_seconds=0, minimum_scans=1, retain_door_clearance=retain
    )
    clear = EntryObservation(True, True, True, True, True, 1)
    policy.update(replace(clear, door_open=False), 0)
    assert policy.phase is EntryPhase.WAITING
    policy.update(clear, 1)
    assert policy.phase is EntryPhase.ENTERING
    policy.update(replace(clear, stationary=False, door_open=False), 2)
    assert policy.phase is expected


@pytest.mark.parametrize(
    "change",
    [
        dict(healthy=False),
        dict(navigation_idle=False),
        dict(passage_clear=False),
        dict(passage_clear=None),
        dict(motion_failed=True),
    ],
)
def test_retained_opening_never_overrides_continuous_protection(
    change: dict[str, bool | None],
) -> None:
    """
    Retaining the doorway observation does not retain path or safety evidence.
    """
    policy = EntryPolicy(
        stability_seconds=0, minimum_scans=1, retain_door_clearance=True
    )
    clear = EntryObservation(True, True, True, True, True, 1)
    policy.update(clear, 0)
    policy.update(replace(clear, **change), 1)
    assert policy.phase is EntryPhase.STOPPING


def test_retained_opening_does_not_relax_final_acceptance() -> None:
    """
    An incomplete crossing remains a failure even after Giskard success.
    """
    policy = EntryPolicy(
        stability_seconds=0, minimum_scans=1, retain_door_clearance=True
    )
    clear = EntryObservation(True, True, True, True, True, 1)
    policy.update(clear, 0)
    policy.update(
        replace(
            clear,
            motion_succeeded=True,
            motion_terminal=True,
            fully_inside=False,
            target_reached=True,
        ),
        1,
    )
    assert policy.phase is EntryPhase.STOPPING


# %% CLI and unchanged motion
def test_retained_mode_is_explicit_and_separate_from_native_action() -> None:
    """
    The comparison changes no default and cannot combine both experiments.
    """
    assert EntryOptions.parse([]).retain_door_clearance is False
    assert EntryOptions.parse(["--retain-door-clearance"]).retain_door_clearance is True
    with pytest.raises(SystemExit):
        EntryOptions.parse(["--retain-door-clearance", "--pass-door-action"])


def test_retained_mode_keeps_the_single_baseline_motion(
    mission: EntryMission, monkeypatch: pytest.MonkeyPatch
) -> None:
    """
    The comparison performs no native staging correction or extra navigation.
    """
    mission.options.retain_door_clearance = True
    clear = EntryObservation(True, True, True, True, True, 1)
    mission.inputs.observe.side_effect = [
        clear,
        replace(clear, door_open=False, fully_inside=True, target_reached=True),
    ]
    factory = Mock()
    monkeypatch.setattr(application.CalibratedDoorPassage, "from_geometry", factory)
    geometry = EntryGeometry(Pose.from_xyz_rpy(), doorway_distance=0.86, distance=1.3)
    assert mission.run(geometry) is EntryPhase.SUCCEEDED
    application.RobocatStraightMotion.assert_called_once()
    factory.assert_not_called()
    mission.pool.submit.assert_called_once()
