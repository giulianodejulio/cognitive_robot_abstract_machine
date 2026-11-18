"""
Named elevator calibration retains explicit per-trial authorizations.
"""

# %% imports
from dataclasses import asdict
from pathlib import Path

import pytest
import yaml

from experiments.robocat_snm.application import EntryOptions


# %% profile behavior
def test_elevator_profile_reproduces_validated_configuration() -> None:
    """
    The compact request retains the calibration of the successful entry trials.
    """
    compact = EntryOptions.parse(
        [
            "--profile",
            "elevator",
            "--execute",
            "--doors-held-open",
            "--bootstrap-navigation",
        ]
    )
    explicit = EntryOptions.parse(
        [
            "--execute",
            "--doors-held-open",
            "--bootstrap-navigation",
            "--feedback",
            "wheel",
            "--retain-door-clearance",
            "--inside-waypoint",
            "elevator_inside",
            "--wall-stop-distance",
            "0.936",
            "--doorway-distance",
            "0.86",
            "--position-tolerance",
            "0.05",
            "--staging-position-tolerance",
            "0.10",
            "--staging-orientation-tolerance-deg",
            "10",
            "--map-fingerprint",
            "14cae3e62b38495028d71f384a7bea50e41025b26b866dfbcf6107edb8961fda",
        ]
    )
    compact_values, explicit_values = asdict(compact), asdict(explicit)
    compact_values.pop("profile")
    explicit_values.pop("profile")
    assert compact_values == explicit_values


def test_profile_does_not_authorize_motion_or_door_hold() -> None:
    """
    Reading calibration alone cannot start navigation or confirm physical conditions.
    """
    options = EntryOptions.parse(["--profile", "elevator"])
    assert options.execute is False
    assert options.doors_held_open is False
    assert options.bootstrap_navigation is False
    assert options.verbose is False


def test_profile_still_requires_door_hold_confirmation() -> None:
    """
    A stored calibration cannot replace the explicit door-hold authorization.
    """
    with pytest.raises(SystemExit):
        EntryOptions.parse(["--profile", "elevator", "--execute"])


@pytest.mark.parametrize(
    "arguments",
    [["--profile", "absent"], ["--profile", "elevator", "--feedback", "laser"]],
)
def test_invalid_profile_or_unsupported_override_is_rejected(
    arguments: list[str],
) -> None:
    """
    Profiles retain name and feedback compatibility validation.
    """
    with pytest.raises(SystemExit):
        EntryOptions.parse(arguments)


def test_explicit_arguments_override_profile_in_either_order() -> None:
    """
    Trial-specific settings take precedence over the stored calibration.
    """
    for arguments in (
        ["--position-tolerance", "0.06", "--profile", "elevator"],
        ["--profile", "elevator", "--position-tolerance", "0.06"],
    ):
        assert EntryOptions.parse(arguments).position_tolerance == 0.06


def test_explicit_distance_replaces_profile_waypoint() -> None:
    """
    A mutually exclusive target override does not leave a hidden inside waypoint.
    """
    options = EntryOptions.parse(["--profile", "elevator", "--distance", "1.5"])
    assert options.inside_waypoint is None
    assert options.distance == 1.5
