"""
Named elevator calibration without per-request execution authorization.
"""

# %% imports
from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum
from pathlib import Path

import yaml
from typing_extensions import get_type_hints

from experiments.robocat_snm.exceptions import EntryError


# %% named calibration
class EntryProfileName(StrEnum):
    """
    Installed calibrations selectable by the entry command.
    """

    ELEVATOR = "elevator"


@dataclass(frozen=True)
class EntryProfile:
    """
    Stable geometry and acceptance settings, excluding operator authorizations.
    """

    staging: str
    """
    Saved staging pose file relative to the repository.
    """

    inside_waypoint: str
    """
    Native Eulero waypoint for the nominal cabin destination.
    """

    feedback: str
    """
    Selected pose feedback source, validated by the entry argument parser.
    """
    wall_stop_distance: float
    """
    Calibrated base-to-back-wall stopping reference, in metres.
    """
    doorway_distance: float
    """
    Threshold position along the staging forward axis, in metres.
    """

    position_tolerance: float
    """
    Longitudinal acceptance around the wall stopping reference, in metres.
    """

    orientation_tolerance_deg: float
    """
    Nominal goal orientation tolerance, in degrees.
    """

    staging_position_tolerance: float
    """
    Maximum staging position error, in metres.
    """

    staging_orientation_tolerance_deg: float
    """
    Maximum staging heading error, in degrees.
    """
    retain_door_clearance: bool
    """
    Whether a stable doorway opening remains authorized during entry.
    """

    map_fingerprint: str
    """
    Expected occupancy-grid identity, checked before motion.
    """

    @classmethod
    def load(cls, name: EntryProfileName) -> EntryProfile:
        """
        Read a complete typed calibration and reject unknown or authorization fields.
        """
        path = (
            Path(__file__).resolve().parents[4]
            / "config"
            / "robocat_entry"
            / f"{name.value}.yaml"
        )
        if not path.is_file():
            raise EntryError(f"Entry profile not found: {path}")
        document = yaml.safe_load(path.read_text())
        schema = get_type_hints(cls)
        if not isinstance(document, dict) or set(document) != set(schema):
            raise EntryError(
                f"Entry profile {name.value} must contain exactly the calibration fields: {', '.join(schema)}"
            )
        for key, expected in schema.items():
            value = document[key]
            valid = (
                isinstance(value, (int, float)) and not isinstance(value, bool)
                if expected is float
                else type(value) is expected
            )
            if not valid:
                raise EntryError(f"Entry profile {name.value}: invalid type for {key}")
        return cls(**document)
