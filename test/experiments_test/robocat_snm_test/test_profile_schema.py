"""
Profiles cannot smuggle per-trial authorization through calibration files.
"""

# %% imports
from dataclasses import asdict
from pathlib import Path

import pytest
import yaml

from experiments.robocat_snm.exceptions import EntryError
from experiments.robocat_snm.profiles import EntryProfile, EntryProfileName


# %% strict schema
@pytest.mark.parametrize(
    "key", ["execute", "doors_held_open", "bootstrap_navigation", "unknown_setting"]
)
def test_profile_rejects_authorization_and_unknown_fields(
    key: str, monkeypatch: pytest.MonkeyPatch
) -> None:
    """
    Only stable calibration fields are accepted from the installed profile.
    """
    document = asdict(EntryProfile.load(EntryProfileName.ELEVATOR))
    document[key] = True
    monkeypatch.setattr(Path, "read_text", lambda self: yaml.safe_dump(document))
    with pytest.raises(EntryError):
        EntryProfile.load(EntryProfileName.ELEVATOR)


def test_profile_rejects_boolean_distance(monkeypatch: pytest.MonkeyPatch) -> None:
    """
    A boolean YAML value cannot be interpreted as a numeric distance.
    """
    document = asdict(EntryProfile.load(EntryProfileName.ELEVATOR))
    document["wall_stop_distance"] = True
    monkeypatch.setattr(Path, "read_text", lambda self: yaml.safe_dump(document))
    with pytest.raises(EntryError):
        EntryProfile.load(EntryProfileName.ELEVATOR)
