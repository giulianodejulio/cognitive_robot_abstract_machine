# %% imports
from __future__ import annotations

import subprocess
from pathlib import Path

import pytest

from experiments.robocat_safety import RobocatEnvironment


# %% prevent accidental Jazzy participation in the robot domain
def test_native_humble_domain_is_rejected(monkeypatch: pytest.MonkeyPatch) -> None:
    """
    Reject the native robot domain before creating a Jazzy node.
    """
    monkeypatch.setenv("ROS_DOMAIN_ID", "102")
    with pytest.raises(SystemExit):
        RobocatEnvironment().validate()


def test_isolated_domain_is_accepted(monkeypatch: pytest.MonkeyPatch) -> None:
    """
    Allow the dedicated local Jazzy domain.
    """
    monkeypatch.setenv("ROS_DOMAIN_ID", "103")
    assert RobocatEnvironment().validate() is None


def test_bringup_launcher_help_starts_no_process() -> None:
    """
    Inspect startup options without requiring ROS, Docker or hardware.
    """
    script = Path(__file__).parents[2] / "scripts/launch_robocat_giskard.sh"
    result = subprocess.run(
        ["bash", str(script), "--help"], capture_output=True, text=True, timeout=5
    )
    assert result.returncode == 0
    assert "--max-linear" in result.stdout
    assert "--max-angular" in result.stdout
