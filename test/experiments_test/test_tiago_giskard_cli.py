# %% imports
from __future__ import annotations

import runpy
from pathlib import Path

import pytest


# %% standalone client arguments
@pytest.fixture(scope="module")
def client_script() -> dict:
    """
    Load the goal client without starting ROS or sending a goal.
    """
    return runpy.run_path(
        str(Path(__file__).parents[2] / "scripts/tiago_giskard_goal.py")
    )


def test_default_distance(client_script: dict) -> None:
    """
    Keep the established forward distance by default.
    """
    assert client_script["GoalOptions"].parse(["--execute"]).distance == 0.3


def test_requested_distance(client_script: dict) -> None:
    """
    Allow distance selection independently of the running server.
    """
    assert (
        client_script["GoalOptions"].parse(["--execute", "--distance", "0.8"]).distance
        == 0.8
    )


@pytest.mark.parametrize("distance", ["0", "-1", "nan", "inf", "invalid"])
def test_reject_invalid_distance(client_script: dict, distance: str) -> None:
    """
    Reject invalid motion requests before ROS initialization.
    """
    with pytest.raises(SystemExit) as error:
        client_script["GoalOptions"].parse(["--execute", "--distance", distance])
    assert error.value.code == 2


def test_require_execution_opt_in(client_script: dict) -> None:
    """
    Require explicit opt-in before sending a real motion goal.
    """
    with pytest.raises(SystemExit) as error:
        client_script["GoalOptions"].parse([])
    assert error.value.code == 2
