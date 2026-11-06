# %% imports
from __future__ import annotations

import subprocess
from pathlib import Path

import pytest
import yaml


# %% feedback bridge configuration
@pytest.fixture
def bridge_configuration() -> dict:
    """
    Read the feedback bridge configuration shipped with the workspace.
    """
    root = Path(__file__).parents[2]
    return yaml.safe_load((root / "doc/robocat_dds_router.yaml").read_text())


def test_feedback_bridge_has_no_route_to_robot(bridge_configuration: dict) -> None:
    """
    Prevent forwarding laptop messages into the robot domain.
    """
    participants = bridge_configuration["participants"]
    robot, laptop = participants
    routes = {
        route["src"]: route.get("dst", [])
        for route in bridge_configuration.get("routes", [])
    }
    assert routes == {robot["name"]: [laptop["name"]], laptop["name"]: []}


def test_feedback_bridge_excludes_graph_services_and_commands(
    bridge_configuration: dict,
) -> None:
    """
    Limit forwarding to explicitly named feedback topics.
    """
    allowed = {topic["name"] for topic in bridge_configuration["allowlist"]}
    assert all(name.startswith("rt/") for name in allowed)
    assert all("*" not in name and "?" not in name for name in allowed)
    assert allowed.isdisjoint(
        {"ros_discovery_info", "rt/cmd_vel", "rt/mobile_base_controller/cmd_vel"}
    )


def test_feedback_bridge_bounds_history(bridge_configuration: dict) -> None:
    """
    Bound retained feedback to avoid an unlimited router history.
    """
    assert 0 < bridge_configuration["specs"]["qos"]["history-depth"] <= 100


# %% launcher usage
def test_router_launcher_help_does_not_need_docker() -> None:
    """
    Show usage without building or starting a network participant.
    """
    script = Path(__file__).parents[2] / "scripts/launch_robocat_dds_router.sh"
    result = subprocess.run(
        ["bash", str(script), "--help"], capture_output=True, text=True, timeout=5
    )
    assert result.returncode == 0
    assert "feedback" in result.stdout
