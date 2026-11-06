"""
Check the dedicated Giskard command path without connecting to a robot.
"""

# %% imports
from pathlib import Path

import yaml

from experiments.robocat_bridge import RobocatBridgeConfiguration


# %% command routing
def test_separate_command_route_does_not_forward_native_navigation() -> None:
    """
    Keep native navigation commands outside the laptop command route.
    """
    source = Path(__file__).parents[2] / "doc/robocat_dds_router.yaml"
    configuration = RobocatBridgeConfiguration(
        source, command_topic="/snm_vel"
    ).for_control()
    robot, laptop = configuration["participants"]
    assert configuration["topic-routes"] == [
        {
            "name": "rt/snm_vel",
            "type": "geometry_msgs::msg::dds_::Twist_",
            "routes": [
                {"src": robot["name"]},
                {"src": laptop["name"], "dst": [robot["name"]]},
            ],
        }
    ]
    assert {"name": "rt/cmd_vel"} not in configuration["allowlist"]
    command = next(
        topic for topic in configuration["topics"] if topic["name"] == "rt/snm_vel"
    )
    assert command["qos"] == {
        "reliability": True,
        "durability": False,
        "history-depth": 1,
        "endpoint-profile-name": "robocat_base_command",
    }


def test_mux_override_adds_only_the_giskard_input() -> None:
    """
    Leave existing PAL inputs and safety locks to the original configuration.
    """
    source = Path(__file__).parents[2] / "doc/robocat_mux_snm_override.yaml"
    configuration = yaml.safe_load(source.read_text())
    assert configuration == {
        "/twist_mux": {
            "ros__parameters": {
                "topics": {"snm": {"topic": "snm_vel", "timeout": 0.5, "priority": 20}}
            }
        }
    }
