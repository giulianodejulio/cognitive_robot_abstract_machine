# %% imports
from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path

import yaml
from typing_extensions import Any


# %% control bridge configuration
@dataclass
class RobocatBridgeConfiguration:
    """
    Extend the feedback bridge with one explicit base-command route.
    """

    source: Path
    """The validated feedback-only YAML configuration."""

    command_topic: str = "/cmd_vel"
    """
    The absolute ROS topic carrying commands toward the robot mux.
    """

    def for_control(self) -> dict[str, Any]:
        """
        Route only volatile Twist commands from Jazzy to the native robot mux.
        """
        configuration = yaml.safe_load(self.source.read_text())
        robot, laptop = configuration["participants"]
        command = f"rt{self.command_topic}"
        configuration["allowlist"].append({"name": command})
        configuration["topics"].append(
            {
                "name": command,
                "qos": {
                    "reliability": True,
                    "durability": False,
                    "history-depth": 1,
                    "endpoint-profile-name": "robocat_base_command",
                },
            }
        )
        configuration["topic-routes"] = [
            {
                "name": command,
                "type": "geometry_msgs::msg::dds_::Twist_",
                "routes": [
                    {"src": robot["name"]},
                    {"src": laptop["name"], "dst": [robot["name"]]},
                ],
            }
        ]
        return configuration


# %% configuration output without starting DDS
def main() -> None:
    """
    Print a control profile derived from the validated feedback configuration.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base", type=Path, required=True)
    parser.add_argument(
        "--command-topic", choices=("/cmd_vel", "/snm_vel"), default="/cmd_vel"
    )
    options = parser.parse_args()
    yaml.safe_dump(
        RobocatBridgeConfiguration(options.base, options.command_topic).for_control(),
        sys.stdout,
        sort_keys=False,
    )


if __name__ == "__main__":
    main()
