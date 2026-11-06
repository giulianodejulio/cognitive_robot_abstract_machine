# %% imports
from __future__ import annotations

from pathlib import Path

from experiments.robocat_bridge import RobocatBridgeConfiguration
from experiments.robocat_safety import RobocatFeedback
from test.experiments_test.test_robocat_giskard import feedback


# %% explicit command direction
def test_control_bridge_only_allows_command_to_robot() -> None:
    """
    Enable only Twist commands toward Robocat while keeping feedback one-way.
    """
    path = Path(__file__).parents[2] / "doc/robocat_dds_router.yaml"
    configuration = RobocatBridgeConfiguration(path).for_control()
    robot, laptop = configuration["participants"]
    assert configuration["topic-routes"] == [
        {
            "name": "rt/cmd_vel",
            "type": "geometry_msgs::msg::dds_::Twist_",
            "routes": [
                {"src": robot["name"]},
                {"src": laptop["name"], "dst": [robot["name"]]},
            ],
        }
    ]
    command = next(
        topic for topic in configuration["topics"] if topic["name"] == "rt/cmd_vel"
    )
    assert command["qos"]["durability"] is False
    assert command["qos"]["history-depth"] == 1
    assert {"name": "ros_discovery_info"} not in configuration["allowlist"]


def test_stale_wheel_feedback_blocks_execution(feedback: RobocatFeedback) -> None:
    """
    A cached stationary wheel sample cannot authorize continued control.
    """
    feedback.wheel_odometry.header.stamp.sec = 7
    assert feedback.ready(require_free_mux=False) is False
