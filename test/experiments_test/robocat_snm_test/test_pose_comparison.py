"""
Capture the exact model/odometry comparison, including rejected observations.
"""

# %% imports
import math
from unittest.mock import Mock

import pytest
from diagnostic_msgs.msg import DiagnosticStatus

from experiments.robocat_snm.feedback import EntryInputs
from semantic_digital_twin.spatial_types import Pose

from .test_feedback import entry_inputs


# %% comparison diagnostic
def test_rejected_model_pose_is_published_with_error_and_timestamps(
    entry_inputs: EntryInputs,
) -> None:
    """
    The bag receives the same poses and bounds that triggered cancellation.
    """
    entry_inputs.comparison_publisher = Mock()
    entry_inputs.comparison_publisher.get_subscription_count.return_value = 1
    entry_inputs.context.robot.root.global_transform = Pose.from_xyz_rpy(
        x=0.1, yaw=0.1
    ).homogeneous_matrix
    observation = entry_inputs.observe(navigation_idle=True)
    assert observation.healthy is False
    entry_inputs.comparison_publisher.publish.assert_called_once()
    message = entry_inputs.comparison_publisher.publish.call_args.args[0]
    values = {item.key: item.value for item in message.status[0].values}
    assert message.header.stamp.sec == 10
    assert float(values["model.x"]) == pytest.approx(0.1)
    assert float(values["feedback.x"]) == 0
    assert float(values["position_error_m"]) == pytest.approx(0.1)
    assert float(values["yaw_error_rad"]) == pytest.approx(0.1)
    assert float(values["feedback_stamp"]) == 10
    assert float(values["scan_stamp"]) == 10
    assert values["model_source_stamp_available"] == "False"
    assert message.status[0].level == DiagnosticStatus.WARN
    assert "position_error=0.1000" in "; ".join(entry_inputs.last_reasons)


def test_unsubscribed_diagnostic_does_not_publish_or_change_acceptance(
    entry_inputs: EntryInputs,
) -> None:
    """
    No subscriber means no diagnostic ROS traffic and no change to entry policy.
    """
    entry_inputs.comparison_publisher = Mock()
    entry_inputs.comparison_publisher.get_subscription_count.return_value = 0
    assert entry_inputs.observe(navigation_idle=True).healthy is True
    entry_inputs.comparison_publisher.publish.assert_not_called()


def test_recorded_target_uses_the_actual_submitted_coordinates(
    entry_inputs: EntryInputs,
) -> None:
    """
    Record both configured geometry and the submitted target without conflating them.
    """
    entry_inputs.comparison_publisher = Mock()
    entry_inputs.comparison_publisher.get_subscription_count.return_value = 1
    entry_inputs.snapshot = Mock()
    entry_inputs.snapshot.feedback_target = Pose.from_xyz_rpy(x=2, y=0.1)
    entry_inputs.observe(navigation_idle=True)
    message = entry_inputs.comparison_publisher.publish.call_args.args[0]
    values = {item.key: item.value for item in message.status[0].values}
    assert float(values["submitted_target.x"]) == 2
    assert float(values["geometry_target.x"]) == entry_inputs.frozen_geometry.distance
    assert float(values["submitted_target.y"]) == pytest.approx(0.1)
