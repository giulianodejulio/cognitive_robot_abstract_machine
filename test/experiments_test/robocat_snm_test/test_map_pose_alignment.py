"""
Delayed map TF is paired with acquisition-time odometry before measured propagation.
"""

# %% imports
import math
from unittest.mock import Mock

import pytest
from geometry_msgs.msg import TransformStamped
from experiments.robocat_snm.feedback import EntryInputs
from .test_scan_alignment import aligned_inputs, wheel_sample
from .test_feedback import entry_inputs


# %% temporal map alignment
@pytest.mark.parametrize(
    "yaw,expected_x,expected_y", [(0.0, 5.1, 0.0), (math.pi / 2, 5.0, 0.1)]
)
def test_delayed_tf_uses_historical_pose_and_current_measured_displacement(
    aligned_inputs: EntryInputs, yaw: float, expected_x: float, expected_y: float
) -> None:
    inputs = aligned_inputs
    inputs.node.get_clock.return_value.now.return_value.nanoseconds = 10_450_000_000
    inputs.feedback.receive_wheels(wheel_sample(10.0, x=1.0))
    current = wheel_sample(10.45, x=1.1)
    inputs.feedback.receive_wheels(current)
    transform = TransformStamped()
    transform.header.stamp.sec = 10
    transform.transform.translation.x = 5.0
    transform.transform.rotation.z = math.sin(yaw / 2)
    transform.transform.rotation.w = math.cos(yaw / 2)
    inputs.transforms = Mock()
    inputs.transforms.can_transform.return_value = True
    inputs.transforms.lookup_transform.return_value = transform
    pose = inputs.map_pose()
    assert pose is not None
    assert float(pose.x) == pytest.approx(expected_x)
    assert float(pose.y) == pytest.approx(expected_y)
    assert inputs.map_alignment.feedback is current


def test_missing_tf_timestamp_match_is_not_propagated(
    aligned_inputs: EntryInputs,
) -> None:
    inputs = aligned_inputs
    inputs.node.get_clock.return_value.now.return_value.nanoseconds = 10_450_000_000
    inputs.feedback.receive_wheels(wheel_sample(10.45))
    transform = TransformStamped()
    transform.header.stamp.sec = 10
    transform.transform.rotation.w = 1.0
    inputs.transforms = Mock()
    inputs.transforms.can_transform.return_value = True
    inputs.transforms.lookup_transform.return_value = transform
    assert inputs.map_pose() is None
