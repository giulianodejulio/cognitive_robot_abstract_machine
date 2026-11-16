"""
Timestamp association and acquisition-pose clearance for delayed real scans.
"""

# %% imports
from unittest.mock import Mock

import pytest
from nav_msgs.msg import Odometry

from experiments.robocat_snm.feedback import EntryFeedback
from semantic_digital_twin.spatial_types import Pose

from .test_feedback import entry_inputs


# %% timestamped feedback
@pytest.fixture
def aligned_inputs(entry_inputs, monkeypatch):
    """
    Retain the production wheel history without creating ROS subscriptions.
    """
    monkeypatch.setattr(EntryFeedback, "__post_init__", lambda self: None)
    feedback = EntryFeedback(entry_inputs.node)
    feedback.blocking_reasons = Mock(return_value=[])
    feedback.wheels_settled = Mock(return_value=True)
    entry_inputs.feedback = feedback
    return entry_inputs


def wheel_sample(stamp: float, x: float = 0.0) -> Odometry:
    """
    Construct a valid planar wheel sample at a given acquisition time.
    """
    message = Odometry()
    message.header.stamp.sec = int(stamp)
    message.header.stamp.nanosec = round((stamp - int(stamp)) * 1e9)
    message.header.frame_id = "odom"
    message.child_frame_id = "base_footprint"
    message.pose.pose.position.x = x
    message.pose.pose.orientation.w = 1.0
    return message


# %% observer regression
def test_delayed_fresh_scan_uses_historical_wheel_sample(aligned_inputs):
    """
    A fresh scan stays usable when the latest wheel timestamp is 300 ms ahead.
    """
    aligned_inputs.node.get_clock.return_value.now.return_value.nanoseconds = (
        10_300_000_000
    )
    aligned_inputs.feedback.receive_wheels(wheel_sample(10.0))
    aligned_inputs.feedback.receive_wheels(wheel_sample(10.3))
    observation = aligned_inputs.observe(navigation_idle=True)
    assert observation.healthy is True
    assert observation.passage_clear is True


def test_current_wheel_freshness_is_independent_of_scan_match(aligned_inputs):
    """
    An exact historical match cannot certify a stale current wheel stream.
    """
    aligned_inputs.feedback.receive_wheels(wheel_sample(10.0))
    aligned_inputs.node.get_clock.return_value.now.return_value.nanoseconds = (
        10_600_000_000
    )
    aligned_inputs.source.received_scan.header.stamp.nanosec = 400_000_000
    observation = aligned_inputs.observe(navigation_idle=True)
    assert observation.healthy is False
    assert observation.passage_clear is None


def test_missing_historical_match_blocks_entry(aligned_inputs):
    """
    Fresh individual streams are insufficient without a timestamp match.
    """
    aligned_inputs.node.get_clock.return_value.now.return_value.nanoseconds = (
        10_300_000_000
    )
    aligned_inputs.feedback.receive_wheels(wheel_sample(10.3))
    observation = aligned_inputs.observe(navigation_idle=True)
    assert observation.healthy is False
    assert observation.door_open is None


def test_acquisition_pose_is_used_for_clearance(aligned_inputs):
    """
    The threshold stays at its acquisition distance despite subsequent motion.
    """
    aligned_inputs.feedback.receive_wheels(wheel_sample(10.0, x=0.0))
    aligned_inputs.feedback.receive_wheels(wheel_sample(10.3, x=0.1))
    aligned_inputs.node.get_clock.return_value.now.return_value.nanoseconds = (
        10_300_000_000
    )
    aligned_inputs.context.robot.root.global_transform = Pose.from_xyz_rpy(
        x=0.1
    ).homogeneous_matrix
    aligned_inputs.visibility = Mock()
    aligned_inputs.visibility.evaluate.return_value = True
    aligned_inputs.observe(navigation_idle=True)
    start = aligned_inputs.visibility.evaluate.call_args_list[0].args[1]
    assert float(start.x) == pytest.approx(
        aligned_inputs.frozen_geometry.doorway_distance + 0.1
    )


# %% history selection
def test_history_selects_closest_timestamp(aligned_inputs):
    """
    The chosen pose comes from the nearest stamp rather than receipt order.
    """
    earlier = wheel_sample(9.99, x=0.1)
    closest = wheel_sample(10.001, x=0.2)
    latest = wheel_sample(10.3, x=0.3)
    for message in (earlier, closest, latest):
        aligned_inputs.feedback.receive_wheels(message)
    history = aligned_inputs.feedback.wheel_history
    assert history.nearest(10.0, aligned_inputs.maximum_skew) is closest


def test_history_storage_is_bounded(aligned_inputs):
    """
    Long-running subscriptions retain only the configured number of poses.
    """
    history = aligned_inputs.feedback.wheel_history
    for index in range(history.samples.maxlen + 1):
        aligned_inputs.feedback.receive_wheels(wheel_sample(10.0 + index / 50))
    assert len(history.samples) == history.samples.maxlen
    assert history.samples[0].header.stamp.nanosec == 20_000_000
