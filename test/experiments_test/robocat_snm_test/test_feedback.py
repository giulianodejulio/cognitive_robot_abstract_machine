"""
Exercise native corridor observations and separate-command interlocks.
"""

# %% imports
import math
from unittest.mock import Mock

import numpy as np
import pytest
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from experiments.robocat_snm.feedback import EntryInputs
from experiments.robocat_snm.geometry import EntryGeometry
from semantic_digital_twin.datastructures.lidar_reading import LidarReading
from semantic_digital_twin.datastructures.scan_pattern import ScanPattern
from semantic_digital_twin.spatial_types import Pose
from semantic_digital_twin.world import World


# %% sensor geometry fixture
@pytest.fixture
def entry_inputs(monkeypatch: pytest.MonkeyPatch) -> EntryInputs:
    """
    Provide a real geometric scan with all live subscriptions replaced.
    """
    monkeypatch.setattr(EntryInputs, "__post_init__", lambda self: None)
    node, context = Mock(), Mock()
    node.get_clock.return_value.now.return_value.nanoseconds = 10_000_000_000
    world = World.create_with_root_body("laser")
    context.world = world
    context.robot.root.global_transform = Pose.from_xyz_rpy().homogeneous_matrix
    inputs = EntryInputs(node, context)
    inputs.feedback = Mock()
    inputs.feedback.blocking_reasons.return_value = []
    inputs.feedback.wheels_settled.return_value = True
    wheel = Odometry()
    wheel.header.stamp.sec = 10
    wheel.pose.pose.orientation.w = 1.0
    inputs.feedback.wheel_odometry = wheel
    inputs.scene = Mock(fingerprint="current-map")
    inputs.fingerprint = inputs.scene.fingerprint
    inputs.frozen_geometry = EntryGeometry(Pose.from_xyz_rpy(), doorway_distance=0.7)
    inputs.source = Mock(has_reading=True)
    scan = LaserScan()
    scan.header.stamp.sec = 10
    inputs.source.received_scan = scan
    pattern = ScanPattern(-math.pi / 2, math.pi / 2, math.pi / 360, 0.05, 10)
    inputs.source.get_lidar_reading.return_value = LidarReading(
        world.root, pattern, np.full(pattern.beam_count, 3.0)
    )
    return inputs


def test_fresh_corridor_has_measured_clearance(entry_inputs: EntryInputs) -> None:
    """
    Every checked native segment is clear in a fresh, finite scan.
    """
    observation = entry_inputs.observe(navigation_idle=True)
    assert observation.healthy is True
    assert observation.door_open is True
    assert observation.passage_clear is True


def test_stale_scan_cannot_certify_open_door(entry_inputs: EntryInputs) -> None:
    """
    The retained old scan becomes unknown rather than remaining clear.
    """
    entry_inputs.source.received_scan.header.stamp.sec = 8
    observation = entry_inputs.observe(navigation_idle=True)
    assert observation.healthy is False
    assert observation.door_open is None


def test_model_wheel_disagreement_blocks_entry(entry_inputs: EntryInputs) -> None:
    """
    A model in a different odometry frame cannot authorize motion.
    """
    entry_inputs.context.robot.root.global_transform = Pose.from_xyz_rpy(
        x=1
    ).homogeneous_matrix
    assert entry_inputs.observe(navigation_idle=True).healthy is False


def test_map_change_blocks_entry(entry_inputs: EntryInputs) -> None:
    """
    An active map replacement invalidates the captured handover geometry.
    """
    entry_inputs.scene.fingerprint = "different-map"
    assert entry_inputs.observe(navigation_idle=True).healthy is False
