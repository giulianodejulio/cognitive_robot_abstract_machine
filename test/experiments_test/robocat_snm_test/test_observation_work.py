"""
Static map and common scan-frame transforms are not recomputed per section.
"""

# %% imports
from unittest.mock import Mock
import math

import pytest

from experiments.robocat_snm.feedback import EntryInputs
from experiments.robocat_waypoints import model
from experiments.robocat_waypoints.model import WaypointMap
from semantic_digital_twin.datastructures.lidar_reading import LidarReading
from semantic_digital_twin.spatial_types import Pose
from semantic_digital_twin.world_description.world_entity import Body
from semantic_digital_twin.world_description.connections import FixedConnection
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName

from .test_feedback import entry_inputs
from ..test_robocat_waypoints import waypoint_map


# %% static clearance reuse
def test_unchanged_map_reuses_clearance_field(
    waypoint_map: WaypointMap, monkeypatch: pytest.MonkeyPatch
) -> None:
    """
    Changing the segment preserves checks while avoiding another full map transform.
    """
    calculation = Mock(wraps=model.distance_transform_edt)
    monkeypatch.setattr(model, "distance_transform_edt", calculation)
    start = Pose.from_xyz_rpy(x=1, y=1)
    assert waypoint_map.blockers(start, Pose.from_xyz_rpy(x=2, y=1)) == []
    assert waypoint_map.blockers(start, Pose.from_xyz_rpy(x=2.1, y=1)) == []
    calculation.assert_called_once()


def test_map_change_invalidates_cached_clearance(
    waypoint_map: WaypointMap, monkeypatch: pytest.MonkeyPatch
) -> None:
    """
    A new obstacle is still rejected even when the ROS message is mutated in place.
    """
    calculation = Mock(wraps=model.distance_transform_edt)
    monkeypatch.setattr(model, "distance_transform_edt", calculation)
    start, end = Pose.from_xyz_rpy(x=1, y=1), Pose.from_xyz_rpy(x=2, y=1)
    assert waypoint_map.blockers(start, end) == []
    waypoint_map.message.data[10 * 40 + 15] = 100
    assert waypoint_map.blockers(start, end) != []
    assert calculation.call_count == 2


# %% native visibility in sensor coordinates
def test_sections_share_sensor_coordinates(entry_inputs: EntryInputs) -> None:
    """
    Native visibility receives already-transformed endpoints for every section.
    """
    world = entry_inputs.context.world
    with world.modify_world():
        sensor = Body(name=PrefixedName("offset_sensor"))
        world.add_body(sensor)
        world.add_connection(
            FixedConnection(
                parent=world.root,
                child=sensor,
                parent_T_connection_expression=Pose.from_xyz_rpy(
                    x=0.113, yaw=0.2
                ).homogeneous_matrix,
            )
        )
    original = entry_inputs.source.get_lidar_reading.return_value
    reading = LidarReading(sensor, original.scan_pattern, original.ranges)
    entry_inputs.source.get_lidar_reading.return_value = reading
    entry_inputs.visibility.evaluate = Mock(wraps=entry_inputs.visibility.evaluate)
    assert entry_inputs.observe(navigation_idle=True).healthy is True
    for call in entry_inputs.visibility.evaluate.call_args_list:
        assert call.args[1].reference_frame is sensor
        assert call.args[2].reference_frame is sensor
    start = entry_inputs.visibility.evaluate.call_args_list[0].args[1]
    x = entry_inputs.frozen_geometry.doorway_distance - 0.113
    y = -entry_inputs.frozen_geometry.half_width
    assert float(start.x) == pytest.approx(math.cos(0.2) * x + math.sin(0.2) * y)
    assert float(start.y) == pytest.approx(-math.sin(0.2) * x + math.cos(0.2) * y)


def test_resolution_change_invalidates_distance_field(
    waypoint_map: WaypointMap, monkeypatch: pytest.MonkeyPatch
) -> None:
    """
    Metric clearance is recomputed if occupancy-grid resolution changes.
    """
    calculation = Mock(wraps=model.distance_transform_edt)
    monkeypatch.setattr(model, "distance_transform_edt", calculation)
    before = waypoint_map.distance_field.copy()
    waypoint_map.message.info.resolution *= 2
    assert (waypoint_map.distance_field == before * 2).all()
    assert calculation.call_count == 2


def test_clearance_threshold_remains_dynamic_without_rebuilding_map(
    waypoint_map: WaypointMap, monkeypatch: pytest.MonkeyPatch
) -> None:
    """
    Reusing obstacle distances does not freeze the required safety clearance.
    """
    calculation = Mock(wraps=model.distance_transform_edt)
    monkeypatch.setattr(model, "distance_transform_edt", calculation)
    start, end = Pose.from_xyz_rpy(x=1, y=1), Pose.from_xyz_rpy(x=2, y=1)
    assert waypoint_map.blockers(start, end) == []
    waypoint_map.clearance = 1.5
    assert waypoint_map.blockers(start, end) != []
    calculation.assert_called_once()
