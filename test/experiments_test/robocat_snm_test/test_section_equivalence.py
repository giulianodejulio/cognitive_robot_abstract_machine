"""
Numeric endpoint preparation preserves the original native visibility verdicts.
"""

# %% imports
from dataclasses import replace
import math

import numpy as np
import pytest

from experiments.robocat_snm.feedback import EntryInputs
from semantic_digital_twin.datastructures.lidar_reading import LidarReading
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.spatial_types import Point3, Pose
from semantic_digital_twin.world_description.connections import FixedConnection
from semantic_digital_twin.world_description.world_entity import Body

from .test_feedback import entry_inputs


# %% identical geometric decisions
@pytest.mark.parametrize("value", [3.0, 0.3, math.inf, math.nan, -math.inf])
@pytest.mark.parametrize("distance", [0.7, 1.3])
def test_optimized_endpoints_match_original_world_frame_sections(
    entry_inputs: EntryInputs, value: float, distance: float
) -> None:
    """
    Free, blocked and unavailable scans retain their verdict under rotated frames.
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
                    x=0.113, y=0.02, z=0.3, yaw=0.2
                ).homogeneous_matrix,
            )
        )
    original = entry_inputs.source.get_lidar_reading.return_value
    reading = LidarReading(
        sensor, original.scan_pattern, np.full_like(original.ranges, value)
    )
    geometry = replace(
        entry_inputs.frozen_geometry,
        staging=Pose.from_xyz_rpy(x=0.1, y=-0.05, yaw=0.15),
    )
    entry_inputs.frozen_geometry = geometry
    compensation = Pose.from_xyz_rpy(x=0.04, yaw=0.03).homogeneous_matrix
    root_T_section = (
        compensation
        @ geometry.staging.homogeneous_matrix
        @ Pose.from_xyz_rpy(x=distance).homogeneous_matrix
    )
    points = [
        root_T_section @ Point3(0, lateral, 0)
        for lateral in (-geometry.half_width, geometry.half_width)
    ]
    start, end = [
        Point3(*point.to_np()[:3], reference_frame=world.root) for point in points
    ]
    expected = entry_inputs.visibility.evaluate(reading, start, end)
    assert entry_inputs.section_clear(reading, distance, compensation) is expected
