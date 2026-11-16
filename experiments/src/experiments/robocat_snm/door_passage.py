"""
Calibrated real-robot geometry for the native door-passing action.
"""

# %% imports
from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import StrEnum

from coraplex.datastructures.dataclasses import Context
from coraplex.robot_plans.actions.core.navigation import PassDoorAction
from experiments.robocat_snm.geometry import EntryGeometry
from giskardpy.motion_statechart.goals.templates import Parallel
from giskardpy.motion_statechart.monitors.sensor_monitors import PassageClear
from scripts.robocat_giskard_goal import RobocatStraightMotion
from scripts.tiago_giskard_goal import MotionTolerances
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Door,
    EntryWay,
)
from semantic_digital_twin.spatial_types import Pose, HomogeneousTransformationMatrix
from semantic_digital_twin.world_description.connections import FixedConnection
from semantic_digital_twin.world_description.geometry import VolumetricBoundingBox
from semantic_digital_twin.world_description.world_entity import Body, Region


# %% semantic doorway coordinates
class DoorwayFrame(StrEnum):
    """
    Client-side entities describing the calibrated doorway plane.
    """

    DOOR = "snm_calibrated_door"
    OPENING = "snm_calibrated_opening"


@dataclass
class CalibratedDoorPassage(PassDoorAction):
    """
    Use the native door plan with goals expressed in the server's world frame.
    """

    half_width: float = field(kw_only=True)
    """
    Half width of the calibrated opening in metres.
    """

    @property
    def _doorway(self) -> VolumetricBoundingBox:
        return VolumetricBoundingBox(
            0,
            -self.half_width,
            0,
            0,
            self.half_width,
            0,
            origin=HomogeneousTransformationMatrix(reference_frame=self._doorway_frame),
        )

    @classmethod
    def from_geometry(
        cls, geometry: EntryGeometry, context: Context
    ) -> CalibratedDoorPassage:
        """
        Represent the calibrated doorway plane and preserve the configured target.
        """
        world = context.world
        frame = Body(name=PrefixedName(DoorwayFrame.DOOR))
        opening = Region(name=PrefixedName(DoorwayFrame.OPENING))
        with world.modify_world():
            world.add_connection(
                FixedConnection(
                    parent=world.root,
                    child=frame,
                    parent_T_connection_expression=(
                        geometry.staging.homogeneous_matrix
                        @ Pose.from_xyz_rpy(
                            x=geometry.doorway_distance
                        ).homogeneous_matrix
                    ),
                )
            )
            world.add_connection(FixedConnection(parent=frame, child=opening))
        door = Door(root=frame, entry_way=EntryWay(root=opening))
        target = geometry.target.position
        target.reference_frame = world.root
        context.motion_tolerances.default_tcp_position_threshold = (
            geometry.position_tolerance
        )
        context.motion_tolerances.tool_orientation_threshold = (
            geometry.orientation_tolerance
        )
        context.alternative_motion_mappings = [RobocatDoorMotion]
        return cls(
            doors=[door],
            destination=target,
            approach_distance=geometry.doorway_distance,
            exit_distance=geometry.distance - geometry.doorway_distance,
            half_width=geometry.half_width,
        )

    def _pose_at(self, distance: float, heading: int) -> Pose:
        return self.world.transform(
            super()._pose_at(distance, heading), self.world.root
        )

    def _passage_clear(self) -> PassageClear:
        monitor = super()._passage_clear()
        monitor.passage_start = self.world.transform(
            monitor.passage_start, self.world.root
        )
        monitor.passage_end = self.world.transform(monitor.passage_end, self.world.root)
        return monitor


# %% native navigation motion mapping
@dataclass
class RobocatDoorMotion(RobocatStraightMotion):
    """
    Apply the request's Cartesian thresholds to native navigation stages.
    """

    @property
    def _motion_chart(self) -> Parallel:
        self.tolerances = MotionTolerances(
            self.context.motion_tolerances.default_tcp_position_threshold,
            math.degrees(self.context.motion_tolerances.tool_orientation_threshold),
        )
        return super()._motion_chart
