"""Fixed entry geometry and native lidar segment clearance."""

# %% imports
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from experiments.robocat_snm.exceptions import EntryError
from semantic_digital_twin.datastructures.lidar_reading import LidarReading
from semantic_digital_twin.spatial_types import Point3, Pose


# %% final entry evidence
@dataclass(frozen=True)
class EntryAssessment:
    """Measured target precision and rear clearance under the configured entry model."""

    position_error: float
    """Planar distance from the final target in metres."""
    yaw_error: float
    """Signed heading error relative to the target in radians."""
    rear_clearance: float
    """Signed rear displacement beyond the threshold plus required margin, in metres."""
    position_tolerance: float
    """Maximum accepted planar target error in metres."""
    orientation_tolerance: float
    """Maximum accepted absolute target heading error in radians."""

    @property
    def fully_inside(self) -> bool:
        """Require positive rear clearance beyond the configured safety margin."""
        return self.rear_clearance > 0

    @property
    def target_reached(self) -> bool:
        """Require position and heading to satisfy their independent bounds."""
        return (
            self.position_error <= self.position_tolerance
            and abs(self.yaw_error) <= self.orientation_tolerance
        )

    def describe(self) -> str:
        """Explain final acceptance using measured residuals and their bounds."""
        return (
            f"ENTRY FINAL: position_error={self.position_error:.4f} m "
            f"(limit={self.position_tolerance:.4f} m); "
            f"yaw_error={math.degrees(self.yaw_error):.3f} deg "
            f"(limit={math.degrees(self.orientation_tolerance):.3f} deg); "
            f"rear_clearance_beyond_required={self.rear_clearance:.4f} m "
            f"(required > 0); fully_inside={self.fully_inside}; "
            f"target_reached={self.target_reached}"
        )


# %% configured straight entry
@dataclass(frozen=True)
class StagingResidual:
    """Measured pose error expressed along the saved staging axes."""

    longitudinal: float
    """Signed forward displacement from staging in metres."""
    lateral: float
    """Signed left displacement from staging in metres."""
    yaw: float
    """Signed heading error in radians."""

    @property
    def distance(self) -> float:
        """Planar distance from the staging position in metres."""
        return math.hypot(self.longitudinal, self.lateral)


@dataclass(frozen=True)
class StagingAlignment:
    """Position and heading bounds for a reached staging waypoint."""

    staging: Pose
    """Validated waypoint in the localized map."""
    position_tolerance: float = 0.02
    """Maximum staging position error in metres."""
    orientation_tolerance: float = math.radians(5)
    """Maximum staging heading error in radians."""

    def measure(self, pose: Pose) -> StagingResidual:
        """Express the measured displacement and heading in staging coordinates."""
        transform = self.staging.homogeneous_matrix.inverse() @ pose.homogeneous_matrix
        local = Pose(position=transform.position, orientation=transform.quaternion)
        return StagingResidual(float(local.x), float(local.y), float(local.yaw))

    def aligned(self, pose: Pose) -> bool:
        """Require both position and heading to match the staging waypoint."""
        residual = self.measure(pose)
        return (
            residual.distance <= self.position_tolerance
            and abs(residual.yaw) <= self.orientation_tolerance
        )


@dataclass(frozen=True)
class EntryGeometry:
    """A staging-aligned target with an explicitly measured threshold distance."""

    staging: Pose
    """Staging waypoint in the currently localized map."""
    doorway_distance: float
    """Measured distance from staging to the threshold along staging X, in metres."""
    distance: float = 1.5
    """Target displacement from staging along staging X, in metres."""
    half_width: float = 0.3
    """Half width of the checked corridor, including lateral clearance."""
    front_extent: float = 0.16
    """Forward chassis extent from base_footprint, in metres."""
    rear_extent: float = 0.30
    """Backward chassis extent from base_footprint, in metres."""
    margin: float = 0.1
    """Longitudinal clearance beyond the threshold and target chassis."""
    position_tolerance: float = 0.02
    """Completion position tolerance in metres."""
    orientation_tolerance: float = math.radians(5)
    """Completion heading tolerance in radians."""
    staging_tolerance: float = StagingAlignment.position_tolerance
    """Maximum staging position error before handing over."""
    section_spacing: float = 0.10
    """Maximum separation of checked transverse corridor sections, in metres."""
    staging_orientation_tolerance: float = StagingAlignment.orientation_tolerance
    """Maximum staging heading error before entry, independently of final heading accuracy."""

    def validate(self) -> None:
        """Reject nonfinite geometry and targets leaving the chassis on the threshold."""
        values = (
            float(self.staging.x),
            float(self.staging.y),
            float(self.staging.yaw),
            self.doorway_distance,
            self.distance,
            self.half_width,
            self.front_extent,
            self.rear_extent,
            self.margin,
            self.position_tolerance,
            self.orientation_tolerance,
            self.staging_tolerance,
            self.staging_orientation_tolerance,
            self.section_spacing,
        )
        if not all(math.isfinite(value) for value in values) or any(
            value <= 0 for value in values[3:]
        ):
            raise EntryError("Entry geometry must be finite with positive dimensions")
        if self.distance <= self.doorway_distance + self.rear_extent + self.margin:
            raise EntryError(
                "The configured target does not place the complete chassis inside"
            )
        if self.distance > 2.0 or self.half_width < 0.25:
            raise EntryError(
                "Initial trial supports entry distances up to 2 m and corridor widths of at least 0.5 m"
            )

    @property
    def target(self) -> Pose:
        """Target pose in map coordinates with staging heading retained."""
        map_T_target = (
            self.staging.homogeneous_matrix
            @ Pose.from_xyz_rpy(x=self.distance).homogeneous_matrix
        )
        return Pose(position=map_T_target.position, orientation=map_T_target.quaternion)

    def local_pose(self, pose: Pose) -> Pose:
        """Express a map pose in the fixed staging coordinates."""
        staging_T_pose = (
            self.staging.homogeneous_matrix.inverse() @ pose.homogeneous_matrix
        )
        return Pose(
            position=staging_T_pose.position, orientation=staging_T_pose.quaternion
        )

    def aligned(self, pose: Pose) -> bool:
        """Require the reached staging pose to permit an immediate forward entry."""
        return StagingAlignment(
            self.staging, self.staging_tolerance, self.staging_orientation_tolerance
        ).aligned(pose)

    def contains_robot(self, pose: Pose) -> bool:
        """Require the rear chassis to be inside, including heading and clearance."""
        return self.assess(pose).fully_inside

    def assess(self, pose: Pose) -> EntryAssessment:
        """Measure the final pose against target precision and rear-clearance bounds."""
        local = self.local_pose(pose)
        yaw = float(local.yaw)
        rear = (
            float(local.x)
            - self.rear_extent * abs(math.cos(yaw))
            - self.half_width * abs(math.sin(yaw))
        )
        return EntryAssessment(
            position_error=math.hypot(float(local.x) - self.distance, float(local.y)),
            yaw_error=yaw,
            rear_clearance=rear - (self.doorway_distance + self.margin),
            position_tolerance=self.position_tolerance,
            orientation_tolerance=self.orientation_tolerance,
        )

    def reached(self, pose: Pose) -> bool:
        """Check measured position and orientation independently of the action result."""
        return self.assess(pose).target_reached


# %% measured segment visibility
@dataclass
class SegmentVisibility:
    """Assess a segment using measured ranges and confirmed source no-return semantics."""

    tolerance: float = 0.05
    """Required observed depth past the segment, in metres."""
    minimum_beams: int = 3
    """Minimum number of independently measured intersecting beams."""
    endpoint_fraction: float = 0.10
    """Maximum unsampled fraction allowed at either end of a segment."""
    allow_isolated_gaps: bool = False
    """Explicit opt-in for interior single-ray gaps bounded by measured clear neighbours."""
    maximum_gap_width: float = 0.01
    """Maximum separation of bounding measured intersections across a missing ray, in metres."""
    no_return_range: float | None = None
    """Confirmed no-return search range for this source; None keeps infinities unknown."""

    def evaluate(
        self, reading: LidarReading, start: Point3, end: Point3
    ) -> bool | None:
        """Return unknown for missing coverage and false for a measured obstruction."""
        distances = reading.distances_to_segment(start, end)
        crossing = np.isfinite(distances)
        if np.count_nonzero(crossing) < self.minimum_beams:
            return None
        measured = reading.ranges[crossing]
        pattern = reading.scan_pattern
        valid = (
            np.isfinite(measured)
            & (measured >= pattern.minimum_range)
            & (measured <= pattern.maximum_range)
        )
        if np.any(np.isneginf(measured)):
            return False
        if np.any(measured[valid] <= distances[crossing][valid] + self.tolerance):
            return False
        if self.no_return_range is not None and math.isfinite(self.no_return_range):
            valid = valid | (
                np.isposinf(measured)
                & (
                    distances[crossing] + self.tolerance
                    <= min(self.no_return_range, pattern.maximum_range)
                )
            )
        if np.count_nonzero(valid) < self.minimum_beams:
            return None
        if not np.all(valid) and not self.allow_isolated_gaps:
            return None
        world = reading.reference_frame._world
        sensor_P_start = world.transform(start, reading.reference_frame).to_np()[:2]
        sensor_P_end = world.transform(end, reading.reference_frame).to_np()[:2]
        segment = sensor_P_end - sensor_P_start
        intersections = (
            pattern.beam_directions[crossing, :2] * distances[crossing, None]
        )
        for index in np.flatnonzero(~valid):
            if (
                index == 0
                or index == len(valid) - 1
                or not valid[index - 1]
                or not valid[index + 1]
            ):
                return None
            if (
                np.linalg.norm(intersections[index + 1] - intersections[index - 1])
                > self.maximum_gap_width
            ):
                return None
        fractions = ((intersections - sensor_P_start) @ segment) / float(
            segment @ segment
        )
        if (
            fractions.min() > self.endpoint_fraction
            or fractions.max() < 1 - self.endpoint_fraction
        ):
            return None
        return True
