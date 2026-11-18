"""
Timestamp-aligned map poses propagated with measured odometry displacement.
"""

# %% imports
from __future__ import annotations

from dataclasses import dataclass
from nav_msgs.msg import Odometry
from semantic_digital_twin.adapters.ros.ros2_to_semdt_converters import (
    PoseToSemDTConverter,
)
from semantic_digital_twin.spatial_types import Pose
from semantic_digital_twin.world import World


# %% preserved alignment evidence
@dataclass(frozen=True)
class MapAlignmentEvidence:
    """
    Actual timestamps used to align a map TF with measured base feedback.
    """

    tf_stamp: float
    """
    Acquisition time represented by the map-to-base transform.
    """

    association_stamp: float
    """
    Timestamp of the selected odometry pose paired with the TF.
    """

    feedback_stamp: float
    """
    Timestamp of the current measured odometry used to propagate the pose.
    """


@dataclass(frozen=True)
class MapPoseAlignment:
    """
    A current map pose paired with the exact odometry message used for it.
    """

    pose: Pose
    """
    Map pose after applying the measured relative displacement.
    """
    feedback: Odometry
    """
    Current selected odometry message used in the alignment.
    """
    evidence: MapAlignmentEvidence
    """
    Sensor acquisition timestamps retained for diagnostics.
    """

    @classmethod
    def from_samples(
        cls,
        map_pose: Pose,
        acquisition: Odometry,
        current: Odometry,
        world: World,
        tf_stamp: float,
    ) -> MapPoseAlignment:
        """
        Apply measured displacement without extrapolating velocity or relaxing
        freshness.
        """
        previous_pose = PoseToSemDTConverter.convert(acquisition.pose.pose, world)
        current_pose = PoseToSemDTConverter.convert(current.pose.pose, world)
        transformation = (
            map_pose.homogeneous_matrix
            @ previous_pose.homogeneous_matrix.inverse()
            @ current_pose.homogeneous_matrix
        )
        return cls(
            Pose(
                position=transformation.position, orientation=transformation.quaternion
            ),
            current,
            MapAlignmentEvidence(
                tf_stamp,
                acquisition.header.stamp.sec + acquisition.header.stamp.nanosec / 1e9,
                current.header.stamp.sec + current.header.stamp.nanosec / 1e9,
            ),
        )
