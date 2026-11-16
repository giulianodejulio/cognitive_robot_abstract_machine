"""
Native spatial conversion, static-map checks and a CRAM waypoint motion.
"""

# %% imports
from __future__ import annotations

import hashlib
import math
import struct
from dataclasses import dataclass, field
from enum import StrEnum

import numpy as np
from nav_msgs.msg import OccupancyGrid
from scipy.ndimage import distance_transform_edt

from coraplex.robot_plans.motions.navigation import MoveMotion
from giskardpy.motion_statechart.tasks.cartesian_tasks import CartesianPose
from semantic_digital_twin.spatial_types import Pose


# %% map and odometry coordinate systems
class WaypointFrame(StrEnum):
    """
    Frames whose meaning is fixed for the waypoint experiments.
    """

    MAP = "map"
    BASE = "base_footprint"


@dataclass(frozen=True)
class WaypointSnapshot:
    """
    One stationary alignment between map, wheel and selected odometry poses.
    """

    map_start: Pose
    """
    Base-footprint pose in the localized map.
    """

    wheel_start: Pose
    """
    The same base-footprint pose in wheel odometry.
    """

    map_target: Pose
    """
    Selected final base-footprint pose in the map.
    """

    feedback_start: Pose | None = None
    """
    Base pose in the selected odometry; defaults to the wheel pose.
    """

    @property
    def wheel_target(self) -> Pose:
        """
        Express the map target in the current wheel-odometry coordinates.
        """
        return self.target_in(self.wheel_start)

    @property
    def feedback_target(self) -> Pose:
        """
        Express the map goal in the selected controller odometry coordinates.
        """
        return self.target_in(
            self.wheel_start if self.feedback_start is None else self.feedback_start
        )

    def target_in(self, odometry_start: Pose) -> Pose:
        """
        Transform the selected map target using a simultaneous odometry pose.
        """
        transformation = (
            odometry_start.homogeneous_matrix
            @ self.map_start.homogeneous_matrix.inverse()
            @ self.map_target.homogeneous_matrix
        )
        return Pose(
            position=transformation.position,
            orientation=transformation.quaternion,
        )


# %% static corridor preflight
@dataclass
class WaypointMap:
    """
    Veto straight segments through unknown cells, walls or insufficient clearance.
    """

    message: OccupancyGrid
    """
    The active occupancy grid, including its coordinate origin.
    """

    clearance: float = 0.35
    """
    Required static-map distance from the centreline to unavailable cells, in metres.
    """

    @property
    def fingerprint(self) -> str:
        """
        Identify map contents and geometry independently of publication time.
        """
        info = self.message.info
        origin = info.origin
        geometry = struct.pack(
            "<IId7d",
            info.width,
            info.height,
            info.resolution,
            origin.position.x,
            origin.position.y,
            origin.position.z,
            origin.orientation.x,
            origin.orientation.y,
            origin.orientation.z,
            origin.orientation.w,
        )
        return hashlib.sha256(
            geometry
            + self.message.header.frame_id.encode()
            + np.asarray(self.message.data, dtype=np.int8).tobytes()
        ).hexdigest()

    def blockers(self, start: Pose, target: Pose) -> list[str]:
        """
        Check a sampled centreline; this neither plans detours nor observes people.
        """
        return self.path_blockers([start, target])

    def path_blockers(self, poses: list[Pose] | tuple[Pose, ...]) -> list[str]:
        """
        Check every polyline segment against one static clearance calculation.
        """
        info = self.message.info
        if (
            not math.isfinite(info.resolution)
            or info.resolution <= 0
            or len(self.message.data) != info.width * info.height
        ):
            return ["Invalid occupancy grid"]
        if len(poses) < 2:
            return ["A path requires at least two poses"]
        vertices = np.array([[float(pose.x), float(pose.y)] for pose in poses])
        if not np.all(np.isfinite(vertices)):
            return ["Path contains nonfinite coordinates"]
        points = np.vstack(
            [
                start
                + np.linspace(
                    0,
                    1,
                    max(
                        2,
                        math.ceil(
                            float(np.linalg.norm(end - start)) / (info.resolution / 2)
                        )
                        + 1,
                    ),
                )[:, None]
                * (end - start)
                for start, end in zip(vertices[:-1], vertices[1:])
            ]
        )
        origin = info.origin
        origin_pose = Pose.from_xyz_quaternion(
            origin.position.x,
            origin.position.y,
            origin.position.z,
            origin.orientation.x,
            origin.orientation.y,
            origin.orientation.z,
            origin.orientation.w,
        )
        coordinates = np.column_stack(
            (points, np.zeros(len(points)), np.ones(len(points)))
        )
        local = coordinates @ origin_pose.homogeneous_matrix.inverse().to_np().T
        columns = np.floor(local[:, 0] / info.resolution).astype(int)
        rows = np.floor(local[:, 1] / info.resolution).astype(int)
        if np.any(
            (columns < 0) | (columns >= info.width) | (rows < 0) | (rows >= info.height)
        ):
            return ["Path leaves the known map bounds"]
        occupancy = np.asarray(self.message.data).reshape(info.height, info.width)
        free = (occupancy >= 0) & (occupancy < 20)
        distances = (
            distance_transform_edt(np.pad(free, 1, constant_values=False))[1:-1, 1:-1]
            * info.resolution
        )
        if np.any(distances[rows, columns] < self.clearance):
            return ["Path intersects unknown/occupied cells or lacks static clearance"]
        return []


# %% native CRAM motion chart
@dataclass
class WaypointMotion(MoveMotion):
    """
    Reach a pose with the native CartesianPose task and explicit tolerances.
    """

    position_tolerance: float = field(default=0.02, kw_only=True)
    """
    Position completion tolerance in the selected odometry, in metres.
    """

    orientation_tolerance_degrees: float = field(default=2.0, kw_only=True)
    """
    Heading completion tolerance in the selected odometry, in degrees.
    """

    @property
    def _motion_chart(self) -> CartesianPose:
        return CartesianPose(
            root_link=self.world.root,
            tip_link=self.robot.root,
            goal_pose=self.target,
            translation_threshold=self.position_tolerance,
            orientation_threshold=math.radians(self.orientation_tolerance_degrees),
        )
