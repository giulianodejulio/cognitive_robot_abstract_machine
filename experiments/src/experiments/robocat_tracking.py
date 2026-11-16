"""Publish numeric trajectory diagnostics without a second feedback/control loop."""

# %% imports
from __future__ import annotations

import math
import time
from dataclasses import asdict, dataclass, field
from enum import StrEnum

import numpy as np
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray, MultiArrayDimension

from giskardpy.motion_statechart.context import MotionStatechartContext
from giskardpy.motion_statechart.tasks.cartesian_tasks import (
    CartesianPositionTrajectory,
)
from giskardpy.motion_statechart.trajectory_tracking import (
    TrajectoryTrackingObserver,
    TrajectoryTrackingState,
)


# %% stable numeric schema
class TrackingField(StrEnum):
    """Metadata preceding the geometric numeric sample."""

    TOPIC = "/robocat_waypoint/tracking"
    SECONDS = "stamp_sec"
    NANOSECONDS = "stamp_nanosec"
    SEQUENCE = "sequence"


@dataclass
class TrackingSample:
    """Planar tracking quantities in radians, metres and the native path index."""

    desired_yaw: float
    """Yaw actually requested by heading tracking, including the arrival hold."""
    actual_yaw: float
    """Yaw of the synchronized base pose before computing the new command."""
    heading_error: float
    """Desired minus actual yaw wrapped to [-pi, pi]."""
    path_index: int
    """Current native tracker index, not a separately estimated closest point."""
    cross_track_error: float
    """Signed lateral displacement from the current segment; left is positive."""
    distance_to_goal: float
    """Euclidean distance from measured tip position to the final point."""
    remaining_distance: float
    """Native remaining-distance metric used for completion and progress checks."""
    target_x: float
    """Current lookahead point x coordinate in the trajectory frame."""
    target_y: float
    """Current lookahead point y coordinate in the trajectory frame."""
    base_x: float
    """Measured tip x coordinate in the trajectory frame."""
    base_y: float
    """Measured tip y coordinate in the trajectory frame."""

    @classmethod
    def from_state(
        cls,
        trajectory: CartesianPositionTrajectory,
        state: TrajectoryTrackingState,
        actual_yaw: float,
        remaining: float,
    ) -> TrackingSample:
        """Match the heading target and measure error against the active segment."""
        delta = state.target - state.position
        desired = (
            math.atan2(float(delta[1]), float(delta[0]))
            if trajectory.control_heading
            and np.linalg.norm(delta) >= trajectory.threshold
            else actual_yaw
        )
        points = trajectory._goal_points_np
        segment_index = min(trajectory.current_index, len(points) - 2)
        start, end = points[segment_index : segment_index + 2, :2]
        segment = end - start
        length = float(np.linalg.norm(segment))
        offset = state.position[:2] - start
        lateral = (
            float(segment[0] * offset[1] - segment[1] * offset[0]) / length
            if length > 0
            else 0.0
        )
        return cls(
            desired,
            actual_yaw,
            math.atan2(math.sin(desired - actual_yaw), math.cos(desired - actual_yaw)),
            int(trajectory.current_index),
            lateral,
            float(np.linalg.norm(points[-1] - state.position)),
            float(remaining),
            float(state.target[0]),
            float(state.target[1]),
            float(state.position[0]),
            float(state.position[1]),
        )


# %% bounded synchronous ROS publication
@dataclass
class TrackingPublisher(TrajectoryTrackingObserver):
    """Publish only subscribed heading-tracking samples at a bounded rate."""

    node: Node
    """Server node sharing the ROS clock used for command publication."""
    frequency: float = 10.0
    """Maximum diagnostic publication rate in Hz."""
    publisher: Publisher = field(init=False)
    """Best-effort depth-one numeric publisher."""
    _last_publication: float = field(default=-math.inf, init=False)
    """Monotonic time of the last emitted sample."""
    _trajectory: CartesianPositionTrajectory | None = field(
        default=None, init=False, repr=False
    )
    """Last sampled task identity, used to distinguish successive goals."""
    _sequence: int = field(default=0, init=False)
    """Local counter incremented for each newly observed trajectory task."""

    def __post_init__(self) -> None:
        """Create the diagnostic publisher without starting a timer or worker."""
        self.publisher = self.node.create_publisher(
            Float64MultiArray,
            TrackingField.TOPIC,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )

    def observe(
        self,
        trajectory: CartesianPositionTrajectory,
        context: MotionStatechartContext,
        state: TrajectoryTrackingState,
    ) -> None:
        if (
            not trajectory.control_heading
            or self.publisher.get_subscription_count() == 0
        ):
            return
        instant = time.monotonic()
        if instant - self._last_publication < 1 / self.frequency:
            return
        self._last_publication = instant
        if trajectory is not self._trajectory:
            self._trajectory = trajectory
            self._sequence += 1
        pose = context.world.compute_forward_kinematics(
            trajectory.goal_reference_frame, trajectory.tip_link
        ).to_np()
        yaw = math.atan2(float(pose[1, 0]), float(pose[0, 0]))
        sample = TrackingSample.from_state(
            trajectory,
            state,
            yaw,
            context.float_variable_data.get_value(trajectory.remaining_distance),
        )
        stamp = self.node.get_clock().now().to_msg()
        values = {
            TrackingField.SECONDS: stamp.sec,
            TrackingField.NANOSECONDS: stamp.nanosec,
            TrackingField.SEQUENCE: self._sequence,
            **asdict(sample),
        }
        message = Float64MultiArray(data=[float(value) for value in values.values()])
        message.layout.dim = [
            MultiArrayDimension(
                label=",".join(values), size=len(values), stride=len(values)
            )
        ]
        self.publisher.publish(message)

    def close(self) -> None:
        """Release the publisher and the last trajectory reference."""
        self.node.destroy_publisher(self.publisher)
        self._trajectory = None
