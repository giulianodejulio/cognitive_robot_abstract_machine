"""Measured wheel settling for real Robocat motion completion."""

# %% imports
from __future__ import annotations

from dataclasses import dataclass, field

from experiments.robocat_safety import RobocatFeedback
from giskardpy.motion_statechart.context import (
    ContextExtension,
    MotionStatechartContext,
)
from giskardpy.motion_statechart.data_types import ObservationStateValues
from giskardpy.motion_statechart.graph_node import MotionStatechartNode, NodeArtifacts
from krrood.symbolic_math.symbolic_math import Scalar


# %% reuse the server's existing wheel feedback
@dataclass(frozen=True)
class MeasuredStopSample:
    """Wheel sample and verdict captured when the native monitor is evaluated."""

    stamp_sec: int
    """Original wheel sample timestamp seconds."""
    stamp_nanosec: int
    """Original wheel sample timestamp fractional nanoseconds."""
    observed_at: float
    """Server ROS time of monitor evaluation."""
    linear_velocity: float
    """Measured forward velocity in metres per second."""
    angular_velocity: float
    """Measured yaw rate in radians per second."""
    fresh: bool
    """Whether the sample satisfies the monitor's freshness bound."""
    settled: bool
    """Whether fresh wheels meet the existing sustained-stop criterion."""


@dataclass
class MeasuredStopContext(ContextExtension):
    """Wheel feedback already subscribed by the real base interface."""

    feedback: RobocatFeedback
    """Existing safety and wheel-settling input, independently of pose feedback choice."""


@dataclass(eq=False, repr=False)
class MeasuredBaseStop(MotionStatechartNode):
    """Observe real wheel settling rather than convergence of commanded velocity."""

    maximum_age: float = 0.5
    """Maximum absolute wheel timestamp age, matching entry feedback freshness."""
    last_sample: MeasuredStopSample | None = field(default=None, init=False, repr=False)
    """Exact wheel values used for this monitor's latest observation."""

    def build_artifacts(self, context: MotionStatechartContext) -> NodeArtifacts:
        """Require measured feedback support and start with an unknown verdict."""
        context.require_extension(MeasuredStopContext)
        return NodeArtifacts(observation=Scalar.const_trinary_unknown())

    def on_tick(self, context: MotionStatechartContext) -> ObservationStateValues:
        """Require fresh measured wheels and the existing sustained-rest criterion."""
        feedback = context.require_extension(MeasuredStopContext).feedback
        message = feedback.wheel_odometry
        if message is None:
            self.last_sample = None
            return ObservationStateValues.UNKNOWN
        now = feedback.node.get_clock().now().nanoseconds / 1e9
        stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        fresh = abs(now - stamp) <= self.maximum_age
        settled = fresh and feedback.wheels_settled()
        self.last_sample = MeasuredStopSample(
            message.header.stamp.sec,
            message.header.stamp.nanosec,
            now,
            message.twist.twist.linear.x,
            message.twist.twist.angular.z,
            fresh,
            settled,
        )
        if not fresh:
            return ObservationStateValues.UNKNOWN
        return ObservationStateValues.TRUE if settled else ObservationStateValues.FALSE
