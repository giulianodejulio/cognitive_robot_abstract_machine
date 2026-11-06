"""Bounded diagnostic output from the real base feedback already received by the client."""

# %% imports
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from enum import StrEnum

from typing_extensions import TYPE_CHECKING

if TYPE_CHECKING:
    from geometry_msgs.msg import Pose
    from nav_msgs.msg import Odometry


# %% planar pose and signed goal errors
@dataclass(frozen=True)
class PlanarPose:
    """A planar position and heading expressed in the odometry frame."""

    x: float
    """Odometry x coordinate in meters."""
    y: float
    """Odometry y coordinate in meters."""
    yaw: float
    """Heading in radians."""

    @classmethod
    def from_ros(cls, pose: Pose) -> PlanarPose:
        """Extract the planar coordinates without changing the feedback message."""
        rotation = pose.orientation
        return cls(
            x=pose.position.x,
            y=pose.position.y,
            yaw=math.atan2(
                2 * (rotation.w * rotation.z + rotation.x * rotation.y),
                1 - 2 * (rotation.y**2 + rotation.z**2),
            ),
        )


@dataclass(frozen=True)
class PlanarGoalError:
    """Target minus measured pose, resolved in the initial robot axes."""

    longitudinal: float
    """Forward error in meters; negative means the robot passed the target."""
    lateral: float
    """Leftward correction in meters; negative means the target is to the right."""
    angular_degrees: float
    """Shortest yaw correction in degrees; positive means turn left."""


class GoalErrorPhase(StrEnum):
    """The action stage at which the latest feedback is reported."""

    TRACKING = "tracking"
    """The action is still running."""
    RESULT = "result"
    """A result or execution error has arrived."""
    SETTLED = "settled"
    """The existing post-success sampling delay has elapsed."""


# %% at most one periodic line per second, plus outcome snapshots
@dataclass
class GoalErrorLog:
    """Report signed errors against one fixed target without retaining sample history."""

    start: PlanarPose
    """Pose captured when this request's target was constructed."""
    distance: float
    """Requested forward displacement in meters."""
    requested_at: float
    """Local monotonic request reception time."""
    turn_degrees: float = 0
    """Requested yaw change relative to the initial heading."""
    interval: float = 1.0
    """Minimum monotonic interval between periodic reports, in seconds."""
    last_report: float | None = field(default=None, init=False)
    """Monotonic time of the latest printed snapshot."""

    def measure(self, current: PlanarPose) -> PlanarGoalError:
        """Resolve translational errors and wrap yaw error to the shortest turn."""
        dx = current.x - self.start.x
        dy = current.y - self.start.y
        cosine, sine = math.cos(self.start.yaw), math.sin(self.start.yaw)
        yaw_error = self.start.yaw + math.radians(self.turn_degrees) - current.yaw
        return PlanarGoalError(
            longitudinal=self.distance - (cosine * dx + sine * dy),
            lateral=sine * dx - cosine * dy,
            angular_degrees=math.degrees(
                math.atan2(math.sin(yaw_error), math.cos(yaw_error))
            ),
        )

    def report(
        self, odometry: Odometry, phase: GoalErrorPhase, force: bool = False
    ) -> None:
        """Print the latest received laser odometry; forced snapshots bypass throttling."""
        now = time.monotonic()
        if (
            not force
            and self.last_report is not None
            and now - self.last_report < self.interval
        ):
            return
        self.last_report = now
        error = self.measure(PlanarPose.from_ros(odometry.pose.pose))
        stamp = odometry.header.stamp
        print(
            f"GOAL ERROR: phase={phase}; elapsed={now - self.requested_at:.3f} s; "
            f"longitudinal={error.longitudinal:+.4f} m; lateral={error.lateral:+.4f} m; "
            f"angular={error.angular_degrees:+.3f} deg; "
            f"planar={math.hypot(error.longitudinal, error.lateral):.4f} m; "
            f"odom_stamp={stamp.sec}.{stamp.nanosec:09d}",
            flush=True,
        )
