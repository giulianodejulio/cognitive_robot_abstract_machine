"""Timestamped evidence of the model and odometry poses compared by SNM."""

# %% imports
from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field
from enum import StrEnum

from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import Header

from experiments.robocat_safety import FeedbackSource
from experiments.robocat_snm.geometry import EntryAssessment
from experiments.tiago_goal_logging import PlanarPose
from semantic_digital_twin.spatial_types import Pose


# %% diagnostic schema
class PoseDiagnostic(StrEnum):
    """ROS identity and verdicts for local pose comparisons."""

    TOPIC = "/snm/pose_comparison"
    STATUS = "snm/model_pose"
    CONSISTENT = "Model and feedback agree"
    DISAGREE = "Model and feedback disagree"


@dataclass(frozen=True)
class PoseComparison:
    """Poses sampled at one comparison, without inventing a model acquisition stamp."""

    model: PlanarPose
    """Model pose observed locally at the comparison instant."""
    feedback: PlanarPose
    """Selected odometry pose actually compared with the model."""
    geometry_target: PlanarPose
    """Inside target derived from frozen entry geometry."""
    submitted_target: PlanarPose
    """Inside target prepared for the owned motion request."""
    source: FeedbackSource
    """Selected laser or wheel pose source."""
    frame_id: str
    """Common reference frame of the recorded poses."""
    model_observed_at: float
    """ROS comparison time, not the model's original sensor acquisition time."""
    feedback_stamp: float
    """Original timestamp of the selected odometry message."""
    scan_stamp: float
    """Original timestamp of the captured scan."""
    position_limit_m: float
    """Unchanged model/feedback position disagreement limit."""
    yaw_limit_rad: float
    """Unchanged model/feedback heading disagreement limit."""
    model_source_stamp_available: bool = field(default=False, init=False)
    """Native world synchronization does not provide an acquisition timestamp."""
    assessment: EntryAssessment | None = None
    """Entry precision and rear-clearance quantities computed from this odometry sample."""

    @staticmethod
    def planar(pose: Pose) -> PlanarPose:
        """Capture immutable numeric planar coordinates from a spatial pose."""
        return PlanarPose(float(pose.x), float(pose.y), float(pose.yaw))

    @property
    def position_error_m(self) -> float:
        """Planar separation between the two poses at this comparison."""
        return math.hypot(
            self.model.x - self.feedback.x, self.model.y - self.feedback.y
        )

    @property
    def yaw_error_rad(self) -> float:
        """Signed model minus feedback yaw wrapped to the shortest angular difference."""
        difference = self.model.yaw - self.feedback.yaw
        return math.atan2(math.sin(difference), math.cos(difference))

    @property
    def disagrees(self) -> bool:
        """Whether the sampled values exceed either existing disagreement bound."""
        return (
            self.position_error_m > self.position_limit_m
            or abs(self.yaw_error_rad) > self.yaw_limit_rad
        )

    def describe(self) -> str:
        """Explain a disagreement with measured distances and timestamps."""
        return (
            f"Giskard model and {self.source.value} pose disagree: "
            f"position_error={self.position_error_m:.4f} m (limit={self.position_limit_m:.4f}); "
            f"yaw_error={math.degrees(self.yaw_error_rad):.3f} deg "
            f"(limit={math.degrees(self.yaw_limit_rad):.3f}); "
            f"feedback_age={self.model_observed_at-self.feedback_stamp:.3f} s; "
            f"model observed at={self.model_observed_at:.6f}; feedback stamp={self.feedback_stamp:.6f}"
        )

    def to_message(self) -> DiagnosticArray:
        """Publish native diagnostic fields with the comparison's own ROS timestamp."""
        values = []
        for name, value in asdict(self).items():
            if isinstance(value, dict):
                values.extend(
                    KeyValue(key=f"{name}.{axis}", value=str(number))
                    for axis, number in value.items()
                )
            else:
                values.append(KeyValue(key=name, value=str(value)))
        values.extend(
            [
                KeyValue(key="position_error_m", value=str(self.position_error_m)),
                KeyValue(key="yaw_error_rad", value=str(self.yaw_error_rad)),
                KeyValue(
                    key="feedback_age_s",
                    value=str(self.model_observed_at - self.feedback_stamp),
                ),
                KeyValue(
                    key="scan_age_s",
                    value=str(self.model_observed_at - self.scan_stamp),
                ),
            ]
        )
        nanoseconds = round(self.model_observed_at * 1e9)
        return DiagnosticArray(
            header=Header(
                stamp=Time(sec=nanoseconds // 10**9, nanosec=nanoseconds % 10**9),
                frame_id=self.frame_id,
            ),
            status=[
                DiagnosticStatus(
                    level=(
                        DiagnosticStatus.WARN if self.disagrees else DiagnosticStatus.OK
                    ),
                    name=PoseDiagnostic.STATUS,
                    message=(
                        PoseDiagnostic.DISAGREE
                        if self.disagrees
                        else PoseDiagnostic.CONSISTENT
                    ),
                    values=values,
                )
            ],
        )
