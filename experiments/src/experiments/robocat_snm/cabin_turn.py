"""
Certified cabin-arrival authorization and a native pure-turn motion.
"""

# %% imports
from __future__ import annotations

import math
from dataclasses import dataclass

from coraplex.alternative_motion_mapping import AlternativeMotion
from experiments.robocat_completion import MeasuredBaseStop
from experiments.robocat_snm.geometry import EntryAssessment
from experiments.robocat_snm.policy import EntryObservation
from experiments.tiago_goal_logging import PlanarPose
from experiments.tiago_pure_turn import ZeroForwardVelocity
from giskardpy.motion_statechart.goals.templates import Parallel
from giskardpy.motion_statechart.monitors.monitors import LocalMinimumReached
from giskardpy.motion_statechart.tasks.cartesian_tasks import CartesianOrientation
from scripts.tiago_giskard_goal import PureTurnMotion
from semantic_digital_twin.robots.robocat import Robocat


# %% longitudinal authorization
@dataclass(frozen=True)
class CabinTurnAuthorization:
    """
    Require successful entry, fresh wall acceptance and measured rest before rotation.
    """

    entry_succeeded: bool
    """
    The owned entry action and entry policy both completed successfully.
    """

    observation: EntryObservation
    """
    A newly evaluated entry observation before any turn is submitted.
    """

    assessment: EntryAssessment | None
    """
    Measured cabin-wall acceptance belonging to that observation.
    """

    @property
    def allowed(self) -> bool:
        """
        Reject incomplete longitudinal arrival even if a native action reports success.
        """
        return (
            self.entry_succeeded
            and self.observation.healthy
            and self.observation.stationary
            and self.observation.navigation_idle
            and self.observation.target_reached
            and self.observation.fully_inside
            and self.assessment is not None
            and self.assessment.wall_reference is not None
            and self.assessment.target_reached
            and self.assessment.fully_inside
        )


# %% final orientation evidence
@dataclass(frozen=True)
class CabinTurnMeasurement:
    """
    Evaluate measured heading and displacement without commanding position recovery.
    """

    measured: PlanarPose
    """
    Current wheel-derived base pose.
    """
    target: PlanarPose
    """
    Starting position and doorway-facing target heading.
    """
    stamp: float
    """
    Wheel acquisition timestamp.
    """

    observed_at: float
    """
    ROS time sampled after capturing the wheel message.
    """

    position_tolerance: float
    """
    Maximum displacement from the starting position, in metres.
    """

    orientation_tolerance_deg: float
    """
    Final heading tolerance, in degrees.
    """

    maximum_age: float = 0.5
    """
    Maximum absolute wheel acquisition age, in seconds.
    """

    @property
    def fresh(self) -> bool:
        """
        Require finite planar feedback and a current acquisition timestamp.
        """
        return (
            all(
                math.isfinite(value)
                for value in (
                    self.measured.x,
                    self.measured.y,
                    self.measured.yaw,
                    self.stamp,
                    self.observed_at,
                )
            )
            and abs(self.observed_at - self.stamp) <= self.maximum_age
        )

    @property
    def displacement(self) -> float:
        """
        Planar displacement from the position at turn authorization.
        """
        return math.hypot(
            self.measured.x - self.target.x, self.measured.y - self.target.y
        )

    @property
    def yaw_error(self) -> float:
        """
        Signed target-heading residual wrapped to the shortest angular difference.
        """
        difference = self.measured.yaw - self.target.yaw
        return math.atan2(math.sin(difference), math.cos(difference))

    @property
    def reached(self) -> bool:
        """
        Require fresh heading acceptance without leaving the authorized position.
        """
        return (
            self.fresh
            and self.displacement <= self.position_tolerance
            and abs(self.yaw_error) <= math.radians(self.orientation_tolerance_deg)
        )


# %% native orientation task and zero forward command
@dataclass
class RobocatCabinTurnMotion(PureTurnMotion, AlternativeMotion[Robocat]):
    """
    Rotate with zero longitudinal command until heading and measured rest agree.
    """

    orientation_tolerance_deg: float = 5.0
    """
    Heading acceptance, in degrees, independent of the straight-entry target.
    """

    @property
    def _motion_chart(self) -> Parallel:
        """
        Reuse pure-turn constraints and retain them through measured wheel settling.
        """
        chart = super()._motion_chart
        for node in chart.nodes:
            if isinstance(node, CartesianOrientation):
                node.threshold = math.radians(self.orientation_tolerance_deg)
            elif isinstance(node, ZeroForwardVelocity):
                node.brake_before_lock = True
        chart.add_node(MeasuredBaseStop(name="measured_base_stop"))
        chart.add_node(
            LocalMinimumReached(
                name="commanded_base_stop",
                minimum_time=0.0,
                measure_from_own_start=False,
            )
        )
        return chart
