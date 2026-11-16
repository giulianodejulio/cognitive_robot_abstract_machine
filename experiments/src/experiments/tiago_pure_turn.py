"""
Experimental longitudinal velocity lock for TIAGo rotation trials.
"""

# %% imports
from __future__ import annotations

from dataclasses import dataclass, field
from typing_extensions import TYPE_CHECKING

from giskardpy.motion_statechart.exceptions import UnexpectedWorldEntityCountError
from giskardpy.motion_statechart.graph_node import NodeArtifacts, Task
from giskardpy.qp.constraint import LargeNumber
from krrood.symbolic_math.symbolic_math import Scalar, abs, if_else, trinary_logic_and
from semantic_digital_twin.world_description.connections import DifferentialDrive

if TYPE_CHECKING:
    from giskardpy.motion_statechart.context import MotionStatechartContext


# %% a hard constraint scoped to one rotation action
@dataclass(eq=False, repr=False)
class ZeroForwardVelocity(Task):
    """
    Require zero forward velocity, optionally braking before the exact lock.
    """

    reference_velocity: float = field(default=0.2, kw_only=True)
    """
    Positive normalization speed in m/s, independent of the zero velocity target.
    """

    brake_before_lock: bool = field(default=False, kw_only=True)
    """
    Allow dynamically limited braking before applying the hard zero constraint.
    """

    stopped_velocity: float = field(default=0.001, kw_only=True)
    """
    Maximum absolute forward speed for the stopped condition, in m/s.
    """

    stopped_acceleration: float = field(default=0.001, kw_only=True)
    """
    Maximum absolute forward acceleration for the stopped condition, in m/s².
    """

    def build_artifacts(self, context: MotionStatechartContext) -> NodeArtifacts:
        """
        Brake with a soft zero target until settled, then enforce an exact lock.
        """
        drives = context.world.get_connections_by_type(DifferentialDrive)
        if len(drives) != 1:
            raise UnexpectedWorldEntityCountError(
                node=self,
                expected_count=1,
                actual_count=len(drives),
                entity_type=DifferentialDrive,
            )
        artifacts = NodeArtifacts()
        settled = Scalar.const_true()
        slack = Scalar(0)
        if self.brake_before_lock:
            forward = drives[0].x_velocity.variables
            settled = trinary_logic_and(
                abs(forward.velocity) <= self.stopped_velocity,
                abs(forward.acceleration) <= self.stopped_acceleration,
            )
            slack = if_else(settled, Scalar(0), Scalar(LargeNumber))
        artifacts.constraints.add_velocity_eq_constraint(
            task_expression=drives[0].x_velocity.variables.position,
            velocity_goal=0,
            velocity_limit=self.reference_velocity,
            quadratic_weight=self.weight,
            lower_slack_limit=-slack,
            upper_slack_limit=slack,
            name="zero_forward_velocity",
        )
        artifacts.observation = settled
        return artifacts
