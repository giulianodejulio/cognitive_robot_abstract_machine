"""
Experimental longitudinal velocity lock for TIAGo rotation trials.
"""

# %% imports
from __future__ import annotations

from dataclasses import dataclass, field
from typing_extensions import TYPE_CHECKING

from giskardpy.motion_statechart.exceptions import UnexpectedWorldEntityCountError
from giskardpy.motion_statechart.graph_node import NodeArtifacts, Task
from krrood.symbolic_math.symbolic_math import Scalar
from semantic_digital_twin.world_description.connections import DifferentialDrive

if TYPE_CHECKING:
    from giskardpy.motion_statechart.context import MotionStatechartContext


# %% a hard constraint scoped to one rotation action
@dataclass(eq=False, repr=False)
class ZeroForwardVelocity(Task):
    """
    Require zero commanded longitudinal velocity throughout the prediction horizon.
    """

    reference_velocity: float = field(default=0.2, kw_only=True)
    """
    Positive normalization speed in m/s, independent of the zero velocity target.
    """

    def build_artifacts(self, context: MotionStatechartContext) -> NodeArtifacts:
        """
        Constrain the single differential drive without permitting slack.
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
        artifacts.constraints.add_velocity_eq_constraint(
            task_expression=drives[0].x_velocity.variables.position,
            velocity_goal=0,
            velocity_limit=self.reference_velocity,
            quadratic_weight=self.weight,
            lower_slack_limit=0,
            upper_slack_limit=0,
            name="zero_forward_velocity",
        )
        artifacts.observation = Scalar.const_true()
        return artifacts
