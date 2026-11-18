"""
Straight entry with native pose regulation held until measured wheel settling.
"""

# %% imports
from dataclasses import dataclass, field

from experiments.robocat_completion import MeasuredBaseStop
from giskardpy.motion_statechart.goals.templates import Parallel
from giskardpy.motion_statechart.monitors.monitors import LocalMinimumReached
from scripts.robocat_giskard_goal import RobocatStraightMotion
from giskardpy.motion_statechart.context import MotionStatechartContext
from giskardpy.motion_statechart.graph_node import NodeArtifacts
from giskardpy.motion_statechart.tasks.cartesian_tasks import CartesianPositionStraight
from krrood.symbolic_math import symbolic_math as sm
from semantic_digital_twin.spatial_types import RotationMatrix
from experiments.robocat_wall import WallStopSettings
from experiments.robocat_snm.lidar_entry import LidarLongitudinalTask
from giskardpy.motion_statechart.tasks.cartesian_tasks import CartesianOrientation
import math


# %% axis-specific target acceptance
@dataclass(eq=False, repr=False)
class EntryPositionStraight(CartesianPositionStraight):
    """
    Retain native straight control with independent target-axis acceptance.
    """

    goal_orientation: RotationMatrix = field(kw_only=True)
    """
    Target heading defining longitudinal and lateral residuals.
    """

    lateral_tolerance: float = field(default=0.05, kw_only=True)
    """
    Maximum lateral residual in metres.
    """

    def build(self, context: MotionStatechartContext) -> NodeArtifacts:
        """
        Observe independent bounds without changing native motion constraints.
        """
        artifacts = super().build(context)
        current = context.world.compose_forward_kinematics_expression(
            self.root_link, self.tip_link
        ).position
        goal = self.root_T_goal_reference_frame @ self.goal_point
        rotation = (
            self.root_T_goal_reference_frame.rotation_matrix @ self.goal_orientation
        )
        residual = rotation.inverse() @ (current - goal)
        artifacts.observation = sm.logic_and(
            sm.abs(residual[0]) <= self.threshold,
            sm.abs(residual[1]) <= self.lateral_tolerance,
        )
        return artifacts


# %% entry-specific completion gate
@dataclass
class RobocatEntryMotion(RobocatStraightMotion):
    """
    Reach and hold the configured pose until real wheels have stopped.
    """

    lateral_tolerance: float = 0.05
    """
    Maximum lateral target error accepted at rest, in metres.
    """

    wall_stop: WallStopSettings | None = None
    """
    Optional lidar stop reference replacing fixed-position completion.
    """

    @property
    def _motion_chart(self) -> Parallel:
        """
        Hold pose until measured and native commanded velocities have settled.
        """
        original = super()._motion_chart
        chart = Parallel(
            name=original.name,
            nodes=[
                (
                    EntryPositionStraight(
                        name=node.name,
                        root_link=node.root_link,
                        tip_link=node.tip_link,
                        goal_point=node.goal_point,
                        goal_orientation=self.target.rotation_matrix,
                        threshold=node.threshold,
                        lateral_tolerance=self.lateral_tolerance,
                        weight=node.weight,
                        binding_policy=node.binding_policy,
                    )
                    if isinstance(node, CartesianPositionStraight)
                    else node
                )
                for node in original.nodes
            ],
        )
        if self.wall_stop is not None:
            chart = Parallel(
                name="LidarReferencedEntry",
                nodes=[
                    LidarLongitudinalTask(
                        name="wall_longitudinal",
                        root_link=self.world.root,
                        tip_link=self.robot.root,
                        goal_pose=self.target,
                        settings=self.wall_stop,
                        threshold=self.wall_stop.tolerance,
                    ),
                    CartesianOrientation(
                        name="orientation",
                        root_link=self.world.root,
                        tip_link=self.robot.root,
                        goal_orientation=self.target.rotation_matrix,
                        threshold=math.radians(15),
                    ),
                ],
            )
        chart.add_node(MeasuredBaseStop(name="measured_base_stop"))
        chart.add_node(
            LocalMinimumReached(
                name="commanded_base_stop",
                minimum_time=0.0,
                measure_from_own_start=False,
            )
        )
        return chart
