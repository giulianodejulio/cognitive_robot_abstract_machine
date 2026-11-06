# %% imports

from __future__ import annotations

from dataclasses import dataclass

from coraplex.alternative_motion_mapping import AlternativeMotion
from coraplex.datastructures.enums import ExecutionType
from coraplex.robot_plans.motions.navigation import MoveMotion
from giskardpy.motion_statechart.goals.cartesian_goals import DifferentialDriveBaseGoal
from krrood.ormatic.utils import classproperty
from semantic_digital_twin.robots.tiago_single_arm import TiagoSingleArm

# %% single-arm base planning


@dataclass
class SingleArmBaseMotion(MoveMotion, AlternativeMotion[TiagoSingleArm]):
    """
    Plan single-arm TIAGo base motions with differential-drive kinematics.
    """

    @classproperty
    def execution_type(cls) -> set[ExecutionType]:
        """
        The execution environments supported by this mapping.
        """
        return {ExecutionType.KINEMATICALLY_SIMULATED, ExecutionType.REAL}

    @property
    def _motion_chart(self) -> DifferentialDriveBaseGoal:
        return DifferentialDriveBaseGoal(goal_pose=self.target)
