# %% imports
from __future__ import annotations

from dataclasses import dataclass, field

from krrood.symbolic_math.symbolic_math import if_less
from semantic_digital_twin.spatial_types import (
    Point3,
    Vector3,
    RotationMatrix,
)
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.world_description.connections import DifferentialDrive
from semantic_digital_twin.world_description.world_entity import (
    Body,
    KinematicStructureEntity,
)
from giskardpy.motion_statechart.goals.templates import Sequence, Parallel
from giskardpy.motion_statechart.binding_policy import GoalBindingPolicy
from giskardpy.motion_statechart.context import MotionStatechartContext
from giskardpy.motion_statechart.data_types import DefaultWeights
from giskardpy.motion_statechart.exceptions import UnexpectedWorldEntityCountError
from giskardpy.motion_statechart.graph_node import Goal, MotionStatechartNode
from giskardpy.motion_statechart.tasks.cartesian_tasks import (
    CartesianOrientation,
    CartesianPositionStraight,
    CartesianPositionVelocityLimit,
    CartesianPose,
)


# %% differential base motion
@dataclass(eq=False, repr=False)
class DifferentialDriveBaseGoal(Sequence):
    """
    A sequence that moves the robot to a goal pose using a differential drive.

    1. Orient to goal position
    2. Drive to goal position
    3. Orient to goal orientation

    The direction to the goal is an expression over the base's forward kinematics, so
    steps 1 and 2 follow the base as it drives. Initial alignment limits translation
    to :attr:`translation_velocity_while_turning`. A nearby goal uses its final
    orientation, and goal height follows the base's current height.
    """

    diff_drive_connection: DifferentialDrive | None = field(kw_only=True, default=None)
    """
    Drive connection to use.

    If it is None and there is only one diff drive in the world, it will be used.
    """

    goal_pose: Pose = field(kw_only=True)
    """
    Pose to reach.
    """

    weight: float = field(
        default=DefaultWeights.WEIGHT_ABOVE_COLLISION_AVOIDANCE, kw_only=True
    )
    """
    Task priority relative to other tasks.
    """

    nodes: list[MotionStatechartNode] = field(default_factory=list, init=False)

    threshold: float = field(default=0.01, kw_only=True)
    """
    Threshold when the drive goals for the base are considered achieved.
    """

    orientation_threshold: float | None = field(default=None, kw_only=True)
    """
    Angular completion tolerance in radians for the pose phases.

    None uses :attr:`threshold`. Initial alignment also respects
    :attr:`heading_threshold`.
    """

    heading_threshold: float = field(default=0.02, kw_only=True)
    """
    Maximum heading error in radians before the translation phase starts.
    """

    translation_velocity_while_turning: float = field(default=0.001, kw_only=True)
    """
    Maximum Cartesian translation speed during initial alignment, in meters/second.
    """

    def expand(self, context: MotionStatechartContext) -> None:
        """
        Build planar approach phases with bounded translation while aligning.
        """
        if self.diff_drive_connection is None:
            diff_drives = context.world.get_connections_by_type(DifferentialDrive)
            if len(diff_drives) == 0:
                raise UnexpectedWorldEntityCountError(
                    node=self,
                    expected_count=1,
                    actual_count=0,
                    entity_type=DifferentialDrive,
                )
            if len(diff_drives) > 1:
                raise UnexpectedWorldEntityCountError(
                    node=self,
                    expected_count=1,
                    actual_count=len(diff_drives),
                    entity_type=DifferentialDrive,
                )
            self.diff_drive_connection = diff_drives[0]
        root = context.world.root
        tip = self.diff_drive_connection.child
        orientation_threshold = (
            self.threshold
            if self.orientation_threshold is None
            else self.orientation_threshold
        )

        given_goal = context.world.transform(self.goal_pose, root)
        current_pose = context.world.compose_forward_kinematics_expression(root, tip)
        planar_goal = Pose(
            position=Point3(
                given_goal.x,
                given_goal.y,
                current_pose.z,
                reference_frame=root,
            ),
            orientation=given_goal.orientation,
            reference_frame=root,
        )
        direction = planar_goal.position - current_pose.position
        distance = direction.norm()
        direction.scale(1)
        vertical_axis = Vector3.Z(reference_frame=root)
        alignment_rotation = if_less(
            distance,
            self.threshold,
            planar_goal.rotation_matrix,
            RotationMatrix.from_vectors(
                x=direction, z=vertical_axis, reference_frame=root
            ),
        )
        alignment_rotation.reference_frame = root

        approach_pose = Pose(
            position=planar_goal.position,
            orientation=alignment_rotation.quaternion,
            reference_frame=root,
        )

        self.nodes = [
            Parallel(
                name=f"{self.name}/step1",
                nodes=[
                    CartesianOrientation(
                        name=f"{self.name}/step1/orientation",
                        root_link=root,
                        tip_link=tip,
                        goal_orientation=alignment_rotation,
                        weight=self.weight,
                        threshold=min(orientation_threshold, self.heading_threshold),
                    ),
                    CartesianPositionVelocityLimit(
                        name=f"{self.name}/step1/stay_in_place",
                        root_link=root,
                        tip_link=tip,
                        max_linear_velocity=self.translation_velocity_while_turning,
                        weight=DefaultWeights.WEIGHT_MAXIMUM,
                    ),
                ],
            ),
            CartesianPose(
                name=f"{self.name}/step2",
                root_link=root,
                tip_link=tip,
                goal_pose=approach_pose,
                weight=self.weight,
                translation_threshold=self.threshold,
                orientation_threshold=orientation_threshold,
            ),
            CartesianPose(
                name=f"{self.name}/step3",
                root_link=root,
                tip_link=tip,
                goal_pose=planar_goal,
                weight=self.weight,
                translation_threshold=self.threshold,
                orientation_threshold=orientation_threshold,
            ),
        ]
        super().expand(context)


# %% Cartesian pose motion
@dataclass(eq=False, repr=False)
class CartesianPoseStraight(Parallel):
    """
    Like CartesianPose, but constrains the tip link to move in a straight line towards
    the goal.
    """

    root_link: KinematicStructureEntity = field(kw_only=True)
    """
    Name of the root link of the kin chain.
    """

    tip_link: KinematicStructureEntity = field(kw_only=True)
    """
    Name of the tip link of the kin chain.
    """

    goal_pose: Pose = field(kw_only=True)
    """
    The goal pose.
    """

    weight: float = DefaultWeights.WEIGHT_ABOVE_COLLISION_AVOIDANCE
    """
    Task priority relative to other tasks.
    """

    binding_policy: GoalBindingPolicy = field(
        default=GoalBindingPolicy.Bind_at_build, kw_only=True
    )
    """
    Describes when the goal is computed.

    See GoalBindingPolicy for more information.
    """

    nodes: list[MotionStatechartNode] = field(default_factory=list, init=False)

    def expand(self, context: MotionStatechartContext) -> None:
        self.nodes = [
            CartesianPositionStraight(
                name=self.name + "/position",
                root_link=self.root_link,
                tip_link=self.tip_link,
                goal_point=self.goal_pose.position,
                weight=self.weight,
                binding_policy=self.binding_policy,
            ),
            CartesianOrientation(
                name=self.name + "/orientation",
                root_link=self.root_link,
                tip_link=self.tip_link,
                goal_orientation=self.goal_pose.rotation_matrix,
                weight=self.weight,
                binding_policy=self.binding_policy,
            ),
        ]
        super().expand(context)
