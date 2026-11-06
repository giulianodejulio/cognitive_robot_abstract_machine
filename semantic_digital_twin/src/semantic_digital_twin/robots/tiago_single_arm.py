# %% imports

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import StrEnum
from importlib.resources import files
from pathlib import Path

from typing_extensions import Self, TYPE_CHECKING

from krrood.ormatic.utils import classproperty
from semantic_digital_twin.collision_checking.collision_rules import (
    AllowCollisionForAdjacentPairs,
    AvoidExternalCollisions,
    AvoidSelfCollisions,
    SelfCollisionMatrixRule,
)
from semantic_digital_twin.robots.robot_part_mixins import (
    HasLidar,
    HasMobileBase,
    HasNeck,
    HasOneArm,
    HasTorso,
    HasTwoFingers,
)
from semantic_digital_twin.robots.robot_parts import (
    AbstractRobot,
    AbstractRobotPart,
    Arm,
    EndEffector,
    Finger,
    MobileBase,
    Torso,
)
from semantic_digital_twin.robots.tiago import TiagoBaseLidar, TiagoNeck, TiagoTopic
from semantic_digital_twin.spatial_types import Vector3
from semantic_digital_twin.world_description.connections import DifferentialDrive

if TYPE_CHECKING:
    from semantic_digital_twin.datastructures.joint_state import JointState
    from semantic_digital_twin.world import World
    from semantic_digital_twin.world_description.world_entity import (
        Body,
        KinematicStructureEntity,
    )

# %% description names


class TiagoSingleArmJoint(StrEnum):
    """
    Movable joints in the description captured from TIAGo-72c.
    """

    ARM_1 = "arm_1_joint"
    ARM_2 = "arm_2_joint"
    ARM_3 = "arm_3_joint"
    ARM_4 = "arm_4_joint"
    ARM_5 = "arm_5_joint"
    ARM_6 = "arm_6_joint"
    ARM_7 = "arm_7_joint"
    LEFT_FINGER = "gripper_left_finger_joint"
    RIGHT_FINGER = "gripper_right_finger_joint"
    HEAD_1 = "head_1_joint"
    HEAD_2 = "head_2_joint"
    TORSO_LIFT = "torso_lift_joint"
    LEFT_WHEEL = "wheel_left_joint"
    RIGHT_WHEEL = "wheel_right_joint"


class TiagoSingleArmLink(StrEnum):
    """
    Bodies delimiting the single-arm robot's semantic parts.
    """

    FOOTPRINT = "base_footprint"
    BASE = "base_link"
    TORSO_FIXED = "torso_fixed_link"
    TORSO_LIFT = "torso_lift_link"
    ARM_TOOL = "arm_tool_link"
    GRIPPER = "gripper_link"
    GRASP = "gripper_grasping_frame"
    LEFT_FINGER = "gripper_left_finger_link"
    RIGHT_FINGER = "gripper_right_finger_link"


# %% geometry without command interfaces


@dataclass(eq=False)
class TiagoSingleArmPart(AbstractRobotPart, ABC):
    """
    A single-arm TIAGo part without enabled hardware command interfaces.
    """

    def setup_hardware_interfaces(self) -> None:
        pass

    def setup_joint_states(self) -> list[JointState]:
        return []


@dataclass(eq=False)
class TiagoSingleArmChain(TiagoSingleArmPart, ABC):
    """
    A robot part delimited by two bodies in the captured description.
    """

    @classproperty
    @abstractmethod
    def root_link(cls) -> TiagoSingleArmLink:
        """
        The body at the beginning of the part's kinematic chain.
        """

    @classproperty
    @abstractmethod
    def tip_link(cls) -> TiagoSingleArmLink:
        """
        The body at the end of the part's kinematic chain.
        """

    @classmethod
    def setup_default_configuration_in_world_below_robot_root(
        cls, robot_root: KinematicStructureEntity
    ) -> Self:
        world = robot_root._world
        return cls(
            root=world.get_body_in_branch_by_name(robot_root, cls.root_link),
            tip=world.get_body_in_branch_by_name(robot_root, cls.tip_link),
        )


# %% gripper and arm


@dataclass(eq=False)
class TiagoSingleArmLeftFinger(TiagoSingleArmChain, Finger):
    """
    The left sliding finger of the PAL parallel gripper.
    """

    @classproperty
    def root_link(cls) -> TiagoSingleArmLink:
        return TiagoSingleArmLink.GRIPPER

    @classproperty
    def tip_link(cls) -> TiagoSingleArmLink:
        return TiagoSingleArmLink.LEFT_FINGER


@dataclass(eq=False)
class TiagoSingleArmRightFinger(TiagoSingleArmChain, Finger):
    """
    The right sliding finger of the PAL parallel gripper.
    """

    @classproperty
    def root_link(cls) -> TiagoSingleArmLink:
        return TiagoSingleArmLink.GRIPPER

    @classproperty
    def tip_link(cls) -> TiagoSingleArmLink:
        return TiagoSingleArmLink.RIGHT_FINGER


@dataclass(eq=False)
class TiagoSingleArmGripper(
    TiagoSingleArmPart,
    EndEffector,
    HasTwoFingers[TiagoSingleArmLeftFinger, TiagoSingleArmRightFinger],
):
    """
    The PAL parallel gripper attached to the single arm.
    """

    @classmethod
    def setup_default_configuration_in_world_below_robot_root(
        cls, robot_root: KinematicStructureEntity
    ) -> Self:
        world = robot_root._world
        return cls(
            root=world.get_body_in_branch_by_name(
                robot_root, TiagoSingleArmLink.GRIPPER
            ),
            tool_frame=world.get_body_in_branch_by_name(
                robot_root, TiagoSingleArmLink.GRASP
            ),
        )

    @property
    def approach_axis(self) -> Vector3:
        return Vector3.X(reference_frame=self.tool_frame)

    @property
    def closing_axis(self) -> Vector3:
        return Vector3.Y(reference_frame=self.tool_frame)


@dataclass(eq=False)
class TiagoArm(TiagoSingleArmChain, Arm[TiagoSingleArmGripper]):
    """
    The seven-joint arm mounted on the lifting torso.
    """

    @classproperty
    def root_link(cls) -> TiagoSingleArmLink:
        return TiagoSingleArmLink.TORSO_LIFT

    @classproperty
    def tip_link(cls) -> TiagoSingleArmLink:
        return TiagoSingleArmLink.ARM_TOOL


# %% torso and base


@dataclass(eq=False)
class TiagoSingleArmNeck(TiagoNeck):
    """
    The shared TIAGo head geometry with hardware interfaces disabled.
    """

    def setup_hardware_interfaces(self) -> None:
        pass


@dataclass(eq=False)
class TiagoSingleArmTorso(
    TiagoSingleArmChain,
    Torso,
    HasOneArm[TiagoArm],
    HasNeck[TiagoSingleArmNeck],
):
    """
    The lifting torso carrying one arm and the sensor head.
    """

    @classproperty
    def root_link(cls) -> TiagoSingleArmLink:
        return TiagoSingleArmLink.TORSO_FIXED

    @classproperty
    def tip_link(cls) -> TiagoSingleArmLink:
        return TiagoSingleArmLink.TORSO_LIFT


@dataclass(eq=False)
class TiagoSingleArmBase(
    TiagoSingleArmPart,
    MobileBase[DifferentialDrive],
    HasTorso[TiagoSingleArmTorso],
    HasLidar[TiagoBaseLidar],
):
    """
    The differential-drive base reporting laser odometry.
    """

    @classproperty
    def topic_name(cls) -> str:
        """
        The odometry topic consistent with the robot's dynamic transforms.
        """
        return TiagoTopic.ODOMETRY

    @classproperty
    def forward_axis(cls) -> Vector3:
        return Vector3.X()

    @classmethod
    def setup_default_configuration_in_world_below_robot_root(
        cls, robot_root: KinematicStructureEntity
    ) -> Self:
        return cls(
            root=robot_root._world.get_body_in_branch_by_name(
                robot_root, TiagoSingleArmLink.BASE
            ),
        )


# %% collision exclusions


@dataclass
class ObservedRobotCollisionExclusions(AllowCollisionForAdjacentPairs):
    """
    Preserve ordinary rigid-chain exclusions outside the observed robots.
    """

    robots: list[AbstractRobot] = field(default_factory=list, kw_only=True)
    """
    Robots whose exclusions must come from their SRDF rather than commandability.
    """

    def _update(self, world: World) -> None:
        """
        Recompute :meth:`AllowCollisionForAdjacentPairs._update` without old exclusions.
        """
        self.allowed_collision_pairs.clear()
        super()._update(world)

    def _no_controlled_connection_between_bodies(
        self, world: World, body_a: Body, body_b: Body
    ) -> bool:
        """
        Retain observed robot pairs instead of applying
        :meth:`AllowCollisionForAdjacentPairs._no_controlled_connection_between_bodies`.
        """
        if any(
            body_a in robot.bodies_with_collision
            or body_b in robot.bodies_with_collision
            for robot in self.robots
        ):
            return False
        return super()._no_controlled_connection_between_bodies(world, body_a, body_b)


# %% complete robot


@dataclass(eq=False)
class TiagoSingleArm(AbstractRobot, HasMobileBase[TiagoSingleArmBase]):
    """
    The single-arm TIAGo-72c geometry with hardware interfaces disabled.
    """

    collision_configuration: Path = field(
        default_factory=lambda: TiagoSingleArm.description_directory / "collision.srdf"
    )
    """
    An SRDF whose collision exclusions match the captured description.
    """

    additional_collision_configuration: Path = field(
        default_factory=lambda: TiagoSingleArm.description_directory
        / "operator_exclusions.srdf"
    )
    """
    Additional collision exclusions explicitly requested by the operator.
    """

    @classproperty
    def description_directory(cls) -> Path:
        """
        The directory containing the descriptions captured from TIAGo-72c.
        """
        return (
            Path(files("semantic_digital_twin")).parent.parent
            / "resources"
            / "robots"
            / "tiago_single_arm"
        )

    @classmethod
    def get_ros_file_path(cls) -> str:
        return str(cls.description_directory / "robot.urdf")

    @classmethod
    def _get_root_body_name(cls) -> str:
        return TiagoSingleArmLink.FOOTPRINT

    def _setup_collision_rules(self) -> None:
        """
        Enable collision avoidance with the description's SRDF exclusions.
        """
        manager = self._world.collision_manager
        for index, rule in enumerate(manager.ignore_collision_rules):
            if isinstance(rule, ObservedRobotCollisionExclusions):
                rule.robots.append(self)
            elif isinstance(rule, AllowCollisionForAdjacentPairs):
                manager.ignore_collision_rules[index] = (
                    ObservedRobotCollisionExclusions(robots=[self])
                )
        exclusions = SelfCollisionMatrixRule.from_collision_srdf(
            str(self.collision_configuration), self._world
        )
        additional_exclusions = SelfCollisionMatrixRule.from_collision_srdf(
            str(self.additional_collision_configuration), self._world
        )
        exclusions.allowed_collision_pairs.update(
            additional_exclusions.allowed_collision_pairs
        )
        manager.add_ignore_collision_rule(exclusions)
        manager.extend_default_rules(
            [AvoidExternalCollisions(robot=self), AvoidSelfCollisions(robot=self)]
        )

    def _setup_velocity_limits(self) -> None:
        """
        Preserve the joint velocity limits supplied by the captured URDF.
        """
