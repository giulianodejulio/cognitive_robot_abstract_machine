# %% imports
from __future__ import annotations

import xml.etree.ElementTree as xml
from dataclasses import dataclass
from enum import StrEnum
from importlib.resources import files
from pathlib import Path
from tempfile import NamedTemporaryFile

from typing_extensions import TYPE_CHECKING, Self

from krrood.ormatic.utils import classproperty
from semantic_digital_twin.adapters.sensors.lidar import Lidar, LidarSource
from semantic_digital_twin.collision_checking.collision_rules import (
    AllowCollisionForAdjacentPairs,
    AvoidExternalCollisions,
    AvoidSelfCollisions,
    SelfCollisionMatrixRule,
)
from semantic_digital_twin.collision_checking.collision_matrix import CollisionCheck
from semantic_digital_twin.datastructures.scan_pattern import ScanPattern
from semantic_digital_twin.robots.robot_part_mixins import HasLidar, HasMobileBase
from semantic_digital_twin.robots.robot_parts import AbstractRobot, MobileBase
from semantic_digital_twin.robots.tiago_single_arm import (
    ObservedRobotCollisionExclusions,
)
from semantic_digital_twin.spatial_types import Vector3
from semantic_digital_twin.world_description.connections import DifferentialDrive

if TYPE_CHECKING:
    from semantic_digital_twin.datastructures.joint_state import JointState
    from semantic_digital_twin.world_description.world_entity import (
        KinematicStructureEntity,
    )


# %% observed names
class RobocatLink(StrEnum):
    """
    Frames in the acquired Robocat-2c description.
    """

    FOOTPRINT = "base_footprint"
    BASE = "base_link"
    LASER = "laser_link"


# %% scanner and differential base
@dataclass(eq=False)
class RobocatLidar(Lidar):
    """
    The real YDLIDAR scanner mounted above the mobile base.
    """

    @classproperty
    def topic_name(cls) -> str:
        return "/scan"

    @classmethod
    def with_source(
        cls, robot_root: KinematicStructureEntity, source: LidarSource
    ) -> Self:
        return cls(
            root=robot_root._world.get_body_in_branch_by_name(
                robot_root, RobocatLink.LASER
            ),
            scan_pattern=ScanPattern(
                minimum_angle=-3.1415927410125732,
                maximum_angle=3.1415927410125732,
                angle_increment=0.0031431643292307854,
                minimum_range=0.009999999776482582,
                maximum_range=30.0,
            ),
            source=source,
        )


@dataclass(eq=False)
class RobocatBase(MobileBase[DifferentialDrive], HasLidar[RobocatLidar]):
    """
    The differential base whose measured pose comes from DLO.
    """

    @classproperty
    def topic_name(cls) -> str:
        return "/direct_laser_odometry/odom"

    @classproperty
    def forward_axis(cls) -> Vector3:
        return Vector3.X()

    @classmethod
    def setup_default_configuration_in_world_below_robot_root(
        cls, robot_root: KinematicStructureEntity
    ) -> Self:
        return cls(
            root=robot_root._world.get_body_in_branch_by_name(
                robot_root, RobocatLink.BASE
            )
        )

    def setup_hardware_interfaces(self) -> None:
        pass

    def setup_joint_states(self) -> list[JointState]:
        return []


# %% complete observed robot
@dataclass(eq=False)
class Robocat(AbstractRobot, HasMobileBase[RobocatBase]):
    """
    Robocat geometry with all physical joint command interfaces disabled.
    """

    @classproperty
    def description_directory(cls) -> Path:
        """
        The acquired description and mesh packages in the local workspace.
        """
        return (
            Path(files("semantic_digital_twin")).parent.parent
            / "resources/robots/robocat"
        )

    @classmethod
    def get_ros_file_path(cls) -> str:
        """
        Return a temporary URDF with mesh paths resolved to the copied resources.
        """
        document = xml.parse(cls.description_directory / "original.urdf")
        for link in document.findall("link"):
            inertial = link.find("inertial")
            if inertial is not None and (
                inertial.find("mass") is None or inertial.find("inertia") is None
            ):
                link.remove(inertial)
        for mesh in document.findall(".//mesh"):
            uri = mesh.attrib["filename"]
            relative = (
                uri.removeprefix("package://")
                if uri.startswith("package://")
                else uri.split("/share/", 1)[1]
            )
            mesh.attrib["filename"] = (
                cls.description_directory / "packages" / relative
            ).as_uri()
        with NamedTemporaryFile(
            mode="w", prefix="cram-robocat-", suffix=".urdf", delete=False
        ) as description:
            document.write(description, encoding="unicode")
            return description.name

    @classmethod
    def _get_root_body_name(cls) -> str:
        return RobocatLink.FOOTPRINT

    def _setup_collision_rules(self) -> None:
        manager = self._world.collision_manager
        manager.extend_default_rules(
            [AvoidExternalCollisions(robot=self), AvoidSelfCollisions(robot=self)]
        )
        for index, rule in enumerate(manager.ignore_collision_rules):
            if isinstance(rule, ObservedRobotCollisionExclusions):
                rule.robots.append(self)
            elif isinstance(rule, AllowCollisionForAdjacentPairs):
                manager.ignore_collision_rules[index] = (
                    ObservedRobotCollisionExclusions(robots=[self])
                )
        manager.add_ignore_collision_rule(
            SelfCollisionMatrixRule(
                allowed_collision_pairs={
                    CollisionCheck.create_for_bodies_with_collision(
                        connection.parent, connection.child
                    )
                    for connection in self.connections
                    if connection.parent.has_collision()
                    and connection.child.has_collision()
                }
            )
        )

    def _setup_velocity_limits(self) -> None:
        """
        Retain the velocity bounds of the acquired wheel and head joints.
        """
