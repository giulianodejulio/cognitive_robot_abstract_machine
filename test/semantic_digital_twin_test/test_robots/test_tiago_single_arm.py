# %% imports

from __future__ import annotations

import xml.etree.ElementTree as xml
from itertools import combinations

import pytest

from semantic_digital_twin.adapters.urdf import URDFParser
from semantic_digital_twin.collision_checking.collision_rules import (
    AllowCollisionForAdjacentPairs,
    AvoidExternalCollisions,
    AvoidSelfCollisions,
    SelfCollisionMatrixRule,
)
from semantic_digital_twin.robots.tiago_single_arm import (
    ObservedRobotCollisionExclusions,
    TiagoSingleArm,
    TiagoSingleArmJoint,
)
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import DifferentialDrive

# %% description


def test_joint_names_match_the_single_arm_description() -> None:
    """
    Every declared joint belongs to the captured single-arm description.
    """
    description = xml.parse(TiagoSingleArm.get_ros_file_path()).getroot()
    joint_names = {
        joint.attrib["name"]
        for joint in description.findall("joint")
        if joint.attrib["type"] != "fixed"
    }

    assert {joint.value for joint in TiagoSingleArmJoint} == joint_names


def test_single_arm_base_uses_differential_drive() -> None:
    """
    The single-arm robot retains its differential-drive kinematics.
    """
    assert TiagoSingleArm.get_drive_connection_type() is DifferentialDrive


def test_collision_description_retains_exactly_the_existing_link_pairs() -> None:
    """
    Filtering removes only exclusions referencing absent bodies.
    """
    directory = TiagoSingleArm.description_directory
    description = xml.parse(TiagoSingleArm.get_ros_file_path()).getroot()
    links = {link.attrib["name"] for link in description.findall("link")}
    original = xml.parse(directory / "original.srdf").getroot()
    filtered = xml.parse(directory / "collision.srdf").getroot()
    expected = [
        exclusion.attrib
        for exclusion in original.findall("disable_collisions")
        if {exclusion.attrib["link1"], exclusion.attrib["link2"]}.issubset(links)
    ]

    assert [
        exclusion.attrib for exclusion in filtered.findall("disable_collisions")
    ] == expected


# %% complete model


@pytest.fixture
def single_arm_robot() -> TiagoSingleArm:
    """
    Load the captured robot using the installed PAL description packages.
    """
    packages = pytest.importorskip("ament_index_python.packages")
    required_packages = {
        "tiago_description",
        "pmb2_description",
        "pal_gripper_description",
        "pal_urdf_utils",
    }
    if not required_packages.issubset(packages.get_packages_with_prefixes()):
        pytest.skip("PAL description packages are not installed")
    world = URDFParser.from_file(TiagoSingleArm.get_ros_file_path()).parse()
    return TiagoSingleArm.from_world(world)


def test_single_arm_hierarchy_contains_exactly_one_arm(
    single_arm_robot: TiagoSingleArm,
) -> None:
    """
    The torso exposes exactly the arm declared by its single-arm contract.
    """
    torso = single_arm_robot.mobile_base.torso

    assert torso.arms == [torso.arm]
    assert {connection.name.name for connection in torso.arm.active_connections} == {
        joint.value
        for joint in (
            TiagoSingleArmJoint.ARM_1,
            TiagoSingleArmJoint.ARM_2,
            TiagoSingleArmJoint.ARM_3,
            TiagoSingleArmJoint.ARM_4,
            TiagoSingleArmJoint.ARM_5,
            TiagoSingleArmJoint.ARM_6,
            TiagoSingleArmJoint.ARM_7,
        )
    }


def test_model_does_not_enable_hardware_interfaces(
    single_arm_robot: TiagoSingleArm,
) -> None:
    """
    Loading geometry leaves every physical joint interface disabled.
    """
    assert single_arm_robot.degrees_of_freedom_with_hardware_interface == []


def test_model_enables_external_and_self_collision_avoidance(
    single_arm_robot: TiagoSingleArm,
) -> None:
    """
    The captured collision matrix accompanies both avoidance rules.
    """
    manager = single_arm_robot._world.collision_manager

    assert [type(rule) for rule in manager.default_rules] == [
        AvoidExternalCollisions,
        AvoidSelfCollisions,
    ]
    assert [
        type(rule)
        for rule in manager.ignore_collision_rules
        if isinstance(rule, SelfCollisionMatrixRule)
    ] == [SelfCollisionMatrixRule]


def test_passive_model_keeps_the_self_collision_pairs_not_excluded_by_srdf(
    single_arm_robot: TiagoSingleArm,
) -> None:
    """
    Disabled command interfaces must not disable the SRDF's remaining checks.
    """
    manager = single_arm_robot._world.collision_manager
    exclusions = next(
        rule
        for rule in manager.ignore_collision_rules
        if isinstance(rule, SelfCollisionMatrixRule)
    )
    excluded_pairs = {
        frozenset([check.body_a, check.body_b])
        for check in exclusions.allowed_collision_pairs
    }
    expected_pairs = {
        frozenset(pair)
        for pair in combinations(single_arm_robot.bodies_with_collision, 2)
    } - excluded_pairs

    manager.update_collision_matrix()

    assert {
        frozenset([check.body_a, check.body_b])
        for check in manager.collision_matrix.collision_checks
    } == expected_pairs


def test_observed_robot_exclusions_preserve_unrelated_rigid_chain_rules(
    single_arm_robot: TiagoSingleArm, mini_world: World
) -> None:
    """
    Observing this robot leaves the base exclusion rule unchanged elsewhere.
    """
    root = mini_world.get_body_by_name("root")
    tip = mini_world.get_body_by_name("tip")
    reference = AllowCollisionForAdjacentPairs()
    observed = ObservedRobotCollisionExclusions(robots=[single_arm_robot])

    assert observed._no_controlled_connection_between_bodies(
        mini_world, root, tip
    ) == reference._no_controlled_connection_between_bodies(mini_world, root, tip)


# %% operator collision exclusions


def test_gripper_torso_pair_is_excluded_from_collision_checks(
    single_arm_robot: TiagoSingleArm,
) -> None:
    """
    The operator-authorized gripper and torso pair is ignored.
    """
    world = single_arm_robot._world
    gripper = world.get_body_by_name("gripper_link")
    torso = world.get_body_by_name("torso_fixed_column_link")
    world.collision_manager.update_collision_matrix()

    assert frozenset([gripper, torso]) not in {
        frozenset([check.body_a, check.body_b])
        for check in world.collision_manager.collision_matrix.collision_checks
    }
