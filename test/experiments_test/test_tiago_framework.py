# %% imports

from __future__ import annotations

import numpy as np
import pytest

from coraplex.alternative_motion_mapping import AlternativeMotion
from coraplex.datastructures.dataclasses import Context
from coraplex.execution_environment import kinematically_simulated_robot, real_robot
from coraplex.plans.factories import execute_single
from coraplex.robot_plans import MoveMotion
from giskardpy.executor import Executor
from giskardpy.motion_statechart.context import MotionStatechartContext
from giskardpy.motion_statechart.goals.cartesian_goals import DifferentialDriveBaseGoal
from giskardpy.motion_statechart.graph_node import EndMotion
from giskardpy.motion_statechart.motion_statechart import MotionStatechart
from semantic_digital_twin.robots.tiago_single_arm import TiagoSingleArm
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix, Pose
from semantic_digital_twin.specifications.robots import RobotSpecification
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import (
    DifferentialDrive,
    FixedConnection,
)
from semantic_digital_twin.world_description.geometry import Box
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import Body

# %% framework robot


@pytest.fixture
def framework_robot() -> TiagoSingleArm:
    """
    Spawn the real robot description in an isolated framework world.
    """
    packages = pytest.importorskip("ament_index_python.packages")
    required = {
        "tiago_description",
        "pmb2_description",
        "pal_gripper_description",
        "pal_urdf_utils",
    }
    if not required.issubset(packages.get_packages_with_prefixes()):
        pytest.skip("PAL description packages are not installed")
    return RobotSpecification(semantic_annotation_type=TiagoSingleArm).spawn(
        World.create_with_root_body()
    )


def test_framework_spawn_controls_only_the_simulated_drive(
    framework_robot: TiagoSingleArm,
) -> None:
    """
    World spawning declares the simulated base without enabling arm joints.
    """
    assert type(framework_robot.drive) is DifferentialDrive
    assert framework_robot.drive.has_hardware_interface is True
    assert framework_robot.controlled_connections == []


# %% isolated planning


@pytest.mark.parametrize("lateral_distance", [0.0, 0.1])
def test_giskard_reaches_a_base_goal_in_the_single_arm_model(
    framework_robot: TiagoSingleArm, lateral_distance: float
) -> None:
    """
    The actual Giskard solver reaches a pose in an isolated TIAGo world.
    """
    world = framework_robot._world
    target = Pose.from_xyz_rpy(
        x=0.2, y=lateral_distance, yaw=0.3, reference_frame=world.root
    )
    chart = MotionStatechart()
    goal = DifferentialDriveBaseGoal(goal_pose=target)
    chart.add_node(goal)
    chart.add_node(EndMotion.when_true(goal))
    executor = Executor(MotionStatechartContext(world=world))

    executor.compile(motion_statechart=chart)
    executor.tick_until_end()

    np.testing.assert_allclose(
        framework_robot.root.global_transform.to_np(),
        target.to_np(),
        atol=goal.threshold,
    )


# %% coraplex integration


def test_coraplex_selects_differential_drive_for_the_single_arm_robot(
    framework_robot: TiagoSingleArm,
) -> None:
    """
    The high-level base motion selects differential-drive planning in simulation.
    """
    world = framework_robot._world
    motion = MoveMotion(Pose.from_xyz_rpy(x=0.2, reference_frame=world.root))
    context = Context(
        world=world,
        robot=framework_robot,
        alternative_motion_mappings=AlternativeMotion.discover_all(),
    )

    with kinematically_simulated_robot:
        execute_single(motion, context=context)
        assert type(motion.motion_chart) is DifferentialDriveBaseGoal


def test_real_mode_builds_a_differential_drive_goal_without_execution(
    framework_robot: TiagoSingleArm,
) -> None:
    """
    Real-mode plan construction selects the base goal without performing it.
    """
    world = framework_robot._world
    motion = MoveMotion(Pose.from_xyz_rpy(x=0.2, reference_frame=world.root))
    context = Context(
        world=world,
        robot=framework_robot,
        alternative_motion_mappings=AlternativeMotion.discover_all(),
    )

    with real_robot:
        execute_single(motion, context=context)
        assert type(motion.motion_chart) is DifferentialDriveBaseGoal


def test_coraplex_executes_a_base_goal_with_collision_avoidance(
    framework_robot: TiagoSingleArm,
) -> None:
    """
    The high-level motion completes through Giskard with collision avoidance enabled.
    """
    world = framework_robot._world
    target = Pose.from_xyz_rpy(x=0.2, reference_frame=world.root)
    motion = MoveMotion(target)
    context = Context(
        world=world,
        robot=framework_robot,
        alternative_motion_mappings=AlternativeMotion.discover_all(),
    )

    with kinematically_simulated_robot(collision_avoidance=True):
        plan = execute_single(motion, context=context)
        tolerance = motion.motion_chart.threshold
        plan.perform()

    np.testing.assert_allclose(
        framework_robot.root.global_transform.to_np(),
        target.to_np(),
        atol=tolerance,
    )


# %% collision checking


def test_framework_detects_an_obstacle_overlapping_the_base(
    framework_robot: TiagoSingleArm,
) -> None:
    """
    An external overlap changes the robot's collision state.
    """
    world = framework_robot._world
    world.collision_manager.update_collision_matrix()
    assert framework_robot.is_in_collision is False
    base_bounds = framework_robot.mobile_base.bounding_box
    obstacle_bounds = base_bounds.bloat(
        x_amount=base_bounds.depth,
        y_amount=base_bounds.width,
        z_amount=base_bounds.height,
    )
    with world.modify_world():
        obstacle = Body(
            name=PrefixedName("overlapping_obstacle"),
            collision=ShapeCollection([Box(scale=obstacle_bounds.scale)]),
        )
        world.add_connection(
            FixedConnection(
                parent=world.root,
                child=obstacle,
                parent_T_connection_expression=HomogeneousTransformationMatrix.from_point_rotation_matrix(
                    point=obstacle_bounds.center
                ),
            )
        )
    world.collision_manager.update_collision_matrix()

    assert framework_robot.is_in_collision is True
