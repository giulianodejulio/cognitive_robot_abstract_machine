"""
Compile the actual entry chart without executing control cycles.
"""

# %% imports
from pathlib import Path

import pytest

from coraplex.datastructures.dataclasses import Context
from coraplex.plans.factories import execute_single
from giskardpy.executor import Executor
from giskardpy.middleware.ros2.scripts.iai_robots.robocat import WorldWithRobocatConfig
from giskardpy.motion_statechart.context import MotionStatechartContext
from giskardpy.motion_statechart.tasks.cartesian_tasks import (
    CartesianOrientation,
    CartesianPositionStraight,
)
from scripts.robocat_giskard_goal import RobocatStraightMotion
from scripts.tiago_giskard_goal import MotionTolerances
from semantic_digital_twin.robots.robocat import Robocat
from semantic_digital_twin.spatial_types import Pose
from semantic_digital_twin.world_description.connections import DifferentialDrive


# %% real model constraints
def test_entry_motion_compiles_with_explicit_thresholds() -> None:
    """
    The production task produces native straight-position and orientation constraints.
    """
    configuration = WorldWithRobocatConfig(
        urdf=Path(Robocat.get_ros_file_path()).read_text()
    )
    with configuration.world.modify_world():
        configuration.setup_world()
        configuration.world.get_connections_by_type(DifferentialDrive)[
            0
        ].has_hardware_interface = True
    context = Context(world=configuration.world, robot=configuration.robot)
    tolerances = MotionTolerances(position=0.02, orientation_degrees=5)
    target = Pose.from_xyz_rpy(x=1.5, reference_frame=configuration.world.root)
    plan = execute_single(
        RobocatStraightMotion(target, tolerances=tolerances), context=context
    )
    executable = plan.parse()
    executable.prepare_for_execution()
    executor = Executor(context=MotionStatechartContext(world=configuration.world))
    executor.compile(executable.motion_state_chart)
    positions = [
        node
        for node in executable.motion_state_chart.nodes
        if isinstance(node, CartesianPositionStraight)
    ]
    orientations = [
        node
        for node in executable.motion_state_chart.nodes
        if isinstance(node, CartesianOrientation)
    ]
    assert len(positions) == 1
    assert positions[0].threshold == tolerances.position
    assert len(orientations) == 1
    assert float(orientations[0].threshold) == pytest.approx(0.08726646259971647)
    assert executor.control_cycles == 0
