"""
A cabin turn requires certified longitudinal arrival and measured rest.
"""

# %% imports
from dataclasses import replace
import math
from pathlib import Path
from unittest.mock import Mock

import pytest

from coraplex.datastructures.dataclasses import Context
from coraplex.plans.factories import execute_single
from experiments.robocat_snm.application import EntryOptions
from experiments.robocat_snm.cabin_turn import (
    CabinTurnAuthorization,
    RobocatCabinTurnMotion,
)
from experiments.robocat_snm.geometry import EntryGeometry
from experiments.robocat_snm.policy import EntryObservation
from experiments.robocat_completion import MeasuredBaseStop
from experiments.tiago_pure_turn import ZeroForwardVelocity
from giskardpy.middleware.ros2.scripts.iai_robots.robocat import WorldWithRobocatConfig
from giskardpy.motion_statechart.tasks.cartesian_tasks import (
    CartesianOrientation,
    CartesianPosition,
)
from giskardpy.motion_statechart.goals.templates import NodeListGoal
from semantic_digital_twin.robots.robocat import Robocat
from semantic_digital_twin.spatial_types import Pose
from semantic_digital_twin.world_description.connections import DifferentialDrive


# %% arrival authorization
@pytest.fixture
def authorization() -> CabinTurnAuthorization:
    """
    A physically calibrated wall reference is reached with fresh stopped feedback.
    """
    geometry = EntryGeometry(Pose.from_xyz_rpy(), doorway_distance=0.86, distance=1.3)
    assessment = replace(
        geometry.assess(Pose.from_xyz_rpy(x=1.3)),
        wall_distance=0.936,
        wall_reference=0.936,
        wall_tolerance=0.05,
    )
    observation = EntryObservation(
        True, True, True, True, True, 1, fully_inside=True, target_reached=True
    )
    return CabinTurnAuthorization(True, observation, assessment)


def test_verified_longitudinal_arrival_allows_turn(
    authorization: CabinTurnAuthorization,
) -> None:
    """
    A successful stopped arrival inside the lidar band opens the turn gate.
    """
    assert authorization.allowed is True


@pytest.mark.parametrize(
    "change", ["failed", "moving", "stale", "navigation", "short", "missing", "nan"]
)
def test_unverified_entry_never_allows_turn(
    authorization: CabinTurnAuthorization, change: str
) -> None:
    """
    Failed, incomplete or unavailable entry evidence cannot authorize rotation.
    """
    if change == "failed":
        authorization = replace(authorization, entry_succeeded=False)
    elif change == "moving":
        authorization = replace(
            authorization,
            observation=replace(authorization.observation, stationary=False),
        )
    elif change == "stale":
        authorization = replace(
            authorization, observation=replace(authorization.observation, healthy=False)
        )
    elif change == "navigation":
        authorization = replace(
            authorization,
            observation=replace(authorization.observation, navigation_idle=False),
        )
    elif change == "short":
        authorization = replace(
            authorization,
            assessment=replace(authorization.assessment, wall_distance=1.12),
        )
    elif change == "missing":
        authorization = replace(authorization, assessment=None)
    else:
        authorization = replace(
            authorization,
            assessment=replace(authorization.assessment, wall_distance=math.nan),
        )
    assert authorization.allowed is False


def test_turn_is_opt_in_and_requires_lidar_reference() -> None:
    """
    The original validated entry profile gains no implicit new motion.
    """
    assert EntryOptions.parse(["--profile", "elevator"]).turn_after_entry is False
    assert (
        EntryOptions.parse(
            ["--profile", "elevator", "--turn-after-entry"]
        ).turn_after_entry
        is True
    )
    with pytest.raises(SystemExit):
        EntryOptions.parse(["--turn-after-entry"])
    with pytest.raises(SystemExit):
        EntryOptions.parse(
            ["--profile", "elevator", "--turn-after-entry", "--handover-only"]
        )


# %% native task composition
def test_turn_uses_native_orientation_zero_forward_and_measured_stop() -> None:
    """
    The CRAM payload requests pure rotation without a translating position task.
    """
    configuration = WorldWithRobocatConfig(
        urdf=Path(Robocat.get_ros_file_path()).read_text()
    )
    world = configuration.world
    with world.modify_world():
        configuration.setup_world()
        world.get_connections_by_type(DifferentialDrive)[
            0
        ].has_hardware_interface = True
    target = Pose.from_xyz_rpy(yaw=math.pi, reference_frame=world.root)
    plan = execute_single(
        RobocatCabinTurnMotion(target, orientation_tolerance_deg=5.0),
        context=Context(world=world, robot=configuration.robot),
    )
    executable = plan.parse()
    executable.prepare_for_execution()
    nodes = list(executable.motion_state_chart.nodes)
    for node in nodes:
        if isinstance(node, NodeListGoal):
            nodes.extend(node.nodes)
    orientations = [node for node in nodes if isinstance(node, CartesianOrientation)]
    assert len(orientations) == 1
    assert orientations[0].threshold == pytest.approx(math.radians(5.0))
    assert sum(isinstance(node, ZeroForwardVelocity) for node in nodes) == 1
    assert sum(isinstance(node, MeasuredBaseStop) for node in nodes) == 1
    assert sum(isinstance(node, CartesianPosition) for node in nodes) == 0
