"""
Native door action geometry, Cartesian mapping and monitor ownership.
"""

# %% imports
import math
from dataclasses import replace
from pathlib import Path
from unittest.mock import Mock

import pytest

from coraplex.datastructures.dataclasses import Context
from coraplex.datastructures.enums import ExecutionType
from coraplex.plans.executables import GiskardExecutable
from coraplex.plans.factories import execute_single
from coraplex.robot_plans.actions.core.navigation import PassDoorAction
from experiments.robocat_snm.application import EntryOptions, EntryMission
from experiments.robocat_snm import application
from experiments.robocat_snm.door_passage import (
    CalibratedDoorPassage,
    RobocatDoorMotion,
)
from experiments.robocat_snm.geometry import EntryGeometry
from experiments.robocat_snm.policy import EntryObservation, EntryPhase, EntryPolicy
from giskardpy.executor import Executor
from giskardpy.middleware.ros2.scripts.iai_robots.robocat import WorldWithRobocatConfig
from giskardpy.motion_statechart.context import MotionStatechartContext
from giskardpy.motion_statechart.monitors.sensor_monitors import PassageClear
from giskardpy.motion_statechart.tasks.cartesian_tasks import (
    CartesianPositionStraight,
    CartesianOrientation,
)
from semantic_digital_twin.robots.robocat import Robocat
from semantic_digital_twin.spatial_types import Pose
from semantic_digital_twin.world_description.connections import DifferentialDrive
from .test_application import mission


# %% native plan fixture
@pytest.fixture
def door_context(monkeypatch: pytest.MonkeyPatch) -> Context:
    """
    Build the acquired robot model without ROS publishers or control cycles.
    """
    monkeypatch.setattr(GiskardExecutable, "execution_type", ExecutionType.REAL)
    configuration = WorldWithRobocatConfig(
        urdf=Path(Robocat.get_ros_file_path()).read_text()
    )
    with configuration.world.modify_world():
        configuration.setup_world()
        configuration.world.get_connections_by_type(DifferentialDrive)[
            0
        ].has_hardware_interface = True
    return Context(
        world=configuration.world,
        robot=configuration.robot,
        alternative_motion_mappings=[RobocatDoorMotion],
    )


def test_calibration_reuses_native_action_and_global_targets(
    door_context: Context,
) -> None:
    """
    A rotated doorway preserves staging and inside poses in server-known frames.
    """
    geometry = EntryGeometry(
        Pose.from_xyz_rpy(x=2, y=3, yaw=math.pi / 2),
        doorway_distance=0.86,
        distance=1.3,
    )
    action = CalibratedDoorPassage.from_geometry(geometry, door_context)
    execute_single(action, context=door_context)
    assert isinstance(action, PassDoorAction)
    approach = action._pose_at(-action.approach_distance, 1)
    inside = action._pose_at(action.exit_distance, 1)
    assert approach.position.to_np()[:2] == pytest.approx(
        geometry.staging.position.to_np()[:2]
    )
    assert inside.position.to_np()[:2] == pytest.approx(
        geometry.target.position.to_np()[:2]
    )
    assert inside.reference_frame is door_context.world.root
    monitor = action._passage_clear()
    assert isinstance(monitor, PassageClear)
    assert monitor.passage_start.reference_frame is door_context.world.root
    assert monitor.passage_end.reference_frame is door_context.world.root
    assert monitor.end_condition is monitor.observation_variable


def test_native_action_compiles_without_running_robot(door_context: Context) -> None:
    """
    Both native navigation stages retain straight motion and configured tolerances.
    """
    geometry = EntryGeometry(Pose.from_xyz_rpy(), doorway_distance=0.86, distance=1.3)
    action = CalibratedDoorPassage.from_geometry(geometry, door_context)
    plan = execute_single(action, context=door_context)
    plan.notify()
    executable = plan.parse()
    executable.prepare_for_execution()
    executor = Executor(context=MotionStatechartContext(world=door_context.world))
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
    assert len(positions) == 2
    assert len(orientations) == 2
    assert [node.threshold for node in positions] == [geometry.position_tolerance] * 2
    assert [float(node.threshold) for node in orientations] == pytest.approx(
        [geometry.orientation_tolerance] * 2
    )
    assert executor.control_cycles == 0


# %% independent continuous protection
def test_owned_native_gate_does_not_cancel_on_door_plane_after_commitment() -> None:
    """
    The native latched monitor owns the doorway after entry was authorized.
    """
    policy = EntryPolicy(stability_seconds=0, minimum_scans=1)
    clear = EntryObservation(True, True, True, True, True, 1)
    policy.update(clear, 0)
    assert policy.phase is EntryPhase.ENTERING
    policy.update(
        replace(clear, stationary=False, door_open=False, native_doorway_monitor=True),
        1,
    )
    assert policy.phase is EntryPhase.ENTERING
    policy.update(
        replace(
            clear, stationary=False, passage_clear=False, native_doorway_monitor=True
        ),
        2,
    )
    assert policy.phase is EntryPhase.STOPPING


def test_native_door_mode_is_an_explicit_comparison_option() -> None:
    """
    The existing command keeps the baseline until the native action is selected.
    """
    assert EntryOptions.parse([]).pass_door_action is False
    assert EntryOptions.parse(["--pass-door-action"]).pass_door_action is True


# %% request integration
def test_native_mode_submits_the_native_action(
    mission: EntryMission, monkeypatch: pytest.MonkeyPatch
) -> None:
    """
    The comparison flag selects the door action rather than the baseline motion.
    """
    mission.options.pass_door_action = True
    native_action = Mock()
    factory = Mock(return_value=native_action)
    monkeypatch.setattr(application.CalibratedDoorPassage, "from_geometry", factory)
    geometry = EntryGeometry(Pose.from_xyz_rpy(), doorway_distance=0.86, distance=1.3)
    assert mission.run(geometry) is EntryPhase.SUCCEEDED
    factory.assert_called_once_with(
        mission.inputs.frozen_geometry, mission.inputs.context
    )
    application.execute_single.assert_called_once_with(
        native_action, context=mission.inputs.context
    )
    application.RobocatStraightMotion.assert_not_called()
