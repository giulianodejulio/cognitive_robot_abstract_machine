# %% imports
from __future__ import annotations

import importlib
import math
import os
import select
import subprocess
from concurrent.futures import Future
from contextlib import ExitStack, nullcontext
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
from nav_msgs.msg import Odometry

from giskardpy.qp.enforcement_strategy import VelocityStrategy
from giskardpy.qp.qp_controller_config import QPControllerConfig
from krrood.adapters.json_serializer import from_json, to_json
from krrood.symbolic_math.symbolic_math import FloatVariable
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.world_description.degree_of_freedom import DegreeOfFreedom
from test.experiments_test.test_tiago_giskard_safety import client_script
from test.experiments_test.test_tiago_goal_command import command
from test.experiments_test.test_tiago_goal_foreground import request_pipe
from test.experiments_test.test_tiago_goal_sequence import sequence_script


# %% exact longitudinal velocity constraint without a robot
@pytest.fixture
def velocity_task():
    """
    Load the optional experimental task independently of the server entrypoint.
    """
    return importlib.import_module("experiments.tiago_pure_turn").ZeroForwardVelocity()


@pytest.fixture
def forward_motion():
    """
    Provide one symbolic forward degree of freedom without running a solver.
    """
    forward = DegreeOfFreedom(name=PrefixedName("forward"))
    forward.variables.position = FloatVariable("forward_position")
    world = Mock()
    world.get_connections_by_type.return_value = [SimpleNamespace(x_velocity=forward)]
    return SimpleNamespace(world=world, forward=forward)


def test_zero_velocity_has_zero_slack_and_positive_normalization(
    velocity_task, forward_motion
) -> None:
    """
    Zero speed is a hard velocity equality, not a large position weight.
    """
    artifacts = velocity_task.build_artifacts(forward_motion)
    (constraint,) = artifacts.constraints.equality_constraints
    assert constraint.enforcement_strategy is VelocityStrategy
    assert constraint.bound == 0
    assert constraint.lower_slack_limit == constraint.upper_slack_limit == 0
    assert constraint.normalization_factor == velocity_task.reference_velocity
    assert constraint.normalization_factor > 0
    assert constraint.expression is forward_motion.forward.variables.position


def test_zero_velocity_is_enforced_across_prediction_horizon(
    velocity_task, forward_motion
) -> None:
    """
    Every predicted velocity step receives zero bounds and zero slack.
    """
    constraints = velocity_task.build_artifacts(
        forward_motion
    ).constraints.equality_constraints
    configuration = QPControllerConfig(target_frequency=20)
    strategy = VelocityStrategy(
        degrees_of_freedom=[forward_motion.forward],
        constraints=constraints,
        qp_controller_config=configuration,
    )
    slack = strategy.create_slack_variables()
    assert len(slack.names) == configuration.control_horizon
    np.testing.assert_array_equal(
        slack.lower_bounds.to_np(), np.zeros(configuration.control_horizon)
    )
    np.testing.assert_array_equal(
        slack.upper_bounds.to_np(), np.zeros(configuration.control_horizon)
    )
    np.testing.assert_array_equal(
        strategy.create_equality_bounds().to_np(),
        np.zeros(configuration.control_horizon),
    )


def test_experimental_task_roundtrips_by_importable_module(velocity_task) -> None:
    """
    The server can deserialize the experiment without modifying native tasks.
    """
    restored = from_json(to_json(velocity_task))
    assert type(restored) is type(velocity_task)
    assert restored.reference_velocity == velocity_task.reference_velocity


def test_velocity_matrix_constrains_each_forward_step(
    velocity_task, forward_motion
) -> None:
    """
    The QP rows select longitudinal velocity at every control step.
    """
    configuration = QPControllerConfig(target_frequency=20)
    strategy = VelocityStrategy(
        degrees_of_freedom=[forward_motion.forward],
        constraints=velocity_task.build_artifacts(
            forward_motion
        ).constraints.equality_constraints,
        qp_controller_config=configuration,
    )
    matrix = strategy.create_matrix().to_np()
    steps = configuration.control_horizon
    np.testing.assert_array_equal(
        matrix[:, :steps],
        np.eye(steps) * configuration.control_time_step.total_seconds(),
    )
    np.testing.assert_array_equal(matrix[:, steps:], np.zeros_like(matrix[:, steps:]))


# %% chart and transport selection
def test_pure_chart_has_no_position_recovery(client_script: dict) -> None:
    """
    The rotation combines an angular target with zero forward velocity only.
    """
    motion = SimpleNamespace(
        world=SimpleNamespace(root=Mock()),
        robot=SimpleNamespace(root=Mock()),
        target=Mock(),
    )
    chart = client_script["PureTurnMotion"]._motion_chart.fget(motion)
    orientation, velocity = chart.nodes
    assert type(orientation) is client_script["CartesianOrientation"]
    assert type(velocity) is client_script["ZeroForwardVelocity"]
    assert orientation.threshold == math.radians(
        client_script["StraightBaseMotion"].tolerances.orientation_degrees
    )


def test_pure_turn_flag_reaches_the_client(
    command: Path, request_pipe, client_script: dict
) -> None:
    """
    Pure rotation is opt-in and travels with the owned turn request.
    """
    path, descriptor = request_pipe
    sender = subprocess.Popen(
        ["bash", str(command), "--turn", "30", "--pure-turn", "--fifo", str(path)],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    with ExitStack() as cleanup:
        cleanup.callback(sender.kill)
        assert select.select([descriptor], [], [], 3)[0] == [descriptor]
        request = client_script["GoalRequest"].parse(
            os.read(descriptor, 4096).decode().strip()
        )
        assert request.pure_turn is True
        assert request.distance == 0
        assert request.turn_degrees == 30
        request.publish("succeeded")
        sender.communicate(timeout=5)
        assert sender.returncode == 0


def test_pure_turn_rejects_forward_request(command: Path) -> None:
    """
    The optional velocity lock cannot silently affect a translation request.
    """
    result = subprocess.run(
        ["bash", str(command), "0.4", "--pure-turn"],
        capture_output=True,
        text=True,
        timeout=5,
    )
    assert result.returncode == 2


def test_session_selects_pure_motion(client_script: dict, monkeypatch) -> None:
    """
    The client sends the pure chart only for an explicitly selected turn.
    """
    session_type = client_script["GoalSession"]
    namespace = session_type.send.__globals__
    factory = Mock()
    monkeypatch.setitem(namespace, "execute_single", factory)
    monkeypatch.setitem(namespace, "real_robot", lambda **kwargs: nullcontext())
    monkeypatch.setattr(namespace["time"], "sleep", Mock())
    feedback = Mock()
    feedback.odometry = Odometry()
    feedback.odometry.pose.pose.orientation.w = 1.0
    completed = Future()
    completed.set_result(None)
    pool = Mock()
    pool.submit.return_value = completed
    session = session_type(context=Mock(), feedback=feedback, pool=pool)
    assert session.send(0, turn_degrees=30, pure_turn=True) is True
    assert type(factory.call_args.args[0]) is client_script["PureTurnMotion"]


# %% original sequence with an opt-in pure rotation
def test_sequence_changes_only_the_turn_request(
    sequence_script: dict, monkeypatch
) -> None:
    """
    Cancellation and both translations keep their existing behavior.
    """
    sequence = sequence_script["GoalSequence"].parse(
        [
            "--execute",
            "--first-distance",
            "0.5",
            "--cancel-after",
            "0.4",
            "--turn-degrees",
            "30",
            "--second-distance",
            "0.4",
            "--pure-turn",
        ]
    )
    commands = Mock(side_effect=[130, 0, 0, 0, 0])
    monkeypatch.setattr(sequence, "run_command", commands)
    sequence.run()
    assert [call.args[0] for call in commands.call_args_list] == [
        ["0.5", "--cancel-after", "0.4"],
        ["--ready"],
        ["--turn", "30", "--pure-turn"],
        ["--ready"],
        ["0.4"],
    ]
