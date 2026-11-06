# %% imports
from __future__ import annotations

import math
import os
import select
import subprocess
from concurrent.futures import Future
from contextlib import ExitStack, nullcontext
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from nav_msgs.msg import Odometry

from test.experiments_test.test_tiago_giskard_safety import client_script
from test.experiments_test.test_tiago_goal_command import command
from test.experiments_test.test_tiago_goal_foreground import request_pipe
from test.experiments_test.test_tiago_goal_sequence import sequence_script


# %% relative turn protocol without ROS actions
def test_turn_sender_and_parser(
    command: Path, request_pipe, client_script: dict
) -> None:
    """
    A turn carries degrees and ownership through the existing foreground protocol.
    """
    path, descriptor = request_pipe
    sender = subprocess.Popen(
        ["bash", str(command), "--turn", "30", "--fifo", str(path)],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    with ExitStack() as cleanup:
        cleanup.callback(sender.kill)
        assert select.select([descriptor], [], [], 3)[0] == [descriptor]
        line = os.read(descriptor, 4096).decode().strip()
        assert line.split()[0] == "turn"
        request = client_script["GoalRequest"].parse(line)
        assert request.turn_degrees == 30
        assert request.distance == 0
        request.publish("succeeded")
        sender.communicate(timeout=5)
        assert sender.returncode == 0


@pytest.mark.parametrize("angle", ["0", "nan", "inf", "181", "-181"])
def test_turn_rejects_invalid_angles(command: Path, angle: str) -> None:
    """
    Invalid angles must not reach a FIFO or initiate movement.
    """
    result = subprocess.run(
        ["bash", str(command), "--turn", angle],
        capture_output=True,
        text=True,
        timeout=5,
    )
    assert result.returncode == 2


# %% goal construction without a controller
def test_turn_chart_holds_position_and_uses_current_tolerances(
    client_script: dict,
) -> None:
    """
    A turn must hold the initial point rather than track a zero-length line.
    """
    tolerances = client_script["StraightBaseMotion"].tolerances
    motion = SimpleNamespace(
        world=SimpleNamespace(root=Mock()),
        robot=SimpleNamespace(root=Mock()),
        target=Mock(),
    )
    chart = client_script["InPlaceTurnMotion"]._motion_chart.fget(motion)
    position, orientation = chart.nodes
    assert type(position) is client_script["CartesianPosition"]
    assert position.threshold == tolerances.position
    assert orientation.threshold == math.radians(tolerances.orientation_degrees)
    assert position.goal_point is motion.target.position
    assert orientation.goal_orientation is motion.target.rotation_matrix


def test_turn_target_is_relative_to_measured_heading(
    client_script: dict, monkeypatch
) -> None:
    """
    A turn changes yaw while retaining the measured odometry position.
    """
    session_type = client_script["GoalSession"]
    namespace = session_type.send.__globals__
    factory = Mock()
    monkeypatch.setitem(namespace, "execute_single", factory)
    monkeypatch.setitem(namespace, "real_robot", lambda **kwargs: nullcontext())
    monkeypatch.setattr(namespace["time"], "sleep", Mock())
    feedback = Mock()
    feedback.odometry = Odometry()
    initial = feedback.odometry.pose.pose
    initial.position.x, initial.position.y = 2.0, -1.0
    initial.orientation.z = math.sin(math.radians(20) / 2)
    initial.orientation.w = math.cos(math.radians(20) / 2)
    execution = Future()
    execution.set_result(None)
    pool = Mock()
    pool.submit.return_value = execution
    session = session_type(context=Mock(), feedback=feedback, pool=pool)
    assert session.send(0, turn_degrees=30) is True
    motion = factory.call_args.args[0]
    assert type(motion) is client_script["InPlaceTurnMotion"]
    assert float(motion.target.x) == initial.position.x
    assert float(motion.target.y) == initial.position.y
    assert float(motion.target.yaw) == pytest.approx(math.radians(50))


# %% cancellation, turn and translation orchestration
def test_sequence_waits_after_turn_before_forward(
    sequence_script: dict, monkeypatch
) -> None:
    """
    The final translation starts only after a successful turn and stable rest.
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
        ]
    )
    commands = Mock(side_effect=[130, 0, 0, 3, 0, 0])
    monkeypatch.setattr(sequence, "run_command", commands)
    monkeypatch.setattr(sequence_script["time"], "sleep", Mock())
    sequence.run()
    assert [call.args[0] for call in commands.call_args_list] == [
        ["0.5", "--cancel-after", "0.4"],
        ["--ready"],
        ["--turn", "30"],
        ["--ready"],
        ["--ready"],
        ["0.4"],
    ]


def test_failed_turn_prevents_final_translation(
    sequence_script: dict, monkeypatch
) -> None:
    """
    A rejected, canceled or failed turn must end the sequence.
    """
    sequence = sequence_script["GoalSequence"](turn_degrees=30)
    commands = Mock(side_effect=[130, 0, 1])
    monkeypatch.setattr(sequence, "run_command", commands)
    with pytest.raises(RuntimeError):
        sequence.run()
    assert commands.call_count == 3
