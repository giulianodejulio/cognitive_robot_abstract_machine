"""
Verify the Robocat sequence uses its own client without duplicating control.
"""

# %% imports
from __future__ import annotations

from pathlib import Path
from unittest.mock import Mock

import pytest

from scripts.robocat_goal_sequence import RobocatGoalSequence
from scripts.tiago_goal_sequence import GoalSequence


# %% dedicated endpoint and shared orchestration
def test_robocat_sequence_uses_robocat_client() -> None:
    """
    Select the Robocat FIFO and sender when parsing ordinary sequence options.
    """
    sequence = RobocatGoalSequence.parse(["--execute"])
    assert sequence.fifo == Path("/tmp/robocat-giskard-goals.fifo")
    assert sequence.sender == Path(__file__).parents[2] / "scripts/robocat-goal"
    assert RobocatGoalSequence.run is GoalSequence.run


def test_robocat_sequence_waits_for_rest_before_second_goal(monkeypatch) -> None:
    """
    Preserve cancellation confirmation and measured rest for Robocat.
    """
    sequence = RobocatGoalSequence.parse(
        ["--execute", "--first-distance", "0.4", "--second-distance", "0.4"]
    )
    commands = Mock(side_effect=[130, 0, 0])
    monkeypatch.setattr(sequence, "run_command", commands)
    sequence.run()
    assert [call.args[0] for call in commands.call_args_list] == [
        ["0.4", "--cancel-after", str(sequence.cancel_after)],
        ["--ready"],
        ["0.4"],
    ]


def test_robocat_sequence_requires_explicit_execution() -> None:
    """
    Parsing without execution authorization cannot start the sequence.
    """
    with pytest.raises(SystemExit) as error:
        RobocatGoalSequence.parse([])
    assert error.value.code == 2
