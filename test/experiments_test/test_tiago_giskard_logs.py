# %% imports
from __future__ import annotations

import subprocess
from pathlib import Path
from unittest.mock import Mock

from std_msgs.msg import Bool

from test.experiments_test.test_tiago_giskard_safety import client_script, feedback


# %% compact terminal output
def test_terminal_filter_preserves_results_and_errors() -> None:
    """
    Suppress routine startup noise while retaining motion outcomes and faults.
    """
    script = Path(__file__).parents[2] / "scripts/tiago_giskard_log.awk"
    lines = [
        'Unknown tag "material" in /robot/link',
        "[INFO] [123] [giskard]: Subscribed to /scan",
        "WARNING:giskard:Model reload synchronization not available because database URI is not set.",
        "READY: /giskard/command",
        "GOALS BLOCKED: teleop",
        "SENDING: 0.3 m forward",
        "WHEEL ONSET: 0.5 s",
        "SUCCESS; final odometry: forward=0.3 m",
        "REQUEST TO WHEEL FEEDBACK: 0.5 s",
        "[ERROR] [123] [giskard]: Goal aborted",
        "RuntimeError: connection lost",
    ]
    result = subprocess.run(
        ["awk", "-f", str(script)],
        input="\n".join(lines) + "\n",
        text=True,
        capture_output=True,
        timeout=5,
    )
    assert result.returncode == 0
    assert result.stdout.splitlines() == [
        lines[3],
        lines[4],
        lines[5],
        lines[6],
        lines[7],
        lines[9],
        lines[10],
    ]


def test_mux_detail_changes_do_not_repeat_same_operator_block(
    client_script: dict, feedback, capsys
) -> None:
    """
    Changing redundant diagnostic fields must not repeat an unchanged teleop state.
    """
    feedback.receive_joy(Bool(data=True))
    session = client_script["GoalSession"](
        context=Mock(), feedback=feedback, pool=Mock()
    )
    session.report_status()
    first = capsys.readouterr().out
    feedback.mux.update(
        {"current priority": "100", "lock locks.joystick": " locked (priority #100)"}
    )
    session.report_status()
    assert capsys.readouterr().out == ""
    assert "teleop" in first
