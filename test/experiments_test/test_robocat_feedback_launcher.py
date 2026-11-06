"""
Verify feedback selection reaches both persistent processes without hardware.
"""

# %% imports
from __future__ import annotations

import json
import os
import shutil
import subprocess
import time
from pathlib import Path

import pytest


# %% launcher arguments
@pytest.mark.parametrize("feedback", [None, "laser", "wheel"])
@pytest.mark.parametrize("command_topic", [None, "/snm_vel"])
def test_feedback_selection_reaches_both_processes(
    tmp_path: Path, feedback: str | None, command_topic: str | None
) -> None:
    """
    Apply one consistent pose source to the server and persistent client.
    """
    root = Path(__file__).parents[2]
    scripts = tmp_path / "scripts"
    scripts.mkdir()
    child = scripts / "child.py"
    shutil.copy(root / "test/experiments_test/dataset/robocat_launcher_child.py", child)
    child.chmod(0o755)
    setup = tmp_path / "setup.bash"
    setup.touch()
    launcher = scripts / "launch_robocat_giskard.sh"
    launcher.write_text(
        (root / "scripts/launch_robocat_giskard.sh")
        .read_text()
        .replace("/opt/ros/overlay_ws/install/setup.bash", str(setup))
        .replace("/opt/ros/cram-env/bin/python", str(child))
    )
    shutil.copy(root / "scripts/tiago_giskard_log.awk", scripts)
    environment = os.environ.copy()
    environment["ROBOCAT_TEST_ARGUMENTS"] = str(tmp_path)
    arguments = [] if feedback is None else ["--feedback", feedback]
    if command_topic is not None:
        arguments.extend(["--command-topic", command_topic])
    with (tmp_path / "launcher.log").open("w") as output:
        process = subprocess.Popen(
            ["bash", str(launcher), *arguments],
            stdout=output,
            stderr=subprocess.STDOUT,
            env=environment,
        )
        deadline = time.monotonic() + 5
        while (
            process.poll() is None
            and not (tmp_path / "client.json").exists()
            and time.monotonic() < deadline
        ):
            time.sleep(0.02)
        if process.poll() is None:
            process.terminate()
        process.wait(timeout=20)
    assert (tmp_path / "server.json").is_file()
    assert (tmp_path / "client.json").is_file()
    expected = (
        [
            "--ros-args",
            "-r",
            "/direct_laser_odometry/odom:=/mobile_base_controller/odom",
        ]
        if feedback == "wheel"
        else []
    )
    if command_topic is not None:
        if not expected:
            expected.append("--ros-args")
        expected.extend(["-r", f"/cmd_vel:={command_topic}"])
    for role in ("server", "client"):
        arguments = json.loads((tmp_path / f"{role}.json").read_text())
        remapping = (
            arguments[arguments.index("--ros-args") :]
            if "--ros-args" in arguments
            else []
        )
        assert remapping == expected


def test_invalid_feedback_is_rejected_before_startup() -> None:
    """
    Reject unknown feedback without sourcing ROS or starting children.
    """
    launcher = Path(__file__).parents[2] / "scripts/launch_robocat_giskard.sh"
    result = subprocess.run(
        ["bash", str(launcher), "--feedback", "unknown"],
        capture_output=True,
        text=True,
        timeout=5,
    )
    assert result.returncode == 2


def test_help_exposes_feedback_selection() -> None:
    """
    Make the supported pose sources discoverable from the launcher CLI.
    """
    launcher = Path(__file__).parents[2] / "scripts/launch_robocat_giskard.sh"
    result = subprocess.run(
        ["bash", str(launcher), "--help"], capture_output=True, text=True, timeout=5
    )
    assert result.returncode == 0
    assert "--feedback" in result.stdout
    assert "wheel" in result.stdout
    assert "laser" in result.stdout
