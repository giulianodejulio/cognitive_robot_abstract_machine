"""
Explicit infrastructure pose selection without Docker or controller changes.
"""

# %% imports
import os
import shutil
import subprocess
from pathlib import Path

import pytest


# %% infrastructure command propagation
def test_infrastructure_accepts_laser_feedback(tmp_path: Path) -> None:
    """
    A laser startup passes its selected source to the scheduled server window.
    """
    root = Path(__file__).parents[3]
    binary = tmp_path / "bin"
    binary.mkdir()
    for name in ("byobu-tmux", "docker"):
        target = binary / name
        shutil.copy(
            Path(__file__).with_name("dataset") / "infra_metadata_tmux.sh", target
        )
        target.chmod(0o755)
    log = tmp_path / "commands.log"
    environment = {
        **os.environ,
        "PATH": str(binary) + os.pathsep + os.environ["PATH"],
        "ROBOCAT_METADATA_LOG": str(log),
    }
    result = subprocess.run(
        ["bash", str(root / "scripts/robocat-infra"), "start", "--feedback", "laser"],
        env=environment,
        capture_output=True,
        text=True,
        timeout=10,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    scheduled = [
        line
        for line in log.read_text().splitlines()
        if line.startswith("new-window") and "-n giskard " in line
    ]
    assert len(scheduled) == 1
    assert scheduled[0].rstrip().endswith(" laser")


@pytest.mark.parametrize("feedback, wheel_remap", [("wheel", True), ("laser", False)])
def test_container_server_remaps_only_wheel_feedback(
    tmp_path: Path, feedback: str, wheel_remap: bool
) -> None:
    """
    Laser mode leaves the laser topic intact and preserves the SNM command route.
    """
    root = Path(__file__).parents[3]
    scripts = tmp_path / "scripts"
    scripts.mkdir()
    setup = tmp_path / "setup.bash"
    setup.touch()
    recorder = tmp_path / "record-arguments"
    shutil.copy(Path(__file__).with_name("dataset") / "feedback_command.sh", recorder)
    recorder.chmod(0o755)
    launcher = scripts / "robocat_session_container.sh"
    launcher.write_text(
        (root / "scripts/robocat_session_container.sh")
        .read_text()
        .replace("/opt/ros/overlay_ws/install/setup.bash", str(setup))
        .replace("/opt/ros/cram-env/bin/python", str(recorder))
    )
    arguments = tmp_path / "arguments"
    environment = {**os.environ, "ROBOCAT_FEEDBACK_ARGUMENTS": str(arguments)}
    environment.pop("ROBOCAT_BYOBU_OWNER", None)
    result = subprocess.run(
        ["bash", str(launcher), "giskard", ".3", ".2", feedback],
        env=environment,
        capture_output=True,
        timeout=10,
    )
    assert result.returncode == 0
    recorded = arguments.read_bytes().decode().split("\0")
    assert "/cmd_vel:=/snm_vel" in recorded
    assert (
        "/direct_laser_odometry/odom:=/mobile_base_controller/odom" in recorded
    ) is wheel_remap
