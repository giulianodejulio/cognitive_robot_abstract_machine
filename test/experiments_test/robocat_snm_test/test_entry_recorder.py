"""
The entry recorder selects a bounded topic set without sending goals.
"""

# %% imports
import os
import shutil
import subprocess
from pathlib import Path


# %% CLI-only recording
def test_entry_recorder_keeps_selected_topics_and_output_path(tmp_path: Path) -> None:
    """
    The recording wrapper neither records everything nor submits motion.
    """
    root = Path(__file__).parents[3]
    scripts = tmp_path / "scripts"
    scripts.mkdir()
    setup = tmp_path / "setup.bash"
    setup.touch()
    binary = tmp_path / "bin"
    binary.mkdir()
    recorder = binary / "ros2"
    shutil.copy(Path(__file__).with_name("dataset") / "record_command.sh", recorder)
    recorder.chmod(0o755)
    launcher = scripts / "robocat-record-entry"
    launcher.write_text(
        (root / "scripts/robocat-record-entry")
        .read_text()
        .replace("/opt/ros/overlay_ws/install/setup.bash", str(setup))
    )
    argument_file = tmp_path / "arguments"
    output = tmp_path / "bag with spaces"
    result = subprocess.run(
        ["bash", str(launcher), str(output)],
        env={
            **os.environ,
            "PATH": str(binary) + os.pathsep + os.environ["PATH"],
            "ROBOCAT_RECORD_ARGUMENTS": str(argument_file),
        },
        capture_output=True,
        text=True,
        timeout=5,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    arguments = argument_file.read_bytes().decode().split("\0")
    assert arguments[:2] == ["bag", "record"]
    assert arguments[arguments.index("-o") + 1] == str(output)
    assert "/snm/pose_comparison" in arguments
    assert "/direct_laser_odometry/odom" in arguments
    assert "/mobile_base_controller/odom" in arguments
    assert "/snm_vel" in arguments
    assert "/tf_static" in arguments
    assert "-a" not in arguments
    assert "--execute" not in arguments
