"""
Launch native RViz without submitting navigation goals.
"""

# %% imports
import os
import shutil
import subprocess
from pathlib import Path


# %% native visualization
def test_rviz_opens_saved_map_configuration(tmp_path: Path) -> None:
    """
    The native display uses the saved elevator RViz configuration.
    """
    root = Path(__file__).parents[3]
    dataset = Path(__file__).with_name("dataset")
    source = root / "experiments/src/experiments/robocat_snm/rviz_remote.sh"
    script = tmp_path / "rviz_remote.sh"
    script.write_text(
        source.read_text()
        .replace("/opt/ros/humble/setup.bash", str(dataset / "ros_setup_unset.bash"))
        .replace("/opt/pal/alum/setup.bash", str(dataset / "pal_setup_unset.bash"))
    )
    binary = tmp_path / "bin"
    binary.mkdir()
    command = binary / "ros2"
    shutil.copy(dataset / "rviz_command.sh", command)
    command.chmod(0o755)
    environment = os.environ.copy()
    environment["DISPLAY"] = ":99"
    environment["PATH"] = str(binary) + os.pathsep + environment["PATH"]
    result = subprocess.run(
        ["bash", str(script)],
        env=environment,
        capture_output=True,
        text=True,
        timeout=5,
    )
    assert result.returncode == 0, result.stderr
    assert result.stdout.splitlines() == [
        "run",
        "rviz2",
        "rviz2",
        "-d",
        "/home/pal/robocat_elevator_mapping_20261109/waypoints.rviz",
    ]
