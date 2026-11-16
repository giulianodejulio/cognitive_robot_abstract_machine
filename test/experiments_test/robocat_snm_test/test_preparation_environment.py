"""
Source native ROS/PAL setups with optional trace variables unset.
"""

# %% imports
import io
import os
import shutil
import subprocess
import tarfile
from pathlib import Path


# %% native preparation bootstrap
def test_optional_setup_variables_do_not_abort_read_only_preparation(
    tmp_path: Path,
) -> None:
    """
    Native environment loading reaches the check without requiring trace flags.
    """
    root = Path(__file__).parents[3]
    dataset = Path(__file__).with_name("dataset")
    source = root / "experiments/src/experiments/robocat_snm/preparation_remote.sh"
    script = tmp_path / "preparation_remote.sh"
    script.write_text(
        source.read_text()
        .replace("/opt/ros/humble/setup.bash", str(dataset / "ros_setup_unset.bash"))
        .replace("/opt/pal/alum/setup.bash", str(dataset / "pal_setup_unset.bash"))
    )
    binary = tmp_path / "bin"
    binary.mkdir()
    python = binary / "python3"
    shutil.copy(dataset / "preparation_python.sh", python)
    python.chmod(0o755)
    environment = os.environ.copy()
    environment.pop("AMENT_TRACE_SETUP_FILES", None)
    environment.pop("COLCON_TRACE", None)
    environment["PATH"] = str(binary) + os.pathsep + environment["PATH"]
    archive = io.BytesIO()
    with tarfile.open(fileobj=archive, mode="w"):
        pass
    result = subprocess.run(
        ["bash", str(script), "--check"],
        input=archive.getvalue(),
        capture_output=True,
        env=environment,
        timeout=5,
    )
    assert result.returncode == 0, result.stderr.decode()
    assert result.stdout.decode().splitlines() == [
        "-u",
        "-m",
        "experiments.robocat_snm.native_preparation",
        "--check",
    ]
