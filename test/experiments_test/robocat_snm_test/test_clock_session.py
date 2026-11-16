"""
Temporary laptop time sources must be restored after tests and failed startup.
"""

# %% imports
import os
import shutil
import subprocess
from pathlib import Path

import pytest


# %% clock lifecycle
@pytest.mark.parametrize("synchronized", [True, False])
def test_clock_override_lifetime(tmp_path: Path, synchronized: bool) -> None:
    """
    Only a verified local NTP source may retain the runtime override.
    """
    root = Path(__file__).parents[3]
    script = tmp_path / "scripts/robocat-clock"
    script.parent.mkdir()
    runtime = tmp_path / "runtime"
    script.write_text(
        (root / "scripts/robocat-clock")
        .read_text()
        .replace("/run/systemd/timesyncd.conf.d", str(runtime))
        .replace("attempt < 60", "attempt < 1")
    )
    configuration = tmp_path / "config/robocat_clock"
    configuration.mkdir(parents=True)
    shutil.copy(root / "config/robocat_clock/90-robocat-client.conf", configuration)
    binary = tmp_path / "bin"
    binary.mkdir()
    for command in ("systemctl", "timedatectl"):
        target = binary / command
        shutil.copy(Path(__file__).with_name("dataset") / "clock_peer.sh", target)
        target.chmod(0o755)
    environment = {
        **os.environ,
        "PATH": str(binary) + os.pathsep + os.environ["PATH"],
        "CLOCK_SYNCHRONIZED": str(int(synchronized)),
    }
    result = subprocess.run(
        ["bash", str(script), "apply"],
        env=environment,
        capture_output=True,
        text=True,
        timeout=10,
    )
    override = runtime / "90-robocat-client.conf"
    assert result.returncode == (0 if synchronized else 1), result.stderr
    assert override.exists() is synchronized
    if synchronized:
        stopped = subprocess.run(
            ["bash", str(script), "restore"],
            env=environment,
            capture_output=True,
            text=True,
            timeout=10,
        )
        assert stopped.returncode == 0, stopped.stderr
        assert not override.exists()
