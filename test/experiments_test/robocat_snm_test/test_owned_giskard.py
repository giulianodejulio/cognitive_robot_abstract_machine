"""
Stop only a process owned by the closing Byobu session.
"""

# %% imports
import os
import subprocess
import sys
from contextlib import ExitStack
from pathlib import Path

import pytest


# %% owned process cleanup
@pytest.mark.parametrize("matching_owner", [True, False])
def test_session_cleanup_respects_process_owner(
    tmp_path: Path, matching_owner: bool
) -> None:
    """
    A correct token interrupts the owned process; a mismatch leaves it alone.
    """
    root = Path(__file__).parents[3]
    token = tmp_path.name
    environment = os.environ.copy()
    environment["ROBOCAT_BYOBU_OWNER"] = token
    with ExitStack() as cleanup:
        child = subprocess.Popen(
            [
                sys.executable,
                str(Path(__file__).with_name("dataset") / "owned_process.py"),
            ],
            env=environment,
            stdout=subprocess.PIPE,
            text=True,
        )
        cleanup.callback(child.wait, timeout=5)
        cleanup.callback(child.terminate)
        assert child.stdout.readline().strip() == "READY"
        pidfile = tmp_path / "giskard.pid"
        pidfile.write_text(str(child.pid))
        result = subprocess.run(
            [
                "bash",
                str(root / "scripts/robocat_stop_owned_process.sh"),
                str(pidfile),
                token if matching_owner else token + "-other",
            ],
            capture_output=True,
            text=True,
            timeout=8,
        )
        if matching_owner:
            assert result.returncode == 0, result.stderr
            assert child.wait(timeout=3) == 0
            assert child.stdout.read().strip() == "INTERRUPTED"
        else:
            assert result.returncode == 1
            assert child.poll() is None
