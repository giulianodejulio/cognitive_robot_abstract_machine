"""
Retain evidence when a request does not finish cancellation.
"""

# %% imports
import os
import subprocess
import sys
from contextlib import ExitStack
from pathlib import Path


# %% unacknowledged interrupt
def test_unfinished_request_is_not_reported_as_stopped(tmp_path: Path) -> None:
    """
    A cancellation timeout preserves registration and reports failure.
    """
    root = Path(__file__).parents[3]
    owner = tmp_path.name
    environment = os.environ.copy()
    environment["ROBOCAT_BYOBU_OWNER"] = owner
    with ExitStack() as cleanup:
        child = subprocess.Popen(
            [
                sys.executable,
                str(Path(__file__).with_name("dataset") / "stubborn_test_request.py"),
            ],
            env=environment,
            stdout=subprocess.PIPE,
            text=True,
        )
        cleanup.callback(child.wait, timeout=5)
        cleanup.callback(child.terminate)
        assert child.stdout.readline().strip() == "READY"
        pidfile = tmp_path / "snm.pid"
        pidfile.write_text(str(child.pid))
        result = subprocess.run(
            [
                "bash",
                str(root / "scripts/robocat_stop_test_session.sh"),
                str(pidfile),
                owner,
                "1",
            ],
            capture_output=True,
            text=True,
            timeout=5,
        )
        assert result.returncode == 1
        assert child.poll() is None
        assert pidfile.exists() is True
