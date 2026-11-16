"""
Check native-status transport using local subprocess pipes only.
"""

# %% imports
import subprocess
import sys
import time
from contextlib import ExitStack
from pathlib import Path
from unittest.mock import Mock

from experiments.robocat_snm.navigation import NavigationConnection
from experiments.robocat_snm.native_navigation import NavigationStatus
from semantic_digital_twin.spatial_types import Pose


# %% streaming ownership
def test_pipe_reports_correlated_success_and_closes_its_child() -> None:
    """
    Read status across pipe boundaries and close only the owned process.
    """
    connection = NavigationConnection(Pose.from_xyz_rpy())
    peer = Path(__file__).with_name("dataset") / "navigation_peer.py"
    with ExitStack() as cleanup:
        connection.process = subprocess.Popen(
            [sys.executable, str(peer)], stdin=subprocess.PIPE, stdout=subprocess.PIPE
        )
        cleanup.callback(connection.close)
        deadline = time.monotonic() + 2
        while not connection.poll() and time.monotonic() < deadline:
            time.sleep(0.02)
        assert connection.status is NavigationStatus.SUCCEEDED
        assert connection.idle is True
        assert connection.routing_ready is True
    assert connection.process.returncode == 0


def test_closed_transport_cannot_retain_idle_success() -> None:
    """
    A terminated native peer invalidates cached success and idle immediately.
    """
    connection = NavigationConnection(Pose.from_xyz_rpy())
    connection.process = Mock()
    connection.process.poll.return_value = 1
    connection.status = NavigationStatus.SUCCEEDED
    connection.idle = True
    assert connection.poll() is False
    assert connection.status is NavigationStatus.FAILED
    assert connection.idle is False
