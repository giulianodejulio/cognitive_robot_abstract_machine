#!/opt/ros/cram-env/bin/python
"""
A process stand-in for launcher lifecycle tests; it creates no ROS nodes.
"""

# %% imports
from __future__ import annotations

import os
import signal
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from types import FrameType


# %% child lifecycle
@dataclass
class LauncherChild:
    """
    Report readiness and record graceful termination for a fake child.
    """

    role: str
    """Whether this process stands in for the server or client."""
    events: Path
    """
    The test-owned directory for lifecycle events.
    """

    def terminate(self, signal_number: int, frame: FrameType | None) -> None:
        """
        Record termination before returning control to the launcher.
        """
        (self.events / f"{self.role}.stopped").write_text("terminated")
        raise SystemExit(0)

    def run(self) -> None:
        """
        Wait without doing any robot or network work.
        """
        signal.signal(signal.SIGTERM, self.terminate)
        print(
            "READY: /giskard/command" if self.role == "server" else "READY CLIENT:",
            flush=True,
        )
        while True:
            time.sleep(0.1)


if __name__ == "__main__":
    LauncherChild(
        "server" if "bringup" in sys.argv[2] else "client",
        Path(os.environ["TIAGO_TEST_EVENT_DIR"]),
    ).run()
