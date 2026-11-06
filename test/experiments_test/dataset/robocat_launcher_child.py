#!/opt/ros/cram-env/bin/python
"""
Expose launcher arguments without creating ROS nodes or sending commands.
"""

# %% imports
from __future__ import annotations

import json
import os
import sys
import time
from pathlib import Path

# %% persistent process substitute
if __name__ == "__main__":
    role = "server" if "scripts.robocat_giskard_bringup" in sys.argv else "client"
    Path(os.environ["ROBOCAT_TEST_ARGUMENTS"]).joinpath(f"{role}.json").write_text(
        json.dumps(sys.argv[1:])
    )
    print(
        "READY: /giskard/command" if role == "server" else "READY CLIENT:", flush=True
    )
    while True:
        time.sleep(1)
