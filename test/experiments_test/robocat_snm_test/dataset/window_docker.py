#!/opt/ros/cram-env/bin/python
"""
Record Docker window commands without contacting a daemon.
"""

# %% imports
import json
import os
import signal
import sys
from pathlib import Path

# %% attached window lifecycle
with Path(os.environ["ROBOCAT_WINDOW_LOG"]).open("a") as output:
    output.write(json.dumps(sys.argv[1:]) + "\n")
if "-it" in sys.argv:
    print("READY: /giskard/command", flush=True)
    while True:
        signal.pause()
