#!/opt/ros/cram-env/bin/python
"""
Record temporary-session operations without contacting Docker.
"""

# %% imports
import json
import os
import signal
import sys
from pathlib import Path

# %% command recording
with Path(os.environ["ROBOCAT_WINDOW_LOG"]).open("a") as output:
    output.write(json.dumps(sys.argv[1:]) + "\n")
if "inspect" in sys.argv:
    print("true", flush=True)
elif "-it" in sys.argv:
    print("TEST TERMINAL READY", flush=True)
    while True:
        signal.pause()
elif "scripts/robocat_stop_test_session.sh" in sys.argv:
    raise SystemExit(int(os.environ.get("ROBOCAT_CLEANUP_RESULT", "0")))
