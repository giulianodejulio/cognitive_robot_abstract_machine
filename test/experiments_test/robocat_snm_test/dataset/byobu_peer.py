#!/opt/ros/cram-env/bin/python
"""
Record session commands without opening terminals or starting robot processes.
"""

# %% imports
import json
import os
import sys
from pathlib import Path

# %% command recording
if __name__ == "__main__":
    destination = Path(os.environ["ROBOCAT_BYOBU_LOG"])
    with destination.open("a") as output:
        output.write(json.dumps(sys.argv[1:]) + "\n")
    if sys.argv[1] == "has-session":
        raise SystemExit(0 if os.environ.get("ROBOCAT_BYOBU_EXISTS") == "1" else 1)
