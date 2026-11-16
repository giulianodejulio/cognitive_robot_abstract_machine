#!/opt/ros/cram-env/bin/python
"""
Provide recorded persistent-infrastructure metadata without tmux.
"""

# %% imports
import json
import os
import sys
from pathlib import Path

# %% recorded session state
arguments = sys.argv[1:]
with Path(os.environ["ROBOCAT_BYOBU_LOG"]).open("a") as output:
    output.write(json.dumps(arguments) + "\n")
if arguments[0] == "has-session":
    target = arguments[arguments.index("-t") + 1].lstrip("=")
    raise SystemExit(0 if target == "robocat-infra" else 1)
if arguments[0] == "show-option":
    print(os.environ["ROBOCAT_INFRA_DIRECTORY"])
if arguments[0] == "list-sessions":
    print("robocat-infra")
