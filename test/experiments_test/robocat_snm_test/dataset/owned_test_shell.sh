#!/usr/bin/env bash
# %% owned request under an interactive session surrogate
set -euo pipefail
/opt/ros/cram-env/bin/python "$(dirname "$0")/owned_process.py" &
wait
