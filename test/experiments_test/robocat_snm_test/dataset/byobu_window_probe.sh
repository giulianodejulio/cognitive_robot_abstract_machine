#!/usr/bin/env bash
# %% harmless terminal lifecycle
set -euo pipefail
printf '%s\n' "$1" >> "$2/probe.log"
exec sleep 120
