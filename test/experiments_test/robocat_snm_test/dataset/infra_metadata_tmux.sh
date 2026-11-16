#!/usr/bin/env bash
set -euo pipefail
printf '%s\n' "$*" >> "$ROBOCAT_METADATA_LOG"
if [[ $1 == has-session ]]; then exit 1; fi
if [[ $1 == set-option && $3 == =* ]]; then exit 1; fi
