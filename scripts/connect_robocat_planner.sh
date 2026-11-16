#!/usr/bin/env bash
# %% usage
set -euo pipefail
if [[ ${1:-} == --help ]]; then
    printf 'Usage: bash scripts/connect_robocat_planner.sh [--stop]\n'
    printf 'Run on the laptop host. Share its authenticated Robocat SSH connection with the CRAM planner client.\n'
    printf 'No credentials are copied and no motion goal is sent.\n'
    exit 0
fi
if (($# > 1)) || [[ ${1:-} != '' && ${1:-} != --stop ]]; then
    printf 'Unknown argument; use --help\n' >&2
    exit 2
fi

# %% one dedicated SSH connection
repository=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
connection_directory="$repository/.robocat-planner"
mkdir -p "$connection_directory"
chmod 700 "$connection_directory"
exec 9>"$connection_directory/lock"
flock -x 9
socket="$connection_directory/ssh.sock"
if [[ ${1:-} == --stop ]]; then
    if [[ -S $socket ]]; then ssh -S "$socket" -O exit robocat; fi
    exit 0
fi
if ssh -S "$socket" -O check robocat 2>/dev/null; then
    printf 'Planner SSH connection already ready.\n'
    exit 0
fi
if [[ -S $socket ]]; then rm -- "$socket"; fi
ssh -M -N -f -o BatchMode=yes -o ConnectTimeout=5 \
    -o ServerAliveInterval=3 -o ServerAliveCountMax=2 \
    -o ControlPersist=yes -o "ControlPath=$socket" robocat 9>&-
ssh -S "$socket" -O check robocat
printf 'Planner SSH ready. Container preview can request ComputePathToPose.\n'
