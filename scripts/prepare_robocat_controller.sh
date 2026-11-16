#!/usr/bin/env bash
# %% explicit preparation mode
set -euo pipefail
mode=${1:---check}
if (($# > 2)) || [[ $mode != --check && $mode != --apply ]] || [[ ${2:-} != '' && ${2:-} != --confirm-navigation-idle ]]; then
    printf 'Usage on laptop host: bash scripts/prepare_robocat_controller.sh --check|--apply [--confirm-navigation-idle]\n' >&2
    exit 2
fi
repository=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
socket="$repository/.robocat-planner/ssh.sock"
if [[ ! -S $socket ]]; then printf 'Run bash scripts/connect_robocat_planner.sh first\n' >&2; exit 1; fi
remote_script=$(cat "$repository/experiments/src/experiments/robocat_snm/preparation_remote.sh")
arguments=("$mode")
if [[ ${2:-} == --confirm-navigation-idle && $mode == --apply ]]; then
    arguments+=(--confirm-navigation-idle)
fi
printf -v command '%q ' bash -c "$remote_script" bash "${arguments[@]}"
tar -cf - -C "$repository/experiments/src" \
    experiments/__init__.py experiments/_version.py experiments/robocat_snm/__init__.py \
    experiments/robocat_snm/native_navigation.py experiments/robocat_snm/native_preparation.py |
    ssh -S "$socket" -o BatchMode=yes -o StrictHostKeyChecking=yes -o ControlMaster=no \
        -o ConnectTimeout=3 pal@10.68.0.1 "$command" \
        2> >(sed -u -f "$repository/experiments/src/experiments/robocat_snm/preparation_log.sed" >&2) |
    sed -u -f "$repository/experiments/src/experiments/robocat_snm/preparation_log.sed"
