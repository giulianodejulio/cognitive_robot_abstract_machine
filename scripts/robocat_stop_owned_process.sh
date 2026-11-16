#!/usr/bin/env bash
# %% verified process ownership
set -euo pipefail
pidfile=${1:?Missing PID file}
owner=${2:?Missing session owner}
if [[ ! -f $pidfile ]]; then exit 0; fi
process=$(cat "$pidfile")
if [[ ! $process =~ ^[0-9]+$ || $process -le 1 ]]; then exit 1; fi
if [[ ! -d /proc/$process ]]; then rm -- "$pidfile"; exit 0; fi
if ! grep -zFxq -- "ROBOCAT_BYOBU_OWNER=$owner" "/proc/$process/environ"; then
    printf 'Refusing to stop process %s: session owner mismatch\n' "$process" >&2
    exit 1
fi
kill -INT "$process"
deadline=$((SECONDS + 5))
while [[ -d /proc/$process ]]; do
    if [[ $(awk '{print $3}' "/proc/$process/stat" 2>/dev/null) == Z ]]; then break; fi
    if ((SECONDS >= deadline)); then
        printf 'Owned process %s has not stopped; inspect before restarting\n' "$process" >&2
        exit 1
    fi
    sleep .1
done
rm -- "$pidfile"
