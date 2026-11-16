#!/usr/bin/env bash
# %% temporary session ownership
set -euo pipefail
pidfile=${1:?Missing session PID file}
owner=${2:?Missing session owner}
grace=${3:-20}
if [[ ! $owner =~ ^[[:alnum:]_.-]+$ || ! $grace =~ ^[1-9][0-9]*$ ]]; then exit 2; fi
if [[ ! -f $pidfile ]]; then
    printf 'SNM session registration missing; termination is not confirmed\n' >&2
    exit 1
fi
root=$(cat "$pidfile")
if [[ ! $root =~ ^[0-9]+$ || $root -le 1 ]]; then exit 1; fi
owns() {
    grep -zFxq -- "ROBOCAT_BYOBU_OWNER=$owner" "/proc/$1/environ" 2>/dev/null
}
if [[ -d /proc/$root ]] && ! owns "$root"; then
    printf 'Refusing SNM cleanup: session process owner mismatch\n' >&2
    exit 1
fi
owned_processes() {
    local directory process state
    for directory in /proc/[0-9]*; do
        process=${directory##*/}
        if ! owns "$process"; then continue; fi
        state=$(awk '{print $3}' "$directory/stat" 2>/dev/null) || continue
        if [[ $state != Z ]]; then printf '%s\n' "$process"; fi
    done
}

# %% cancel requests before closing their terminal
deadline=$((SECONDS + grace))
declare -A interrupted=()
while true; do
    requests=()
    while IFS= read -r process; do
        executable=$(readlink "/proc/$process/exe" 2>/dev/null) || continue
        if [[ ${executable##*/} == python* ]]; then requests+=("$process"); fi
    done < <(owned_processes)
    if ((${#requests[@]} == 0)); then break; fi
    for process in "${requests[@]}"; do
        if [[ -z ${interrupted[$process]:-} ]] && owns "$process"; then
            printf 'SNM: interrupting owned request PID %s for cancellation\n' "$process"
            kill -INT "$process" 2>/dev/null || true
            interrupted[$process]=true
        fi
    done
    if ((SECONDS >= deadline)); then
        printf 'SNM request termination NOT confirmed; inspect robot and cleanup log before retrying\n' >&2
        exit 1
    fi
    sleep .1
done

# %% close only the remaining owned shell and transport processes
while IFS= read -r process; do
    if owns "$process"; then kill -HUP "$process" 2>/dev/null || true; fi
done < <(owned_processes)
while [[ -n $(owned_processes) ]]; do
    if ((SECONDS >= deadline)); then
        printf 'SNM session processes remain; cleanup is not confirmed\n' >&2
        exit 1
    fi
    sleep .1
done
rm -- "$pidfile"
printf 'SNM session processes stopped; verify physical arrest separately. Infrastructure retained.\n'
