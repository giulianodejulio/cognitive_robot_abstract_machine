#!/usr/bin/env bash
# %% owned persistent infrastructure window
set -euo pipefail
role=${1:?Missing window role}
run_directory=${2:?Missing run directory}
linear=${3:?Missing linear limit}
angular=${4:?Missing angular limit}
feedback=${5:-wheel}
if [[ $# -lt 5 && -f $run_directory/feedback ]]; then
    read -r feedback < "$run_directory/feedback"
fi
repository=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$repository"
container=aicor-cram-demo
watcher=
owner=$(basename "$run_directory")
interrupted=false
finish() {
    local result=$?
    trap - EXIT INT TERM HUP
    if [[ -n $watcher ]]; then kill "$watcher" 2>/dev/null || true; fi
    if [[ $role == giskard ]]; then
        if ! docker exec -w /workspace/cram "$container" bash scripts/robocat_stop_owned_process.sh \
            "/tmp/$owner.giskard.pid" "$owner"; then
            printf 'Giskard cleanup incomplete; inspect the owned server before restarting.\n' >&2
            result=1
        fi
    fi
    if [[ $role == prepare && $result != 0 && -f $run_directory/clock.ready ]]; then
        if ! bash "$repository/scripts/robocat-clock" stop; then
            printf 'Clock restore incomplete: run bash scripts/robocat-clock stop after stopping infra.\n' >&2
        fi
    fi
    touch "$run_directory/$role.closed"
    if ((result != 0)); then touch "$run_directory/$role.failed"; fi
    printf '\nInfrastructure window %s stopped (exit %s). Logs: %s\n' "$role" "$result" "$run_directory"
    if [[ $interrupted == false && -t 0 ]]; then exec bash --noprofile --norc -i; fi
    exit "$result"
}
trap finish EXIT
trap 'interrupted=true; exit 129' HUP
trap 'interrupted=true; exit 143' TERM
trap 'interrupted=true; exit 130' INT
wait_ready() {
    local dependency=$1 deadline=$((SECONDS + ${2:-900}))
    printf 'Waiting for %s...\n' "$dependency"
    while [[ ! -f $run_directory/$dependency.ready ]]; do
        if [[ -f $run_directory/$dependency.failed || -f $run_directory/$dependency.closed ]]; then
            printf '%s failed/stopped; inspect that window.\n' "$dependency" >&2; return 1
        fi
        if ((SECONDS >= deadline)); then printf 'Timeout waiting for %s\n' "$dependency" >&2; return 1; fi
        sleep .2
    done
}
case "$role" in
    prepare)
        command -v docker >/dev/null
        command -v ssh >/dev/null
        state=$(docker inspect "$container" --format '{{.State.Status}}')
        if [[ $state != running ]]; then
            command_line=$(docker inspect "$container" --format '{{json .Config.Cmd}}')
            if [[ $command_line != '["sleep","infinity"]' ]]; then
                printf 'Container stopped and not using expected idle command; start it manually.\n' >&2; exit 1
            fi
            docker start "$container"
        fi
        if docker inspect robocat-dds-router --format '{{.State.Running}}' 2>/dev/null | grep -qx true; then
            printf 'DDS bridge already running outside new infrastructure session; inspect/stop it first.\n' >&2; exit 1
        fi
        docker exec -w /workspace/cram "$container" bash scripts/robocat_session_container.sh guard
        bash scripts/connect_robocat_planner.sh
        bash scripts/prepare_robocat_controller.sh --check | tee "$run_directory/prepare.log"
        printf '\nAPPLY will deactivate, clean up, set 2.0 / 2.0 / 1.22, configure and activate base controller.\n'
        printf 'Confirm robot stopped, no active goals, teleop OFF, charger disconnected, operator and E-stop ready.\n'
        read -r -p 'Type APPLY to continue, anything else to stop: ' confirmation
        if [[ $confirmation != APPLY ]]; then printf 'Preparation canceled; dependent processes remain stopped.\n'; exit 1; fi
        bash scripts/robocat-clock start
        touch "$run_directory/clock.ready"
        docker exec -w /workspace/cram "$container" bash scripts/robocat_session_container.sh guard
        bash scripts/prepare_robocat_controller.sh --apply --confirm-navigation-idle | tee -a "$run_directory/prepare.log"
        touch "$run_directory/prepare.ready"
        printf '\nPreparation verified; infrastructure dependent windows may start.\n'
        ;;
    dds)
        wait_ready prepare
        (
            deadline=$((SECONDS + 30))
            while ((SECONDS < deadline)); do
                if docker inspect robocat-dds-router --format '{{.State.Running}}' 2>/dev/null | grep -qx true; then
                    label=$(docker inspect robocat-dds-router --format '{{index .Config.Labels "org.robocat.infra.owner"}}' 2>/dev/null || true)
                    if [[ $label == "$owner" ]]; then
                        touch "$run_directory/dds.ready"; exit 0
                    fi
                fi
                sleep .2
            done
            touch "$run_directory/dds.failed"
        ) &
        watcher=$!
        ROBOCAT_INFRA_OWNER="$owner" bash scripts/launch_robocat_dds_router.sh --control --command-topic /snm_vel 2>&1 |
            sed -u -f "$repository/experiments/src/experiments/robocat_snm/preparation_log.sed" |
            tee "$run_directory/dds.log"
        ;;
    giskard)
        wait_ready prepare
        wait_ready dds 60
        touch "$run_directory/giskard.log"
        (
            deadline=$((SECONDS + 120))
            while ! grep -Fq 'READY: /giskard/command' "$run_directory/giskard.log"; do
                if ((SECONDS >= deadline)); then touch "$run_directory/giskard.failed"; exit 1; fi
                sleep .2
            done
            touch "$run_directory/giskard.ready"
        ) &
        watcher=$!
        docker exec -e "ROBOCAT_BYOBU_OWNER=$owner" -w /workspace/cram -it "$container" bash scripts/robocat_session_container.sh giskard "$linear" "$angular" "$feedback" 2>&1 |
            sed -u -f "$repository/experiments/src/experiments/robocat_snm/preparation_log.sed" |
            tee "$run_directory/giskard.log"
        ;;
    *) printf 'Unsupported persistent infrastructure role: %s\n' "$role" >&2; exit 2 ;;
esac
