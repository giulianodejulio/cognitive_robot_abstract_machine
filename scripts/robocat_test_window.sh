#!/usr/bin/env bash
# %% temporary test windows
set -euo pipefail
role=${1:?Missing role}
run_directory=${2:?Missing log directory}
session_record=${3:?Missing session registration}
owner=$(basename "$run_directory")
repository=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$repository"
finish() {
    local status=$?
    trap - EXIT
    trap '' HUP TERM INT PIPE
    if [[ $role == snm ]]; then
        if docker exec -w /workspace/cram aicor-cram-demo \
            bash scripts/robocat_stop_test_session.sh "/tmp/$owner.snm.pid" "$owner" \
            >> "$run_directory/cleanup.log" 2>&1; then
            rm -f -- "$session_record"
        else
            touch "$run_directory/snm.cleanup_failed"
            status=1
            printf 'SNM cleanup unconfirmed. New test sessions remain blocked. Logs: %s/cleanup.log\n' "$run_directory" >> "$run_directory/window.log"
        fi
        cat "$run_directory/cleanup.log" >&2 2>/dev/null || true
    fi
    touch "$run_directory/$role.closed"
    printf '\nTest window %s stopped (%s); logs: %s\n' "$role" "$status" "$run_directory" >> "$run_directory/window.log"
    cat "$run_directory/window.log" >&2 2>/dev/null || true
    exit "$status"
}
trap finish EXIT
trap 'exit 129' HUP
trap 'exit 143' TERM
trap 'exit 130' INT
case "$role" in
    snm)
        feedback=wheel
        infra_directory=$(dirname "$(dirname "$session_record")")
        if [[ -f $infra_directory/feedback ]]; then read -r feedback < "$infra_directory/feedback"; fi
        docker exec -e "ROBOCAT_BYOBU_OWNER=$owner" -e "ROBOCAT_FEEDBACK=$feedback" -w /workspace/cram -it aicor-cram-demo \
            bash scripts/robocat_session_container.sh snm
        ;;
    robot)
        printf 'Opening native RViz. No movement goal is sent by this window.\n'
        printf -v command '%q ' bash -c "$(cat experiments/src/experiments/robocat_snm/rviz_remote.sh)"
        ssh -X -t robocat "$command"
        ssh -t robocat
        ;;
    *) printf 'Unsupported test role %s\n' "$role" >&2; exit 2 ;;
esac
