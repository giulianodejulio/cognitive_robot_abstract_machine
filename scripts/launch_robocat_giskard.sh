#!/usr/bin/env bash
# %% arguments
set -eo pipefail
script_directory=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
repository=$(dirname "$script_directory")
linear=0.1
angular=0.2
feedback=laser
command_topic=/cmd_vel
verbose=false
while (($#)); do
    case "$1" in
        --help|-h)
            printf 'Usage: bash scripts/launch_robocat_giskard.sh [--feedback laser|wheel] [--max-linear M/S] [--max-angular RAD/S] [--command-topic /cmd_vel|/snm_vel] [--verbose]\n'
            printf 'Run inside aicor-cram-demo. Starts Robocat server and persistent client; sends no goal.\n'
            printf 'Defaults: 0.1 m/s, 0.2 rad/s. Separate terminal: ./scripts/robocat-goal DISTANCE\n'
            printf 'Feedback: laser (default) or wheel; applied to both server and client.\n'
            exit 0 ;;
        --max-linear|--max-angular)
            if (($# < 2)); then printf 'Missing value for %s\n' "$1" >&2; exit 2; fi
            if [[ $1 == --max-linear ]]; then linear=$2; else angular=$2; fi
            shift 2 ;;
        --verbose) verbose=true; shift ;;
        --command-topic)
            if (($# < 2)); then printf 'Missing value for --command-topic\n' >&2; exit 2; fi
            case "$2" in
                /cmd_vel|/snm_vel) command_topic=$2 ;;
                *) printf 'Invalid command topic: %s\n' "$2" >&2; exit 2 ;;
            esac
            shift 2 ;;
        --feedback)
            if (($# < 2)); then printf 'Missing value for --feedback\n' >&2; exit 2; fi
            case "$2" in
                laser|wheel) feedback=$2 ;;
                *) printf 'Invalid feedback: %s (expected laser or wheel)\n' "$2" >&2; exit 2 ;;
            esac
            shift 2 ;;
        *) printf 'Unknown option: %s\n' "$1" >&2; exit 2 ;;
    esac
done
feedback_arguments=()
if [[ $feedback == wheel ]]; then
    feedback_arguments=(--ros-args -r /direct_laser_odometry/odom:=/mobile_base_controller/odom)
fi
if [[ $command_topic != /cmd_vel ]]; then
    if ((${#feedback_arguments[@]} == 0)); then feedback_arguments=(--ros-args); fi
    feedback_arguments+=(-r "/cmd_vel:=$command_topic")
fi

# %% isolated ROS environment
source /opt/ros/overlay_ws/install/setup.bash
export ROS_DOMAIN_ID=103 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$repository/doc/robocat_cyclonedds.xml"
export PYTHONPATH="$repository:${PYTHONPATH:-}"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
set -u
cd "$repository"
exec 9>/tmp/robocat-giskard-launch.lock
flock -n 9 || { printf 'Another Robocat launcher is running\n' >&2; exit 1; }
logs=$(mktemp -d /tmp/robocat-giskard-launch.XXXXXX)
server_pid=
client_pid=
log_pid=
touch "$logs/server.log" "$logs/client.log"

# %% stop only owned processes
cleanup() {
    local result=$?
    trap - EXIT INT TERM
    for child in "$client_pid" "$server_pid"; do
        if [[ -n $child ]]; then kill -TERM "$child" 2>/dev/null || true; fi
    done
    local deadline=$((SECONDS + 15))
    for child in "$client_pid" "$server_pid"; do
        if [[ -z $child ]]; then continue; fi
        while kill -0 "$child" 2>/dev/null && ((SECONDS < deadline)); do sleep 0.1; done
        if kill -0 "$child" 2>/dev/null; then kill -KILL "$child" 2>/dev/null || true; fi
        wait "$child" 2>/dev/null || true
    done
    if [[ -n $log_pid ]]; then kill -TERM "$log_pid" 2>/dev/null || true; fi
    printf 'Logs retained in %s\n' "$logs"
    exit "$result"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

# %% readiness and persistent processes
wait_ready() {
    local child=$1 marker=$2 logfile=$3 deadline=$((SECONDS + 120))
    while ! grep -Fq "$marker" "$logfile"; do
        kill -0 "$child" 2>/dev/null || { printf 'Startup failed: %s\n' "$logfile" >&2; return 1; }
        if ((SECONDS >= deadline)); then printf 'Startup timeout: %s\n' "$logfile" >&2; return 1; fi
        sleep 0.1
    done
}
printf 'Logs: %s\n' "$logs"
printf 'Feedback: %s\n' "$feedback"
printf 'Command topic: %s\n' "$command_topic"
if [[ $verbose == true ]]; then
    tail --pid="$$" -q -n +1 -F "$logs/server.log" "$logs/client.log" 9>&- &
else
    tail --pid="$$" -q -n +1 -F "$logs/server.log" "$logs/client.log" 9>&- |
        awk -W interactive -f "$script_directory/tiago_giskard_log.awk" 9>&- &
fi
log_pid=$!
/opt/ros/cram-env/bin/python -u -m scripts.robocat_giskard_bringup \
    --max-linear "$linear" --max-angular "$angular" "${feedback_arguments[@]}" > "$logs/server.log" 2>&1 9>&- &
server_pid=$!
wait_ready "$server_pid" 'READY: /giskard/command' "$logs/server.log"
/opt/ros/cram-env/bin/python -u -m scripts.robocat_giskard_goal \
    --listen /tmp/robocat-giskard-goals.fifo "${feedback_arguments[@]}" > "$logs/client.log" 2>&1 9>&- &
client_pid=$!
wait_ready "$client_pid" 'READY CLIENT:' "$logs/client.log"
printf 'STACK READY. Another container terminal: ./scripts/robocat-goal DISTANCE\n'
printf 'Ctrl+C stops this server and client.\n'
wait -n "$server_pid" "$client_pid"
exit 1
