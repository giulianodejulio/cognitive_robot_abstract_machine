#!/usr/bin/env bash
# %% arguments
set -eo pipefail
script_directory=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
repository=$(dirname -- "$script_directory")
fifo=/tmp/tiago-giskard-goals.fifo
log_directory=
startup_timeout=90
verbose=false
python=${TIAGO_PYTHON:-/opt/ros/cram-env/bin/python}
while (($#)); do
    case "$1" in
        --help|-h)
            cat <<'HELP'
Usage: bash scripts/launch_tiago_giskard.sh [options]
Run inside the configured TIAGo container, after Ethernet is connected.
Starts a persistent Giskard server and goal client; sends no motion goal.

  --fifo PATH                 Request FIFO (default: /tmp/tiago-giskard-goals.fifo)
  --log-dir PATH              Directory for separate server/client logs
  --startup-timeout SECONDS   Readiness timeout per process (default: 90)
  --verbose                   Show full unfiltered logs in the terminal

After STACK READY, use another terminal in the same container:
  ./scripts/tiago-goal 0.30
Ctrl+C stops both child processes. Stop previous manually started instances first.
HELP
            exit 0 ;;
        --verbose) verbose=true; shift ;;
        --fifo|--log-dir|--startup-timeout)
            if (($# < 2)); then echo "Missing value for $1" >&2; exit 2; fi
            case "$1" in
                --fifo) fifo=$2 ;;
                --log-dir) log_directory=$2 ;;
                --startup-timeout) startup_timeout=$2 ;;
            esac
            shift 2 ;;
        *) echo "Unknown option: $1 (use --help)" >&2; exit 2 ;;
    esac
done
if [[ ! $startup_timeout =~ ^[1-9][0-9]*$ ]]; then
    echo "--startup-timeout must be a positive integer" >&2
    exit 2
fi
fifo=$(realpath -m -- "$fifo")

# %% shared ROS environment
test -x "$python" || { echo "Missing Python: $python" >&2; exit 1; }
test -r /opt/ros/overlay_ws/install/setup.bash || { echo "Run this launcher inside aicor-cram-demo" >&2; exit 1; }
source /opt/ros/overlay_ws/install/setup.bash
export ROS_DOMAIN_ID=72
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$repository/doc/tiago_cyclonedds.xml"
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
set -u
cd "$repository"
test -r doc/tiago_cyclonedds.xml
command -v flock >/dev/null
exec 9>/tmp/tiago-giskard-launch.lock
flock -n 9 || { echo "Another launcher is already running" >&2; exit 1; }
if [[ -z $log_directory ]]; then
    log_directory=$(mktemp -d /tmp/tiago-giskard-launch.XXXXXX)
else
    mkdir -p -- "$log_directory"
    log_directory=$(realpath -- "$log_directory")
fi
server_log=$log_directory/server.log
client_log=$log_directory/client.log
: > "$server_log"
: > "$client_log"
server_pid=
client_pid=
log_pid=

# %% owned child shutdown
cleanup() {
    local result=$?
    trap - EXIT INT TERM
    echo "Stopping launcher-owned client and server..."
    for child in "$client_pid" "$server_pid"; do
        if [[ -n $child ]]; then kill -TERM "$child" 2>/dev/null || true; fi
    done
    local deadline=$((SECONDS + 15))
    for child in "$client_pid" "$server_pid"; do
        if [[ -z $child ]]; then continue; fi
        while kill -0 "$child" 2>/dev/null && ((SECONDS < deadline)); do sleep 0.1; done
        if kill -0 "$child" 2>/dev/null; then
            echo "Process $child did not stop after 15 s; forcing termination" >&2
            kill -KILL "$child" 2>/dev/null || true
        fi
        wait "$child" 2>/dev/null || true
    done
    if [[ -n $log_pid ]]; then
        kill -TERM "$log_pid" 2>/dev/null || true
        wait "$log_pid" 2>/dev/null || true
    fi
    echo "Logs retained in $log_directory"
    exit "$result"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

# %% readiness and supervision
wait_ready() {
    local child=$1 marker=$2 logfile=$3
    local deadline=$((SECONDS + startup_timeout))
    while ! grep -Fq -- "$marker" "$logfile"; do
        if ! kill -0 "$child" 2>/dev/null; then
            echo "Process $child exited before readiness; see $logfile" >&2
            return 1
        fi
        if ((SECONDS >= deadline)); then
            echo "Readiness timeout; see $logfile" >&2
            return 1
        fi
        sleep 0.1
    done
    kill -0 "$child" 2>/dev/null
}
echo "Logs: $log_directory"
if [[ $verbose == true ]]; then
    tail --pid="$$" -q -n +1 -F "$server_log" "$client_log" 9>&- &
else
    tail --pid="$$" -q -n +1 -F "$server_log" "$client_log" 9>&- |
        awk -W interactive -f "$script_directory/tiago_giskard_log.awk" 9>&- &
fi
log_pid=$!
"$python" -u "$script_directory/tiago_giskard_bringup.py" >"$server_log" 2>&1 9>&- &
server_pid=$!
wait_ready "$server_pid" 'READY: /giskard/command' "$server_log"
"$python" -u "$script_directory/tiago_giskard_goal.py" --listen "$fifo" >"$client_log" 2>&1 9>&- &
client_pid=$!
wait_ready "$client_pid" 'READY CLIENT:' "$client_log"
echo "STACK READY. In another container terminal:"
printf './scripts/tiago-goal 0.30 --fifo %q\n' "$fifo"
echo "Ctrl+C stops both processes."
wait -n "$server_pid" "$client_pid"
echo "A child process exited; shutting down the other process." >&2
exit 1
