#!/usr/bin/env bash
# %% usage
set -euo pipefail
if [[ ${1:-} == --help ]]; then
    printf 'Usage: bash scripts/launch_robocat_dds_router.sh [--control] [--command-topic /cmd_vel|/snm_vel]\n'
    printf 'Run on the laptop host. Forward Robocat feedback from domain 102 to local domain 103.\n'
    printf 'Default: feedback only. --control also enables volatile cmd_vel toward the robot mux.\n'
    printf 'The ROS discovery graph is never forwarded. Ctrl+C stops the router.\n'
    exit 0
fi
control=false
command_topic=/cmd_vel
while (($#)); do
    case "$1" in
        --control) control=true; shift ;;
        --command-topic)
            if (($# < 2)); then printf 'Missing value for --command-topic\n' >&2; exit 2; fi
            case "$2" in
                /cmd_vel|/snm_vel) command_topic=$2 ;;
                *) printf 'Invalid command topic: %s\n' "$2" >&2; exit 2 ;;
            esac
            shift 2 ;;
        *) printf 'Unknown argument: %s\n' "$1" >&2; exit 2 ;;
    esac
done
if [[ $command_topic != /cmd_vel && $control != true ]]; then
    printf '%s\n' '--command-topic requires --control' >&2
    exit 2
fi

# %% prerequisites
repository=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
image=robocat-dds-router:3.5.4
for program in docker ip; do
    command -v "$program" >/dev/null
done
if ! ip -4 -o address show | grep -q 'inet 10\.68\.0\.206/'; then
    printf 'Expected Ethernet address 10.68.0.206 is absent. Check the dedicated Robocat XML profiles before starting.\n' >&2
    exit 1
fi
docker image inspect "$image" >/dev/null

# %% optional command route generated in the CRAM Python environment
configuration=/workspace/cram/doc/robocat_dds_router.yaml
extra_mount=()
if [[ $control == true ]]; then
    configuration_directory=$(mktemp -d /tmp/robocat-dds-control.XXXXXX)
    trap 'rm -rf -- "$configuration_directory"' EXIT
    chmod 755 "$configuration_directory"
    docker exec aicor-cram-demo /opt/ros/cram-env/bin/python \
        -m experiments.robocat_bridge --base /workspace/cram/doc/robocat_dds_router.yaml \
        --command-topic "$command_topic" \
        > "$configuration_directory/router.yaml"
    chmod 644 "$configuration_directory/router.yaml"
    extra_mount=(--mount "type=bind,source=$configuration_directory,target=/router-config,readonly")
    configuration=/router-config/router.yaml
fi

# %% foreground feedback router
docker run --rm --init --name robocat-dds-router \
    --network host --read-only --cap-drop ALL --security-opt no-new-privileges \
    --memory 256m --memory-swap 256m --cpus 1 --pids-limit 128 \
    --tmpfs /tmp:rw,noexec,nosuid,size=16m \
    --mount "type=bind,source=$repository/doc,target=/workspace/cram/doc,readonly" \
    "${extra_mount[@]}" "$image" --config-path "$configuration"
