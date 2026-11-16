#!/usr/bin/env bash
# %% isolated container environment
set -eo pipefail
role=${1:?Missing role}
shift
repository=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source /opt/ros/overlay_ws/install/setup.bash
export ROS_DOMAIN_ID=103 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$repository/doc/robocat_cyclonedds.xml"
export PYTHONPATH="$repository:${PYTHONPATH:-}"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
cd "$repository"
case "$role" in
    guard) exec /opt/ros/cram-env/bin/python -m experiments.robocat_snm.session_check ;;
    giskard)
        feedback=${3:-wheel}
        remapping=()
        case "$feedback" in
            wheel) remapping=(-r /direct_laser_odometry/odom:=/mobile_base_controller/odom) ;;
            laser) ;;
            *) printf 'Feedback must be wheel or laser\n' >&2; exit 2 ;;
        esac
        if [[ -n ${ROBOCAT_BYOBU_OWNER:-} ]]; then
            if [[ ! $ROBOCAT_BYOBU_OWNER =~ ^[[:alnum:]_.-]+$ ]]; then exit 2; fi
            printf '%s\n' "$$" > "/tmp/$ROBOCAT_BYOBU_OWNER.giskard.pid"
        fi
        exec /opt/ros/cram-env/bin/python -u scripts/robocat_giskard_bringup.py \
            --max-linear "${1:?Missing linear limit}" --max-angular "${2:?Missing angular limit}" \
            --ros-args -r /cmd_vel:=/snm_vel "${remapping[@]}"
        ;;
    snm)
        feedback=${ROBOCAT_FEEDBACK:-wheel}
        if [[ -n ${ROBOCAT_BYOBU_OWNER:-} ]]; then
            if [[ ! $ROBOCAT_BYOBU_OWNER =~ ^[[:alnum:]_.-]+$ ]]; then exit 2; fi
            printf '%s\n' "$$" > "/tmp/$ROBOCAT_BYOBU_OWNER.snm.pid"
        fi
        if ! bash scripts/robocat-entry --feedback "$feedback"; then
            printf 'Read-only SNM check failed. Correct the reported blockers before sending any request.\n'
        fi
        printf '\nSNM terminal ready. No movement goal has been sent.\n'
        printf 'Read-only check: bash scripts/robocat-entry --feedback %s\n' "$feedback"
        printf 'Execution examples and calibration: doc/robocat_snm.md\n'
        printf 'Ctrl+C cancels an active request. F6 detaches Byobu and leaves processes running.\n'
        exec bash --noprofile --norc -i
        ;;
    *) printf 'Unknown container role: %s\n' "$role" >&2; exit 2 ;;
esac
