# Native recovery uses Humble and the robot's diagnostic interfaces.
set -euo pipefail
work_directory=$(mktemp -d /tmp/robocat-recovery-native.XXXXXX)
trap 'rm -rf -- "$work_directory"' EXIT
tar -xf - -C "$work_directory"
set +u
source /opt/ros/humble/setup.bash
source /opt/pal/alum/setup.bash
set -u
export ROS_DOMAIN_ID=102 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export PYTHONPATH="$work_directory${PYTHONPATH:+:$PYTHONPATH}"
python3 -u -m experiments.robocat_snm.motor_recovery "$@" </dev/null
