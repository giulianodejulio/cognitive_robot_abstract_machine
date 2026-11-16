# Extract only the native preparation sources into an owned temporary directory.
set -euo pipefail
work_directory=$(mktemp -d /tmp/robocat-preparation.XXXXXX)
trap 'rm -rf -- "$work_directory"' EXIT
tar -xf - -C "$work_directory"
set +u
source /opt/ros/humble/setup.bash
source /opt/pal/alum/setup.bash
set -u
export ROS_DOMAIN_ID=102 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export PYTHONPATH="$work_directory${PYTHONPATH:+:$PYTHONPATH}"
python3 -u -m experiments.robocat_snm.native_preparation "$@" </dev/null
