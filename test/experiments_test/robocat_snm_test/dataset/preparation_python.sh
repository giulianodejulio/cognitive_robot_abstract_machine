#!/usr/bin/env bash
# %% inspect sourced environment without contacting ROS
set -eu
[[ $ROBOCAT_PREPARATION_ROS == loaded ]]
[[ $ROBOCAT_PREPARATION_PAL == loaded ]]
[[ $ROS_DOMAIN_ID == 102 ]]
[[ $RMW_IMPLEMENTATION == rmw_cyclonedds_cpp ]]
printf '%s\n' "$@"
