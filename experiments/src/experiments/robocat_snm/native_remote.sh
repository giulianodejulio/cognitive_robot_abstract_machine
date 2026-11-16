# Native navigation stays in Humble; stdin remains available for watchdog heartbeats.
set -e
source /opt/ros/humble/setup.bash
source /opt/pal/alum/setup.bash
export ROS_DOMAIN_ID=102
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
exec python3 -u -c "$@"
