# %% native map visualization
set -eo pipefail
source /opt/ros/humble/setup.bash
source /opt/pal/alum/setup.bash
export ROS_DOMAIN_ID=102 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
if [[ -z ${DISPLAY:-} ]]; then
    printf 'RViz requires X11 forwarding: launch Byobu from a graphical laptop terminal.\n' >&2
    exit 1
fi
export LIBGL_ALWAYS_SOFTWARE=1 QT_X11_NO_MITSHM=1
exec ros2 run rviz2 rviz2 -d /home/pal/robocat_elevator_mapping_20261109/waypoints.rviz
