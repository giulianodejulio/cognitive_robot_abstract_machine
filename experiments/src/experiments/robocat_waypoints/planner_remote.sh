# Run only the native planning action. Goal data arrives on stdin, never as shell code.
set -e
source /opt/ros/humble/setup.bash
source /opt/pal/alum/setup.bash
export ROS_DOMAIN_ID=102
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
goal_payload=$(cat)
exec timeout --signal=INT --kill-after=2s 12s \
  ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose \
  "$goal_payload" --timeout 8
