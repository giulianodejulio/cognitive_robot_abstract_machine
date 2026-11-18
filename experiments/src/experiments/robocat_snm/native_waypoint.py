"""
Read a saved native Eulero waypoint in the current map frame.
"""

# %% imports
from __future__ import annotations

import argparse
from contextlib import ExitStack
import json
import time

import rclpy
from eulero_msgs.srv import GetWaypoint
from geometry_msgs.msg import Pose
from rosidl_runtime_py.convert import message_to_ordereddict
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose


# %% read-only native request
def main() -> None:
    """
    Return one enabled floor waypoint without submitting any motion.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("waypoint_key")
    options = parser.parse_args()
    rclpy.init()
    with ExitStack() as cleanup:
        cleanup.callback(rclpy.shutdown)
        node = rclpy.create_node("robocat_inside_waypoint_reader")
        cleanup.callback(node.destroy_node)
        buffer = Buffer()
        listener = TransformListener(buffer, node)
        cleanup.callback(listener.unregister)
        client = node.create_client(GetWaypoint, "/eulero_manager/get_waypoint")
        if not client.wait_for_service(timeout_sec=3):
            parser.error("Eulero get_waypoint is unavailable")
        future = client.call_async(
            GetWaypoint.Request(waypoint_key=options.waypoint_key)
        )
        rclpy.spin_until_future_complete(node, future, timeout_sec=5)
        if not future.done() or future.exception() is not None:
            parser.error("Eulero waypoint query did not complete")
        response = future.result()
        if (
            not response.success
            or not response.waypoint.is_active
            or not response.waypoint.is_enabled
        ):
            parser.error("Waypoint missing, inactive or disabled")
        waypoint = response.waypoint
        if waypoint.parent_key + "/" + waypoint.name != options.waypoint_key:
            parser.error("Eulero returned a different waypoint")
        deadline = time.monotonic() + 3
        while (
            not buffer.can_transform("map", waypoint.parent_key, rclpy.time.Time())
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(node, timeout_sec=0.05)
        if not buffer.can_transform("map", waypoint.parent_key, rclpy.time.Time()):
            parser.error("Cannot transform the Eulero floor into the current map")
        floor_pose = Pose(orientation=waypoint.transform.rotation)
        floor_pose.position.x = waypoint.transform.translation.x
        floor_pose.position.y = waypoint.transform.translation.y
        floor_pose.position.z = waypoint.transform.translation.z
        transform = buffer.lookup_transform(
            "map", waypoint.parent_key, rclpy.time.Time()
        )
        pose = do_transform_pose(floor_pose, transform)
        print(
            "WAYPOINT_RESULT "
            + json.dumps(
                {
                    "waypoint_key": options.waypoint_key,
                    "frame_id": "map",
                    "pose": message_to_ordereddict(pose),
                }
            ),
            flush=True,
        )


if __name__ == "__main__":
    main()
