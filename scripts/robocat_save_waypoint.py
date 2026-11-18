"""
Capture the current localized base pose in the native Eulero database.
"""

# %% imports
from __future__ import annotations

import argparse
from contextlib import ExitStack
from dataclasses import dataclass
import math
import time
from typing_extensions import Any

import rclpy
from rclpy.node import Node
from eulero_msgs.srv import CreateWaypoint, List, ListActive
from geometry_msgs.msg import Pose, TransformStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose

from scripts.robocat_waypoint_record import MapFile, WaypointEvidence


# %% native capture
@dataclass
class WaypointCapture:
    """
    Read current native context before persisting a named floor waypoint.
    """

    node: Node
    """Native ROS node in the robot domain."""
    parser: argparse.ArgumentParser
    """
    Command parser reporting actionable acquisition failures.
    """

    wheel: Odometry | None = None
    """
    Most recent measured wheel feedback.
    """

    map_message: OccupancyGrid | None = None
    """
    Actual occupancy grid currently published on the robot.
    """

    def receive_map(self, message: OccupancyGrid) -> None:
        """
        Retain the published grid rather than an old map-server filename.
        """
        self.map_message = message

    def receive_wheel(self, message: Odometry) -> None:
        """
        Retain measured wheel evidence without commanding the base.
        """
        self.wheel = message

    def call(self, service_type: Any, name: str, request: Any) -> Any:
        """
        Require a successful bounded native service response.
        """
        client = self.node.create_client(service_type, name)
        if not client.wait_for_service(timeout_sec=5):
            self.parser.error(f"Service unavailable: {name}")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=8)
        if not future.done() or future.exception() is not None:
            self.parser.error(f"Service did not complete: {name}")
        response = future.result()
        self.node.destroy_client(client)
        if not response.success:
            self.parser.error(f"Service rejected request: {name}; {response}")
        return response

    def pose(
        self, buffer: Buffer, frame: str, name: str, existing: list[str], key: str
    ) -> TransformStamped:
        """
        Require fresh localization and stationary wheel feedback.
        """
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        if self.wheel is None or not buffer.can_transform(
            frame, "base_footprint", rclpy.time.Time()
        ):
            self.parser.error("Missing wheel feedback or localized base transform")
        transform = buffer.lookup_transform(frame, "base_footprint", rclpy.time.Time())
        now = self.node.get_clock().now().nanoseconds / 1e9
        evidence = WaypointEvidence(
            now - (transform.header.stamp.sec + transform.header.stamp.nanosec / 1e9),
            now - (self.wheel.header.stamp.sec + self.wheel.header.stamp.nanosec / 1e9),
            self.wheel.twist.twist.linear.x,
            self.wheel.twist.twist.angular.z,
        )
        blockers = evidence.blockers(name, existing, key)
        if blockers:
            self.parser.error("; ".join(blockers))
        return transform


def main() -> None:
    """
    Preview and optionally save one stationary pose without moving the robot.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("name")
    parser.add_argument(
        "--check", action="store_true", help="Preview only; never create a waypoint"
    )
    options = parser.parse_args()
    rclpy.init()
    with ExitStack() as cleanup:
        cleanup.callback(rclpy.shutdown)
        node = rclpy.create_node("robocat_waypoint_capture")
        cleanup.callback(node.destroy_node)
        capture = WaypointCapture(node, parser)
        node.create_subscription(
            Odometry, "/mobile_base_controller/odom", capture.receive_wheel, 10
        )
        node.create_subscription(
            OccupancyGrid,
            "/map",
            capture.receive_map,
            QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            ),
        )
        buffer = Buffer()
        listener = TransformListener(buffer, node)
        cleanup.callback(listener.unregister)
        active = capture.call(
            ListActive, "/eulero_manager/list_active", ListActive.Request()
        ).result
        if capture.map_message is None:
            parser.error("No live /map received")
        published_map = capture.map_message
        map_frame = published_map.header.frame_id
        maps = [
            entry
            for entry in active.maps
            if entry.is_active
            and entry.is_enabled
            and MapFile(entry.filename).matches(published_map)
        ]
        if len(maps) != 1:
            parser.error(
                "Published /map must match exactly one active enabled Eulero map file"
            )
        selected = maps[0]
        floors = [
            entry
            for entry in active.floors
            if entry.is_active
            and entry.is_enabled
            and entry.parent_key + "/" + entry.name == selected.parent_key
        ]
        if len(floors) != 1:
            parser.error("Active map must belong to exactly one active enabled floor")
        loaded_filename = selected.filename
        key = selected.parent_key + "/" + options.name
        existing = capture.call(
            List,
            "/eulero_manager/list_waypoints",
            List.Request(parent_key=selected.parent_key),
        ).results
        print(
            f"PUBLISHED MAP MATCH: {selected.parent_key}/{selected.name}; YAML: {loaded_filename}",
            flush=True,
        )
        transform = capture.pose(buffer, map_frame, options.name, existing, key)
        floor_transform = TransformStamped(transform=selected.transform)
        base_pose = Pose(orientation=transform.transform.rotation)
        base_pose.position.x = transform.transform.translation.x
        base_pose.position.y = transform.transform.translation.y
        base_pose.position.z = transform.transform.translation.z
        floor_pose = do_transform_pose(base_pose, floor_transform)
        print(
            f"MAP: {selected.parent_key}/{selected.name}; FRAME: {map_frame}; YAML: {loaded_filename}",
            flush=True,
        )
        print(f"WAYPOINT: {key}\nPOSE IN FLOOR: {floor_pose}", flush=True)
        print(
            "Confirm in RViz that localization matches the real robot. No movement is requested.",
            flush=True,
        )
        if options.check:
            print("CHECK ONLY: no waypoint saved", flush=True)
            return
        if (
            input("Type SAVE to persist this pose, anything else to cancel: ").strip()
            != "SAVE"
        ):
            print("CANCELED: no waypoint saved", flush=True)
            return
        current = capture.pose(buffer, map_frame, options.name, existing, key)
        difference = math.dist(
            (transform.transform.translation.x, transform.transform.translation.y),
            (current.transform.translation.x, current.transform.translation.y),
        )
        alignment = abs(
            sum(
                a * b
                for a, b in zip(
                    (
                        transform.transform.rotation.x,
                        transform.transform.rotation.y,
                        transform.transform.rotation.z,
                        transform.transform.rotation.w,
                    ),
                    (
                        current.transform.rotation.x,
                        current.transform.rotation.y,
                        current.transform.rotation.z,
                        current.transform.rotation.w,
                    ),
                )
            )
        )
        if difference > 0.02 or alignment < math.cos(math.radians(1)):
            parser.error("Pose changed after preview; rerun capture")
        if capture.map_message is None or not MapFile(selected.filename).matches(
            capture.map_message
        ):
            parser.error("Published map changed after preview; rerun capture")
        latest_existing = capture.call(
            List,
            "/eulero_manager/list_waypoints",
            List.Request(parent_key=selected.parent_key),
        ).results
        if key in latest_existing:
            parser.error("Waypoint appeared after preview; no overwrite allowed")
        request = CreateWaypoint.Request()
        request.new_waypoint.name = options.name
        request.new_waypoint.parent_key = selected.parent_key
        request.new_waypoint.is_active = True
        request.new_waypoint.is_enabled = True
        request.new_waypoint.transform.translation.x = floor_pose.position.x
        request.new_waypoint.transform.translation.y = floor_pose.position.y
        request.new_waypoint.transform.translation.z = floor_pose.position.z
        request.new_waypoint.transform.rotation = floor_pose.orientation
        response = capture.call(
            CreateWaypoint, "/eulero_manager/create_waypoint", request
        )
        verified = capture.call(
            List,
            "/eulero_manager/list_waypoints",
            List.Request(parent_key=selected.parent_key),
        ).results
        if key not in verified:
            parser.error(
                f"Create returned {response}, but waypoint was not found on verification"
            )
        print(f"SAVED AND VERIFIED: {key}", flush=True)


if __name__ == "__main__":
    main()
