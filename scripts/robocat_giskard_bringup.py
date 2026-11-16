"""
Keep the odometry-based Robocat Giskard server ready for separate clients.
"""

# %% imports
from __future__ import annotations

import argparse
import logging
import sys
import time
from contextlib import ExitStack, redirect_stderr
from pathlib import Path
from rclpy.utilities import remove_ros_args

from experiments.robocat_safety import RobocatEnvironment, RobocatTopic
from experiments.robocat_startup import RobocatStartupOutput
from experiments.robocat_tracking import TrackingPublisher
from experiments.robocat_goal_diagnostics import DiagnosticControlLoop
from giskardpy.motion_statechart.trajectory_tracking import TrajectoryTrackingContext

from giskardpy.middleware.ros2 import rospy
from giskardpy.middleware.ros2.giskard import Giskard
from giskardpy.middleware.ros2.graceful_shutdown import GracefulShutdownSignals
from giskardpy.middleware.ros2.scripts.iai_robots.robocat import (
    RobocatBaseInterface,
    WorldWithRobocatConfig,
)
from giskardpy.middleware.ros2.scripts.iai_robots.tiago.single_arm import (
    BaseVelocityLimits,
)
from giskardpy.middleware.ros2.server_config import ExecutionMode, GiskardServerConfig
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.adapters.ros.tf_publisher import TFPublisher, TfFrameNames
from semantic_digital_twin.robots.robocat import Robocat


# %% persistent server
def main() -> None:
    """
    Load Robocat once and accept actions without sending a startup goal.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--max-linear", type=float, default=0.1)
    parser.add_argument("--max-angular", type=float, default=0.2)
    parser.add_argument(
        "--no-tracking",
        action="store_true",
        help="Disable the 10 Hz tracker diagnostic topic",
    )
    parser.add_argument(
        "--verbose", action="store_true", help="Show all model startup notices"
    )
    options = parser.parse_args(remove_ros_args()[1:])
    RobocatEnvironment().validate()
    limits = BaseVelocityLimits(options.max_linear, options.max_angular)
    with ExitStack() as cleanup:
        rospy.init_node("giskard")
        cleanup.callback(rospy.shutdown)
        cleanup.enter_context(GracefulShutdownSignals())
        time.sleep(2)
        if any(
            name == "/giskard/command/_action/send_goal"
            for name, _ in rospy.get_node().get_service_names_and_types()
        ):
            parser.error("A Giskard action server is already running in this domain")
        server = Giskard(
            world_config=WorldWithRobocatConfig(
                root_name=PrefixedName("odom"),
                odom_body_name=PrefixedName("robocat_drive_odom"),
                urdf=Path(Robocat.get_ros_file_path()).read_text(),
                velocity_limits=limits,
            ),
            robot_interface_config=RobocatBaseInterface(velocity_limits=limits),
            server_config=GiskardServerConfig(execution_mode=ExecutionMode.CLOSED_LOOP),
        )
        with ExitStack() as startup:
            if not options.verbose:
                messages = RobocatStartupOutput()
                startup.callback(messages.emit, sys.stderr)
                startup.enter_context(redirect_stderr(messages.output))
                logger = logging.getLogger(Giskard.__module__)
                logger.addFilter(messages)
                startup.callback(logger.removeFilter, messages)
            server.setup()
        native_loop = server.motion_server.control_loop
        diagnostic_loop = DiagnosticControlLoop.from_loop(native_loop, rospy.get_node())
        server.motion_server.control_loop = diagnostic_loop
        cleanup.callback(diagnostic_loop.close)
        if not options.no_tracking:
            tracking = TrackingPublisher(rospy.get_node())
            server.executor.context.add_extension(TrajectoryTrackingContext(tracking))
            cleanup.callback(tracking.close)
        server.tf_publisher.stop()
        server.tf_publisher = TFPublisher(
            node=rospy.get_node(),
            _world=server.world_config.world,
            frame_names=TfFrameNames(prefix="giskard/"),
        )
        cleanup.callback(server.close_world_model_ros_interface)
        cleanup.callback(native_loop.stop)
        print(
            f"READY: /giskard/command; Robocat; odom; feedback {rospy.get_node().resolve_topic_name(RobocatTopic.ODOMETRY)}; limits {limits.maximum_forward_velocity:g} m/s, {limits.maximum_yaw_velocity:g} rad/s; waiting for goals",
            flush=True,
        )
        print(
            "DIAGNOSTICS: /giskard/goal_diagnostics (native decision snapshots)",
            flush=True,
        )
        server.motion_server.live()


if __name__ == "__main__":
    main()
