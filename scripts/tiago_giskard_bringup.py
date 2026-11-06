"""
Keep the real TIAGo base controller available for separate goal clients.
"""

# %% imports
from __future__ import annotations

import argparse
import time
from contextlib import ExitStack
from dataclasses import dataclass
from pathlib import Path

from giskardpy.middleware.ros2 import rospy
from giskardpy.middleware.ros2.giskard import Giskard
from giskardpy.middleware.ros2.graceful_shutdown import GracefulShutdownSignals
from giskardpy.middleware.ros2.scripts.iai_robots.tiago.single_arm import (
    BaseVelocityLimits,
    SingleArmTiagoBaseInterface,
    WorldWithSingleArmTiagoConfig,
)
from giskardpy.middleware.ros2.server_config import ExecutionMode, GiskardServerConfig
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.robots.tiago_single_arm import TiagoSingleArm
from semantic_digital_twin.world_description.connections import Connection6DoF


# %% local odometry configuration
@dataclass
class LocalBaseInterface(SingleArmTiagoBaseInterface):
    """
    Track real base odometry with an identity localization connection.
    """

    def sync_6dof_joint_with_tf_frame(
        self, joint: Connection6DoF, tf_parent_frame: str, tf_child_frame: str
    ) -> None:
        """
        Leave localization independent of map and AMCL transforms.
        """
        return None


# %% persistent server
def main() -> None:
    """
    Load the controller once and wait for goals until SIGINT or SIGTERM.
    """
    argparse.ArgumentParser(description=__doc__).parse_args()
    with ExitStack() as cleanup:
        rospy.init_node("giskard")
        cleanup.callback(rospy.shutdown)
        cleanup.enter_context(GracefulShutdownSignals())
        time.sleep(2)
        if any(
            name == "/giskard/command/_action/send_goal"
            for name, types in rospy.get_node().get_service_names_and_types()
        ):
            raise RuntimeError(
                "A Giskard action server is already running; stop it before starting another"
            )
        limits = BaseVelocityLimits(
            maximum_forward_velocity=0.3,   # original value: 0.2
            maximum_yaw_velocity=0.2        # 0.1
        )
        server = Giskard(
            world_config=WorldWithSingleArmTiagoConfig(
                root_name=PrefixedName("odom"),
                odom_body_name=PrefixedName("trial_drive_odom"),
                urdf=Path(TiagoSingleArm.get_ros_file_path()).read_text(),
                velocity_limits=limits,
            ),
            robot_interface_config=LocalBaseInterface(velocity_limits=limits),
            server_config=GiskardServerConfig(execution_mode=ExecutionMode.CLOSED_LOOP),
        )
        server.setup()
        cleanup.callback(server.close_world_model_ros_interface)
        cleanup.callback(server.motion_server.control_loop.stop)
        print(
            "READY: /giskard/command; odom; limits "
            f"{limits.maximum_forward_velocity:g} m/s, "
            f"{limits.maximum_yaw_velocity:g} rad/s; waiting for goals",
            flush=True,
        )
        server.motion_server.live()


if __name__ == "__main__":
    main()
