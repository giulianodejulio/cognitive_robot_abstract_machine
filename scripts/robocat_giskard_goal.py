"""
Persistent CRAM goal client for Robocat on the isolated Jazzy domain.
"""

# %% imports
from __future__ import annotations

import time
from concurrent.futures import ThreadPoolExecutor
from contextlib import ExitStack
from dataclasses import dataclass
from rclpy.utilities import remove_ros_args

from coraplex.alternative_motion_mapping import AlternativeMotion
from coraplex.datastructures.dataclasses import Context
from experiments.robocat_safety import RobocatFeedback, RobocatEnvironment
from giskardpy.middleware.ros2 import rospy
from giskardpy.middleware.ros2.graceful_shutdown import GracefulShutdownSignals
from scripts.tiago_giskard_goal import (
    GoalInbox,
    GoalOptions,
    GoalSession,
    StraightBaseMotion,
)
from semantic_digital_twin.adapters.ros.world_fetcher import fetch_world_from_service
from semantic_digital_twin.adapters.ros.world_synchronizer import WorldSynchronizer
from semantic_digital_twin.robots.robocat import Robocat


# %% Cartesian mapping without duplicating the task implementation
@dataclass
class RobocatStraightMotion(StraightBaseMotion, AlternativeMotion[Robocat]):
    """
    Use fixed-heading Cartesian straight goals with the Robocat semantic model.
    """


# %% persistent client
def main() -> None:
    """
    Keep the model, action client and local request protocol connected.
    """
    options = GoalOptions.parse(remove_ros_args()[1:])
    RobocatEnvironment().validate()
    with ExitStack() as cleanup:
        rospy.init_node("robocat_giskard_goal")
        cleanup.callback(rospy.shutdown)
        cleanup.enter_context(GracefulShutdownSignals())
        node = rospy.get_node()
        feedback = RobocatFeedback(node)
        options.preflight(feedback)
        if options.check:
            print("PREFLIGHT OK: Robocat feedback and interlocks", flush=True)
            return
        world = fetch_world_from_service(node, timeout_seconds=12)
        synchronizer = WorldSynchronizer(_world=world, node=node)
        cleanup.callback(synchronizer.close)
        robot = world.get_semantic_annotations_by_type(Robocat)[0]
        context = Context(
            world=world,
            robot=robot,
            ros_node=node,
            alternative_motion_mappings=[RobocatStraightMotion],
        )
        cleanup.callback(context.giskard_wrapper.close)
        pool = cleanup.enter_context(ThreadPoolExecutor(max_workers=1))
        pool.submit(time.monotonic).result()
        session = GoalSession(context=context, feedback=feedback, pool=pool)
        if options.listen is not None:
            GoalInbox(options.listen).serve(session)
        elif options.interactive:
            session.interact()
        else:
            if not session.send(options.distance):
                raise SystemExit(1)


if __name__ == "__main__":
    main()
