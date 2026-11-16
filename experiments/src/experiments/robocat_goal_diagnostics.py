"""
Immutable ROS snapshots of native Giskard task decisions before QP integration.
"""

# %% imports
from __future__ import annotations

from dataclasses import dataclass, field
from enum import StrEnum

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from giskardpy.executor import Executor
from giskardpy.middleware.ros2.action_server import ActionServerHandler
from giskardpy.middleware.ros2.control_loop import ControlLoop
from giskardpy.middleware.ros2.scripts.iai_robots.robocat import RobocatOdometrySource
from giskardpy.motion_statechart.graph_node import EndMotion
from giskardpy.motion_statechart.motion_statechart import (
    MotionStatechart,
    StateHistory,
    StateHistoryObserver,
)
from giskardpy.motion_statechart.tasks.cartesian_tasks import (
    CartesianOrientation,
    CartesianPosition,
    CartesianPositionStraight,
)
from semantic_digital_twin.adapters.ros.input_synchronization import (
    OdometrySynchronizer,
)
from semantic_digital_twin.input_synchronization import WorldStateInputs
from semantic_digital_twin.spatial_types import Pose


# %% native decision events
class GoalDiagnosticTopic(StrEnum):
    """
    The decision stream recorded alongside SNM geometry and odometry.
    """

    DECISION = "/giskard/goal_diagnostics"


class DecisionStage(StrEnum):
    """
    Control phases distinguished from the controller's predicted next state.
    """

    STATE_CHANGE = "state_change_before_qp"
    COMPILED = "compiled_initial_state"
    STOPPED = "command_stop"


@dataclass
class GoalDecisionObserver(StateHistoryObserver):
    """
    Publish every native state transition, including the end-motion decision.
    """

    node: Node
    """
    ROS node whose clock timestamps the decision snapshot.
    """

    executor: Executor
    """
    Executor holding the current compiled goal and its measured world.
    """

    action_server: ActionServerHandler
    """
    Owner of the integer goal identifier printed in server logs.
    """

    inputs: WorldStateInputs
    """
    Actual odometry inputs applied by this control loop.
    """

    publisher: Publisher = field(init=False)
    """
    Reliable retained event publisher without a background timer.
    """

    def __post_init__(self) -> None:
        """
        Retain recent decision events for a recorder joining after startup.
        """
        self.publisher = self.node.create_publisher(
            DiagnosticArray,
            GoalDiagnosticTopic.DECISION,
            QoSProfile(
                depth=100,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

    def on_state_change(self, history: StateHistory) -> None:
        """
        Capture the world synchronously with the native observation update.
        """
        self.capture(DecisionStage.STATE_CHANGE)

    def capture(self, stage: DecisionStage) -> None:
        """
        Publish copied task values and held sensor samples for this exact phase.
        """
        chart = self.executor.motion_statechart
        if chart is None:
            return
        message = DiagnosticArray()
        message.header.stamp = self.node.get_clock().now().to_msg()
        common = {
            "goal_id": str(self.action_server.goal_id),
            "control_cycle": str(self.executor.control_cycles),
            "stage": stage.value,
            "end_motion": str(chart.is_end_motion()),
        }
        for source in self.inputs.synchronizers:
            if not isinstance(source, OdometrySynchronizer):
                continue
            applied = isinstance(source, RobocatOdometrySource)
            measured = source.applied_message if applied else source.latest_message
            if measured is None:
                continue
            common.update(
                {
                    "feedback_sample_is_applied": str(applied),
                    "feedback_topic": str(
                        self.node.resolve_topic_name(source.topic_name)
                    ),
                    "feedback_frame": measured.header.frame_id,
                    "feedback_child_frame": measured.child_frame_id,
                    "feedback_stamp_sec": str(measured.header.stamp.sec),
                    "feedback_stamp_nanosec": str(measured.header.stamp.nanosec),
                    "feedback.x": str(measured.pose.pose.position.x),
                    "feedback.y": str(measured.pose.pose.position.y),
                    "feedback.z": str(measured.pose.pose.position.z),
                    "feedback.qx": str(measured.pose.pose.orientation.x),
                    "feedback.qy": str(measured.pose.pose.orientation.y),
                    "feedback.qz": str(measured.pose.pose.orientation.z),
                    "feedback.qw": str(measured.pose.pose.orientation.w),
                }
            )
        for task in chart.nodes:
            if not isinstance(
                task,
                (
                    CartesianPosition,
                    CartesianPositionStraight,
                    CartesianOrientation,
                    EndMotion,
                ),
            ):
                continue
            values = {
                **common,
                "task_class": type(task).__name__,
                "observation": str(task.observation_state),
                "life_cycle": str(task.life_cycle_state),
            }
            status = DiagnosticStatus(name=str(task.name), message=stage.value)
            if not isinstance(task, EndMotion):
                values.update(self.task_values(task))
                status.hardware_id = str(task.tip_link.name)
            status.values = [
                KeyValue(key=key, value=value) for key, value in values.items()
            ]
            message.status.append(status)
        self.publisher.publish(message)

    def task_values(
        self, task: CartesianPosition | CartesianPositionStraight | CartesianOrientation
    ) -> dict[str, str]:
        """
        Read the compiled error and goal in the same root frame as its current pose.
        """
        current = self.executor.context.world.compute_forward_kinematics(
            task.root_link, task.tip_link
        )
        current_pose = Pose(position=current.position, orientation=current.quaternion)
        values = {
            "root_frame": str(task.root_link.name),
            "goal_reference_frame": str(task.goal_reference_frame.name),
            "threshold": str(task.threshold),
            "task_error": str(float(task.error_signal.evaluate()[0])),
            "current.x": str(float(current.position.x)),
            "current.y": str(float(current.position.y)),
            "current.z": str(float(current.position.z)),
            "current.yaw": str(float(current_pose.yaw)),
        }
        if isinstance(task, CartesianOrientation):
            rotation = task.root_T_goal_reference_frame @ task.goal_orientation
            target = Pose(orientation=rotation.quaternion)
            values.update(
                {
                    "error_unit": "rad",
                    "target.yaw": str(float(target.yaw.evaluate()[0])),
                }
            )
        else:
            target = task.root_T_goal_reference_frame @ task.goal_point
            values.update(
                {
                    "error_unit": "m",
                    "target.x": str(float(target.x.evaluate()[0])),
                    "target.y": str(float(target.y.evaluate()[0])),
                    "target.z": str(float(target.z.evaluate()[0])),
                }
            )
        return values


# %% use the existing history observer API without changing controller equations
@dataclass
class DiagnosticControlLoop(ControlLoop):
    """
    Attach decision telemetry to each compiled chart before executing it.
    """

    decisions: GoalDecisionObserver = field(kw_only=True)
    """
    Observer recording native transitions before command prediction.
    """

    observed_chart: MotionStatechart | None = field(default=None, init=False)
    """
    Chart whose history already owns the observer subscription.
    """

    observed_goal_id: int | None = field(default=None, init=False)
    """
    Goal owning that chart, excluding later compilation failures.
    """

    @classmethod
    def from_loop(cls, loop: ControlLoop, node: Node) -> DiagnosticControlLoop:
        """
        Retain every native loop component and add only a diagnostic observer.
        """
        return cls(
            executor=loop.executor,
            action_server=loop.action_server,
            feedback_publisher=loop.feedback_publisher,
            inputs=loop.inputs,
            cycle_counter=loop.cycle_counter,
            client_watchdog=loop.client_watchdog,
            world_updates=loop.world_updates,
            command_publishers=loop.command_publishers,
            decisions=GoalDecisionObserver(
                node, loop.executor, loop.action_server, loop.inputs
            ),
        )

    def run_cycle(self) -> None:
        chart = self.executor.motion_statechart
        if chart is not self.observed_chart:
            if self.observed_chart is not None:
                self.observed_chart.history.remove_observer(self.decisions)
            self.observed_chart = chart
            self.observed_goal_id = self.action_server.goal_id
            chart.history.add_observer(self.decisions)
            self.decisions.capture(DecisionStage.COMPILED)
        super().run_cycle()

    def stop(self) -> None:
        super().stop()
        if (
            self.observed_chart is not None
            and self.executor.motion_statechart is self.observed_chart
            and self.action_server.goal_id == self.observed_goal_id
        ):
            self.decisions.capture(DecisionStage.STOPPED)

    def close(self) -> None:
        """
        Release the event subscription and ROS publisher after server shutdown.
        """
        if self.observed_chart is not None:
            self.observed_chart.history.remove_observer(self.decisions)
        self.decisions.node.destroy_publisher(self.decisions.publisher)
