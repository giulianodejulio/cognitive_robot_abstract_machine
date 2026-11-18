"""
Native Giskard longitudinal regulation using a measured cabin-wall reference.
"""

# %% imports
from __future__ import annotations

from dataclasses import dataclass, field
import math

import numpy as np
from giskardpy.motion_statechart.context import (
    ContextExtension,
    MotionStatechartContext,
)
from giskardpy.motion_statechart.graph_node import ConvergingTask, NodeArtifacts
from giskardpy.motion_statechart.data_types import DefaultWeights
from krrood.symbolic_math import symbolic_math as sm
from semantic_digital_twin.adapters.ros.lidar import SubscribedLidarSource
from semantic_digital_twin.spatial_types import Pose
from semantic_digital_twin.world_description.world_entity import (
    KinematicStructureEntity,
)
from experiments.robocat_safety import RobocatFeedback
from experiments.robocat_wall import (
    WallSample,
    WallStopController,
    WallStopSettings,
    WallMotionProgress,
)
from experiments.robocat_snm.exceptions import EntryError
from experiments.tiago_goal_logging import PlanarPose


# %% existing real sensor context
@dataclass
class WallControlContext(ContextExtension):
    """
    Real scan and wheel inputs already subscribed by the server.
    """

    source: SubscribedLidarSource
    """Existing real scan source."""
    feedback: RobocatFeedback
    """
    Existing wheel stream and acquisition history.
    """

    base_T_laser: np.ndarray
    """Model-derived sensor offset in base coordinates."""


# %% native scalar position objective
@dataclass(eq=False, repr=False)
class LidarLongitudinalTask(ConvergingTask):
    """
    Update one native position constraint from fresh cabin-wall measurements.
    """

    root_link: KinematicStructureEntity = field(kw_only=True)
    """
    Wheel odometry root of the controlled base.
    """

    tip_link: KinematicStructureEntity = field(kw_only=True)
    """
    Base origin whose wall distance is calibrated.
    """

    goal_pose: Pose = field(kw_only=True)
    """
    Nominal target used to bound adaptation and retain travel heading.
    """

    settings: WallStopSettings = field(kw_only=True)
    """
    Calibrated wall reference and adaptation bounds.
    """
    reference_velocity: float = 0.3
    """
    Native position-constraint reference speed, bounded by server dynamics.
    """

    controller: WallStopController = field(init=False, repr=False)
    """
    Acquisition-time wall estimator and bounded endpoint filter.
    """

    target_progress: sm.FloatVariable = field(init=False, repr=False)
    """
    Updated native position target along the travel heading.
    """

    measured_distance: sm.FloatVariable = field(init=False, repr=False)
    """
    Current measured perpendicular wall distance.
    """

    last_sample: WallSample | None = field(default=None, init=False, repr=False)
    """
    Exact wall evidence used by the latest control cycle.
    """

    progress_guard: WallMotionProgress = field(init=False, repr=False)
    """
    Bounded settled waiting outside the calibrated distance band.
    """

    def build_artifacts(self, context: MotionStatechartContext) -> NodeArtifacts:
        """
        Reuse native position constraints, optimization and braking limits.
        """
        inputs = context.require_extension(WallControlContext)
        goal = context.world.transform(
            spatial_object=self.goal_pose, target_frame=self.root_link
        )
        nominal = PlanarPose(float(goal.x), float(goal.y), float(goal.yaw))
        self.controller = WallStopController(
            self.settings, nominal, inputs.base_T_laser
        )
        self.progress_guard = WallMotionProgress(self.settings.stalled_seconds)
        self.target_progress = sm.FloatVariable(f"{self.name}_wall_target_progress")
        self.measured_distance = sm.FloatVariable(f"{self.name}_wall_distance")
        context.float_variable_data.register_expression(self.target_progress)
        context.float_variable_data.register_expression(self.measured_distance)
        self.on_tick(context)
        current = context.world.compose_forward_kinematics_expression(
            self.root_link, self.tip_link
        ).position
        artifacts = NodeArtifacts(
            error=sm.abs(self.measured_distance - self.settings.reference_distance)
        )
        artifacts.geometry.add_position_constraint(
            expression_current=math.cos(nominal.yaw) * current.x
            + math.sin(nominal.yaw) * current.y,
            expression_goal=self.target_progress,
            reference_velocity=self.reference_velocity,
            quadratic_weight=DefaultWeights.WEIGHT_ABOVE_COLLISION_AVOIDANCE,
            name="wall/longitudinal",
        )
        return artifacts

    def on_tick(self, context: MotionStatechartContext) -> None:
        """
        Abort on lost or implausible wall evidence rather than use an old target.
        """
        inputs = context.require_extension(WallControlContext)
        if not inputs.source.has_reading or inputs.feedback.wheel_odometry is None:
            raise EntryError(
                "Lidar-controlled entry requires real scan and wheel feedback"
            )
        now = inputs.feedback.node.get_clock().now().nanoseconds / 1e9
        sample = self.controller.observe(
            inputs.source.received_scan,
            inputs.feedback.wheel_odometry,
            inputs.feedback.wheel_history,
            now,
        )
        self.last_sample = sample
        if not sample.valid:
            raise EntryError("Lidar stop feedback unavailable: " + sample.reason)
        if self.progress_guard.blocked(
            now,
            inputs.feedback.wheels_settled(),
            abs(sample.error) <= self.settings.tolerance,
        ):
            raise EntryError(
                "Base remained stopped outside the lidar distance band; no automatic recovery"
            )
        context.float_variable_data.set_value(
            self.target_progress, sample.target_progress
        )
        context.float_variable_data.set_value(self.measured_distance, sample.distance)
