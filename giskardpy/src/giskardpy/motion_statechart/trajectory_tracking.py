"""
Optional synchronous observers of trajectory tracking state.
"""

# %% imports
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np
from typing_extensions import TYPE_CHECKING

from giskardpy.motion_statechart.context import ContextExtension

if TYPE_CHECKING:
    from giskardpy.motion_statechart.context import MotionStatechartContext
    from giskardpy.motion_statechart.tasks.cartesian_tasks import (
        CartesianPositionTrajectory,
    )


# %% same-cycle state and observer
@dataclass
class TrajectoryTrackingState:
    """
    Measured tip and selected target in the trajectory's reference frame.
    """

    position: np.ndarray
    """Tip position before the next command is computed."""
    target: np.ndarray
    """
    Lookahead target selected in that same control cycle.
    """


@dataclass
class TrajectoryTrackingObserver(ABC):
    """
    Consume tracking state without changing task constraints or observations.
    """

    @abstractmethod
    def observe(
        self,
        trajectory: CartesianPositionTrajectory,
        context: MotionStatechartContext,
        state: TrajectoryTrackingState,
    ) -> None:
        """
        Observe the current measured position and newly selected target.
        """


@dataclass
class TrajectoryTrackingContext(ContextExtension):
    """
    Enable one optional tracking observer for a motion execution context.
    """

    observer: TrajectoryTrackingObserver
    """Observer invoked synchronously after the trajectory target is updated."""
