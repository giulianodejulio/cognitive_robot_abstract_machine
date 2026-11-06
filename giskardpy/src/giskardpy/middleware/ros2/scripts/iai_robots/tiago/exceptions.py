# %% imports

from __future__ import annotations

from dataclasses import dataclass

from krrood.exceptions import DataclassException

# %% invalid base velocities


@dataclass
class InvalidBaseSpeedLimits(DataclassException):
    """
    The configured base speed bounds are not finite positive values.
    """

    maximum_forward_velocity: float
    """The rejected forward speed bound."""

    maximum_yaw_velocity: float
    """
    The rejected yaw speed bound.
    """

    def error_message(self) -> str:
        return "The base speed limits must be finite and positive."

    def suggest_correction(self) -> str:
        return "Configure finite positive forward and yaw speed bounds."


@dataclass
class NonFiniteBaseCommand(DataclassException):
    """
    The solver supplied a base command containing a non-finite velocity.
    """

    velocities: list[float]
    """The rejected velocity values."""

    def error_message(self) -> str:
        return f"The base command contains non-finite velocities: {self.velocities}."

    def suggest_correction(self) -> str:
        return "Inspect the solver state and robot feedback before commanding the base."
