"""Cancel a Robocat forward goal, wait for rest, optionally turn, then advance."""

# %% imports
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

from scripts.tiago_goal_sequence import GoalSequence


# %% dedicated Robocat client
@dataclass
class RobocatGoalSequence(GoalSequence):
    """Run the existing cancellable sequence through the Robocat client."""

    fifo: Path = Path("/tmp/robocat-giskard-goals.fifo")
    """The request pipe of the persistent Robocat client."""

    sender: Path = field(
        default_factory=lambda: Path(__file__).resolve().with_name("robocat-goal")
    )
    """The Robocat sender used for goals, cancellation and readiness checks."""


# %% explicit real execution
if __name__ == "__main__":
    RobocatGoalSequence.parse().run()
