"""
Failures preventing a supervised elevator-entry request.
"""

# %% imports
from dataclasses import dataclass

from krrood.exceptions import DataclassException


# %% entry failures
@dataclass
class EntryError(DataclassException):
    """
    Report an unmet entry requirement without scheduling a retry.
    """

    reason: str
    """Unavailable condition or failed operation."""

    def error_message(self) -> str:
        return self.reason

    def suggest_correction(self) -> str:
        return "Check the logged phase, map geometry, feedback, door and command routing before a new request."
