"""
Unsafe mode combinations and invalid final distance cannot enable lidar stopping.
"""

# %% imports
import pytest
from experiments.robocat_snm.application import EntryOptions


@pytest.mark.parametrize(
    "arguments",
    [
        ["--wall-stop-distance", "nan"],
        ["--wall-stop-distance", "0.1"],
        ["--wall-stop-distance", "0.936", "--feedback", "laser"],
        ["--wall-stop-distance", "0.936", "--pass-door-action"],
    ],
)
def test_lidar_stop_requires_valid_wheel_mode(arguments: list[str]) -> None:
    """
    A wall reference cannot enable unsupported feedback or unsafe clearance.
    """
    with pytest.raises(SystemExit):
        EntryOptions.parse(arguments)
