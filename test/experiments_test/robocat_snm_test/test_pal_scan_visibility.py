"""
The Robocat entry observer uses confirmed PAL no-return semantics by default.
"""

# %% imports
import numpy as np

from experiments.robocat_snm.feedback import EntryInputs

from .test_feedback import entry_inputs


# %% source integration
def test_pal_no_returns_allow_open_door_and_free_passage(
    entry_inputs: EntryInputs,
) -> None:
    """
    Source-specific no-return handling requires no gap-acceptance override.
    """
    reading = entry_inputs.source.get_lidar_reading.return_value
    reading.ranges[:] = np.inf
    observation = entry_inputs.observe(navigation_idle=True)
    assert entry_inputs.visibility.allow_isolated_gaps is False
    assert observation.door_open is True
    assert observation.passage_clear is True
    assert observation.healthy is True


def test_pal_invalid_measurements_still_cannot_certify_passage(
    entry_inputs: EntryInputs,
) -> None:
    """
    Unknown measurements remain distinct from no object detected in range.
    """
    reading = entry_inputs.source.get_lidar_reading.return_value
    reading.ranges[:] = np.nan
    observation = entry_inputs.observe(navigation_idle=True)
    assert observation.door_open is None
    assert observation.passage_clear is None
