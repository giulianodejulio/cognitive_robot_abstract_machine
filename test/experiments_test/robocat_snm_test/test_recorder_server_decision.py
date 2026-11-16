"""
Entry bags include the authoritative Giskard decision stream.
"""

# %% imports
from pathlib import Path

import yaml


# %% recorder coverage
def test_recorder_includes_reliable_server_decisions() -> None:
    """
    Late recorder subscription retains the most recent server decision events.
    """
    root = Path(__file__).parents[3]
    assert (
        "/giskard/goal_diagnostics"
        in (root / "scripts/robocat-record-entry").read_text()
    )
    policy = yaml.safe_load((root / "doc/robocat_entry_bag_qos.yaml").read_text())
    assert policy["/giskard/goal_diagnostics"]["reliability"] == "reliable"
    assert policy["/giskard/goal_diagnostics"]["durability"] == "transient_local"
