# %% imports
from __future__ import annotations

import math
from types import SimpleNamespace
from unittest.mock import Mock

from test.experiments_test.test_tiago_giskard_safety import client_script


# %% reported and applied tolerances
def test_description_uses_configured_tolerances(client_script: dict) -> None:
    """
    Formatting must use configured values rather than the old fixed log text.
    """
    tolerance = client_script["MotionTolerances"](position=0.02, orientation_degrees=3)
    assert tolerance.describe() == "0.02 m / 3 deg"


def test_chart_uses_the_same_tolerance_configuration(client_script: dict) -> None:
    """
    The chart and its message must read the same configuration instance.
    """
    tolerance = client_script["MotionTolerances"](position=0.02, orientation_degrees=3)
    motion = SimpleNamespace(
        world=SimpleNamespace(root=Mock()),
        robot=SimpleNamespace(root=Mock()),
        target=Mock(),
        tolerances=tolerance,
    )
    chart = client_script["StraightBaseMotion"]._motion_chart.fget(motion)
    assert chart.nodes[0].threshold == tolerance.position
    assert chart.nodes[1].threshold == math.radians(tolerance.orientation_degrees)
