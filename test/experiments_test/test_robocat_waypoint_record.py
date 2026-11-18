"""
Waypoint persistence rejects stale, moving and duplicate acquisitions.
"""

# %% imports
from dataclasses import replace
import pytest
from scripts.robocat_waypoint_record import MapFile, WaypointEvidence
import numpy as np
import yaml
from PIL import Image
from nav_msgs.msg import OccupancyGrid


# %% acquisition guards
@pytest.mark.parametrize(
    "name,existing,changes,blocked",
    [
        ("elevator_inside", [], {}, False),
        ("bad/name", [], {}, True),
        ("elevator_inside", ["floor/elevator_inside"], {}, True),
        ("elevator_inside", [], {"pose_age": 0.6}, True),
        ("elevator_inside", [], {"wheel_age": -0.6}, True),
        ("elevator_inside", [], {"linear_velocity": 0.01}, True),
        ("elevator_inside", [], {"angular_velocity": 0.02}, True),
    ],
)
def test_capture_guards(
    name: str, existing: list[str], changes: dict, blocked: bool
) -> None:
    """
    Only a fresh stationary named acquisition can be saved.
    """
    evidence = replace(WaypointEvidence(0.05, 0.05, 0.0, 0.0), **changes)
    assert bool(evidence.blockers(name, existing, "floor/" + name)) is blocked


# %% published map identity
@pytest.mark.parametrize("altered", [False, True])
def test_map_identity_uses_cells_instead_of_old_yaml_parameter(
    tmp_path, altered: bool
) -> None:
    """
    A single changed cell distinguishes two map revisions.
    """
    image = Image.fromarray(np.array([[0, 255], [205, 255]], dtype=np.uint8))
    image.save(tmp_path / "map.pgm")
    configuration = {
        "image": "map.pgm",
        "resolution": 0.05,
        "origin": [0, 0, 0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.196,
    }
    path = tmp_path / "map.yaml"
    path.write_text(yaml.safe_dump(configuration))
    message = OccupancyGrid()
    message.info.width = 2
    message.info.height = 2
    message.info.resolution = 0.05
    message.info.origin.orientation.w = 1.0
    message.data = [-1, 0, 0 if altered else 100, 0]
    assert MapFile(str(path)).matches(message) is (not altered)
