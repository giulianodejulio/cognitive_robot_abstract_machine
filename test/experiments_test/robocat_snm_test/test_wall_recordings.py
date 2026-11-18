"""
Optional read-only regressions against the locally retained cabin recordings.
"""

# %% imports
from pathlib import Path
import sqlite3
import statistics

import numpy as np
import pytest
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.serialization import deserialize_message
from experiments.robocat_wall import (
    CabinWallEstimator,
    WallStopSettings,
    WallStopController,
)
from experiments.robocat_snm.odometry_history import OdometryHistory
from experiments.tiago_goal_logging import PlanarPose


# %% local-only recording evidence
@pytest.mark.parametrize(
    "recording,expected",
    [("entry-20261117-121322", 1.149), ("entry-20261117-133711", 0.936)],
)
def test_recorded_stopped_wall_distance(recording: str, expected: float) -> None:
    """
    The same production fitter distinguishes autonomous stop and manual reference.
    """
    directory = (
        Path(__file__).resolve().parents[3] / "tmp/robocat-entry-bags" / recording
    )
    if not directory.is_dir():
        pytest.skip(
            "Local real-robot bag is not distributed in CI; synthetic cases cover the estimator"
        )
    database = sqlite3.connect(
        f"file:{next(directory.glob('*.db3'))}?mode=ro", uri=True
    )
    end = database.execute("select max(timestamp) from messages").fetchone()[0]
    rows = database.execute(
        "select data from messages where topic_id=(select id from topics where name='/scan') and timestamp>?",
        (end - 8_000_000_000,),
    )
    transform = np.eye(4)
    transform[0, 3] = 0.113
    distances = []
    for (data,) in rows:
        scan = deserialize_message(data, LaserScan)
        fit = CabinWallEstimator().estimate(scan, transform)
        assert fit is not None
        distances.append(fit.distance)
    database.close()
    assert statistics.median(distances) == pytest.approx(expected, abs=0.015)


def test_recorded_entry_accepts_continuous_wall_tracking() -> None:
    """
    A stopped-short run remains outside the wall stop band throughout its final arrest.
    """
    directory = (
        Path(__file__).resolve().parents[3]
        / "tmp/robocat-entry-bags/entry-20261117-121322"
    )
    if not directory.is_dir():
        pytest.skip(
            "Local real-robot bag is not distributed in CI; synthetic cases cover association"
        )
    database = sqlite3.connect(
        f"file:{next(directory.glob('*.db3'))}?mode=ro", uri=True
    )
    topics = dict(database.execute("select id,name from topics"))
    goal = database.execute(
        "select data from messages where topic_id=(select id from topics where name='/giskard/goal_diagnostics') order by timestamp limit 1"
    ).fetchone()[0]
    statuses = {
        status.name: {item.key: item.value for item in status.values}
        for status in deserialize_message(goal, DiagnosticArray).status
    }
    position, heading = statuses["position"], statuses["orientation"]
    nominal = PlanarPose(
        float(position["target.x"]),
        float(position["target.y"]),
        float(heading["target.yaw"]),
    )
    transform = np.eye(4)
    transform[0, 3] = 0.113
    controller = WallStopController(WallStopSettings(0.936), nominal, transform)
    history = OdometryHistory()
    wheel = None
    samples = []
    for timestamp, topic, data in database.execute(
        "select timestamp,topic_id,data from messages order by timestamp"
    ):
        name = topics[topic]
        if name == "/mobile_base_controller/odom":
            wheel = deserialize_message(data, Odometry)
            history.samples.append(wheel)
        elif name == "/scan" and 1794917634 <= timestamp / 1e9 <= 1794917644:
            assert wheel is not None
            sample = controller.observe(
                deserialize_message(data, LaserScan), wheel, history, timestamp / 1e9
            )
            assert sample.valid, sample.reason
            samples.append(sample)
    database.close()
    assert len(samples) > 80
    assert samples[-1].error > controller.settings.tolerance
    assert samples[-1].correction == pytest.approx(0.214, abs=0.04)
