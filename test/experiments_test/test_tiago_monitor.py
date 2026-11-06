# %% imports

from __future__ import annotations

import pytest
import json
from pathlib import Path

from krrood.adapters.json_serializer import DataclassJSONSerializer
from experiments.tiago_monitor.status import (
    MonitorSnapshot,
    StreamCondition,
    StreamStatus,
)
from experiments.tiago_monitor.view import MonitorFile, RobotView
from semantic_digital_twin.world import World

# %% stream freshness


@pytest.mark.parametrize(
    "sample_time, expected",
    [
        (None, StreamCondition.WAITING),
        (99.0, StreamCondition.CURRENT),
        (98.0, StreamCondition.CURRENT),
        (97.0, StreamCondition.STALE),
        (103.0, StreamCondition.CLOCK_SKEW),
    ],
)
def test_stream_classifies_timestamp_age(
    sample_time: float | None, expected: StreamCondition
) -> None:
    """
    Missing, recent, old and future samples have distinct conditions.
    """
    status = StreamStatus(topic="scan", sample_time=sample_time)

    status.update(current_time=100.0, maximum_age=2.0)

    assert status.condition == expected


def test_new_sample_recovers_a_stale_stream() -> None:
    """
    A fresh sample restores a stream previously marked stale.
    """
    status = StreamStatus(topic="scan", sample_time=90.0)
    status.update(current_time=100.0, maximum_age=2.0)
    status.sample_time = 100.0

    status.update(current_time=101.0, maximum_age=2.0)

    assert status.condition == StreamCondition.CURRENT


def test_evaluated_stream_health_survives_serialization() -> None:
    """
    The browser receives computed age and condition as well as the sample time.
    """
    status = StreamStatus(topic="scan", sample_time=90.0)
    status.update(current_time=100.0, maximum_age=2.0)

    restored = DataclassJSONSerializer.from_json(
        DataclassJSONSerializer.to_json(status), StreamStatus
    )

    assert restored == status


# %% browser publication


def test_snapshot_round_trips_through_the_framework_serializer(
    mini_world: World, tmp_path: Path
) -> None:
    """
    The browser file preserves the structured observation snapshot.
    """
    view = RobotView(world=mini_world, directory=tmp_path)
    snapshot = MonitorSnapshot(
        generated_at=100.0,
        maximum_age=2.0,
        streams=[StreamStatus(topic="scan")],
        poses=view.get_mesh_poses(),
        lidar_translation_error=None,
        penetrating_contacts=None,
    )

    view.write(snapshot)

    restored = DataclassJSONSerializer.from_json(
        json.loads((tmp_path / MonitorFile.SNAPSHOT).read_text()), MonitorSnapshot
    )
    assert restored == snapshot
    assert list(tmp_path.glob(f"*{MonitorFile.TEMPORARY_SUFFIX}")) == []
