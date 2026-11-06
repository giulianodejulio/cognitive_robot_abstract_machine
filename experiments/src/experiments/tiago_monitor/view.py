# %% imports

from __future__ import annotations

import json
from dataclasses import dataclass
from enum import StrEnum
from pathlib import Path

import trimesh
from trimesh.viewer import scene_to_html
from typing_extensions import TYPE_CHECKING

from krrood.adapters.json_serializer import DataclassJSONSerializer
from experiments.tiago_monitor.status import MeshPose, MonitorSnapshot

if TYPE_CHECKING:
    from semantic_digital_twin.world import World


# %% browser artifacts


class MonitorFile(StrEnum):
    """
    Files served by the monitor's local web server.
    """

    PAGE = "index.html"
    SCRIPT = "monitor.js"
    SNAPSHOT = "snapshot.json"
    TEMPORARY_SUFFIX = ".tmp"


@dataclass
class RobotView:
    """
    A browser scene whose geometry stays fixed while mesh poses change.
    """

    world: World
    """
    The observed world containing the visual geometry.
    """

    directory: Path
    """
    The directory served by the local web server.
    """

    def create(self) -> None:
        """
        Export geometry and the browser updater without starting any ROS publishers.
        """
        self.directory.mkdir(parents=True, exist_ok=True)
        scene = trimesh.Scene()
        for body in self.world.bodies:
            for index, geometry in enumerate(body.visual):
                scene.add_geometry(
                    geometry.mesh,
                    node_name=f"{body.name.name}_{index}",
                    transform=self.world.compute_forward_kinematics_np(
                        self.world.root, body
                    )
                    @ geometry.origin.to_np(),
                )
        page = scene_to_html(scene).replace(
            "</body>", f'<script src="{MonitorFile.SCRIPT}"></script></body>'
        )
        (self.directory / MonitorFile.PAGE).write_text(page)
        (self.directory / MonitorFile.SCRIPT).write_text(
            Path(__file__).with_name(MonitorFile.SCRIPT).read_text()
        )

    def get_mesh_poses(self) -> list[MeshPose]:
        """
        Read the visual transforms without replacing the browser scene.
        """
        return [
            MeshPose(
                node_name=f"{body.name.name}_{index}",
                matrix=(
                    self.world.compute_forward_kinematics_np(self.world.root, body)
                    @ geometry.origin.to_np()
                )
                .T.reshape(-1)
                .tolist(),
            )
            for body in self.world.bodies
            for index, geometry in enumerate(body.visual)
        ]

    def write(self, snapshot: MonitorSnapshot) -> None:
        """
        Atomically publish one serialized snapshot to the local browser.
        """
        destination = self.directory / MonitorFile.SNAPSHOT
        temporary = destination.with_suffix(MonitorFile.TEMPORARY_SUFFIX)
        temporary.write_text(
            json.dumps(DataclassJSONSerializer.to_json(snapshot), allow_nan=False)
        )
        temporary.replace(destination)
