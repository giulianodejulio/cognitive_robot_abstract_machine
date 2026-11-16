"""
Supervise native Nav2 through the host's authenticated SSH connection.
"""

# %% imports
from __future__ import annotations

import os
import select
import shlex
import subprocess
import time
from dataclasses import dataclass, field
from importlib.resources import files
from pathlib import Path

from experiments.robocat_snm.exceptions import EntryError
from experiments.robocat_snm.native_navigation import NavigationStatus
from experiments.robocat_waypoints.planner import PlannerProcess
from semantic_digital_twin.spatial_types import Pose


# %% native action transport
@dataclass
class NavigationConnection:
    """
    Retain action ownership and a watchdog until the entry sequence finishes.
    """

    staging: Pose
    """Desired staging pose in map coordinates."""
    execute: bool = False
    """
    Whether native navigation may submit a real motion goal.
    """

    confirm_navigation_idle: bool = False
    """
    Operator confirmation for read-only preflight when Nav2 status is unavailable.
    """

    bootstrap_navigation: bool = False
    """
    Explicit first-goal authorization requiring current supervisor interlocks.
    """

    socket: Path = Path("/workspace/cram/.robocat-planner/ssh.sock")
    """
    Host-authenticated SSH master socket shared with the container.
    """

    robot: str = "pal@10.68.0.1"
    """
    Native robot identity authenticated by the existing master connection.
    """

    timeout: float = 180.0
    """
    Maximum duration of native staging navigation, in seconds.
    """

    process: subprocess.Popen[bytes] | None = field(default=None, init=False)
    """
    Owned SSH process, not a velocity publisher.
    """

    buffer: bytes = field(default=b"", init=False)
    """
    Partial output line awaiting its terminator.
    """

    status: NavigationStatus = field(default=NavigationStatus.WAITING, init=False)
    """
    Latest correlated navigation lifecycle state.
    """

    idle: bool = field(default=False, init=False)
    """
    Whether the native status stream reports no active goals.
    """

    received_at: float = field(default=0, init=False)
    """
    Monotonic receipt time of navigation ownership information.
    """

    maximum_age: float = 1.0
    """
    Maximum heartbeat age before navigation ownership becomes unknown.
    """

    routing_ready: bool = field(default=False, init=False)
    """
    Native mux configuration was explicitly verified by the supervising agent.
    """

    def start(self) -> None:
        """
        Open native action supervision without copying credentials or robot files.
        """
        if not self.socket.is_socket():
            raise EntryError(
                "Run bash scripts/connect_robocat_planner.sh on the laptop host first"
            )
        package = files("experiments.robocat_snm")
        command = [
            "bash",
            "-c",
            package.joinpath("native_remote.sh").read_text(),
            "bash",
            package.joinpath("native_navigation.py").read_text(),
            "--x",
            str(float(self.staging.x)),
            "--y",
            str(float(self.staging.y)),
            "--yaw",
            str(float(self.staging.yaw)),
            "--timeout",
            str(self.timeout),
        ]
        if self.execute:
            command.append("--execute")
        if self.bootstrap_navigation and self.execute:
            command.append("--bootstrap-navigation")
        if self.confirm_navigation_idle and not self.execute:
            command.append("--confirm-navigation-idle")
        self.process = subprocess.Popen(
            [
                "ssh",
                "-S",
                str(self.socket),
                "-o",
                "BatchMode=yes",
                "-o",
                "StrictHostKeyChecking=yes",
                "-o",
                "ControlMaster=no",
                "-o",
                "ConnectTimeout=3",
                self.robot,
                shlex.join(command),
            ],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
        )

    def poll(self, bootstrap_ready: bool = False) -> bool:
        """
        Exchange heartbeats; lost transport never counts as an idle Nav2 stack.
        """
        process = self.process
        if process is None or process.poll() is not None:
            self.idle = False
            if process is not None:
                self.status = NavigationStatus.FAILED
            return False
        commands = [NavigationStatus.HEARTBEAT]
        if self.bootstrap_navigation and self.execute:
            commands.append(
                NavigationStatus.BOOTSTRAP_READY
                if bootstrap_ready
                else NavigationStatus.BOOTSTRAP_BLOCKED
            )
        process.stdin.write(("\n".join(commands) + "\n").encode())
        process.stdin.flush()
        while select.select([process.stdout], [], [], 0)[0]:
            chunk = os.read(process.stdout.fileno(), 4096)
            if not chunk:
                break
            self.buffer += chunk
            while b"\n" in self.buffer:
                line, _, self.buffer = self.buffer.partition(b"\n")
                text = line.decode().strip()
                if text not in NavigationStatus:
                    print(f"NATIVE STATUS: {text!r}", flush=True)
                    continue
                status = NavigationStatus(text)
                if status in (NavigationStatus.IDLE, NavigationStatus.BUSY):
                    self.idle = status is NavigationStatus.IDLE
                    self.received_at = time.monotonic()
                elif status is NavigationStatus.ROUTING_READY:
                    self.routing_ready = True
                else:
                    if status is not self.status:
                        print(f"NATIVE STATUS: {text!r}", flush=True)
                    self.status = status
        return time.monotonic() - self.received_at <= self.maximum_age

    def close(self) -> None:
        """
        Close the watchdog and allow bounded cancellation of the owned native goal.
        """
        process = self.process
        if process is None:
            return
        if process.poll() is None:
            process.stdin.close()
            deadline = time.monotonic() + 6
            while process.poll() is None and time.monotonic() < deadline:
                time.sleep(0.05)
        PlannerProcess(process).stop()
        process.stdout.close()
