"""
Cancel a forward goal, wait for rest, optionally turn, then advance again.
"""

# %% imports
from __future__ import annotations

import argparse
import math
import signal
import subprocess
import time
from contextlib import ExitStack
from dataclasses import dataclass, field
from pathlib import Path


# %% lightweight orchestration of the persistent client
@dataclass
class GoalSequence:
    """
    Run cancellable relative movements through the existing foreground sender.
    """

    fifo: Path = Path("/tmp/tiago-giskard-goals.fifo")
    """
    The request pipe of the already running persistent client.
    """

    cancel_after: float = 0.4
    """
    Delay after the first movement feedback before sending SIGINT.
    """

    first_distance: float = 0.5
    """
    Forward distance requested before cancellation, in meters.
    """

    second_distance: float = 1.2
    """
    Forward distance requested after the optional turn, in meters.
    """

    turn_degrees: float = 0
    """
    Optional relative turn between translations; positive means left.
    """

    pure_turn: bool = False
    """
    Whether the optional turn requires zero commanded forward velocity.
    """

    ready_timeout: float = 10
    """
    Maximum time to await settled wheels and released interlocks.
    """

    sender: Path = field(
        default_factory=lambda: Path(__file__).resolve().with_name("tiago-goal")
    )
    """
    Existing sender reused for requests, cancellation and readiness checks.
    """

    active_process: subprocess.Popen | None = field(default=None, init=False)
    """
    The command currently owned by this sequence.
    """

    @classmethod
    def parse(cls, arguments: list[str] | None = None) -> GoalSequence:
        """
        Require explicit execution and validate cancellation timing.
        """
        defaults = cls()
        parser = argparse.ArgumentParser(description=__doc__)
        parser.add_argument(
            "--execute", action="store_true", help="Execute the real movement sequence"
        )
        parser.add_argument("--fifo", type=Path, default=defaults.fifo)
        parser.add_argument(
            "--cancel-after",
            type=float,
            default=defaults.cancel_after,
            metavar="SECONDS",
            help="Seconds after wheel movement feedback (default: %(default)s)",
        )
        parser.add_argument(
            "--first-distance", type=float, default=defaults.first_distance
        )
        parser.add_argument(
            "--second-distance", type=float, default=defaults.second_distance
        )
        parser.add_argument(
            "--turn-degrees",
            type=float,
            default=defaults.turn_degrees,
            help="Turn before the second translation; positive left (default: %(default)s)",
        )
        parser.add_argument(
            "--pure-turn",
            action="store_true",
            help="Experiment: require zero longitudinal velocity during the turn",
        )
        options = parser.parse_args(arguments)
        if not options.execute:
            parser.error(
                "--execute requires an operator present, clear area and accessible emergency stop"
            )
        if not math.isfinite(options.cancel_after) or options.cancel_after <= 0:
            parser.error("--cancel-after must be positive and finite")
        if any(
            not math.isfinite(distance) or distance <= 0
            for distance in (options.first_distance, options.second_distance)
        ):
            parser.error("Distances must be positive and finite")
        if not math.isfinite(options.turn_degrees) or abs(options.turn_degrees) > 180:
            parser.error("--turn-degrees must be finite and within +/-180 degrees")
        if options.pure_turn and options.turn_degrees == 0:
            parser.error("--pure-turn requires nonzero --turn-degrees")
        return cls(
            fifo=options.fifo,
            cancel_after=options.cancel_after,
            first_distance=options.first_distance,
            second_distance=options.second_distance,
            turn_degrees=options.turn_degrees,
            pure_turn=options.pure_turn,
        )

    def run_command(self, arguments: list[str]) -> int:
        """
        Run the existing CLI with separate signal ownership.
        """
        quiet = arguments == ["--ready"]
        self.active_process = subprocess.Popen(
            [str(self.sender), *arguments, "--fifo", str(self.fifo)],
            start_new_session=True,
            stdout=subprocess.DEVNULL if quiet else None,
            stderr=subprocess.STDOUT if quiet else None,
        )
        return self.active_process.wait()

    def cancel_active(self) -> None:
        """
        Forward interruption to the owned command and await its cancel reply.
        """
        if self.active_process is not None and self.active_process.poll() is None:
            self.active_process.send_signal(signal.SIGINT)
            self.active_process.wait(timeout=12)

    def wait_for_rest(self) -> None:
        """
        Require settled wheel feedback and released interlocks before continuing.
        """
        deadline = time.monotonic() + self.ready_timeout
        while True:
            result = self.run_command(["--ready"])
            if result == 0:
                return
            if result != 3:
                raise RuntimeError(
                    f"Readiness query failed ({result}); next goal not sent"
                )
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    "Wheels/interlocks did not become ready; next goal not sent"
                )
            time.sleep(0.1)

    def run(self) -> None:
        """
        Continue only after expected cancellation and measured stable rest.
        """
        with ExitStack() as cleanup:
            cleanup.callback(self.cancel_active)
            print(
                f"STEP 1: {self.first_distance:g} m goal; automatic Ctrl+C after wheel movement",
                flush=True,
            )
            result = self.run_command(
                [f"{self.first_distance:g}", "--cancel-after", f"{self.cancel_after:g}"]
            )
            if result != 130:
                raise RuntimeError(
                    f"Expected confirmed cancellation (130), received {result}; second goal not sent"
                )
            print(
                "WAIT: cancellation confirmed; waiting for settled wheels and free mux",
                flush=True,
            )
            self.wait_for_rest()
            if self.turn_degrees:
                print(
                    f"TURN: {self.turn_degrees:g} deg from the reached heading",
                    flush=True,
                )
                turn_request = ["--turn", f"{self.turn_degrees:g}"]
                if self.pure_turn:
                    turn_request.append("--pure-turn")
                result = self.run_command(turn_request)
                if result != 0:
                    raise RuntimeError(
                        f"Turn did not succeed ({result}); forward goal not sent"
                    )
                self.wait_for_rest()
            print(
                f"STEP 2: {self.second_distance:g} m forward from the reached pose",
                flush=True,
            )
            result = self.run_command([f"{self.second_distance:g}"])
            if result != 0:
                raise RuntimeError(f"Second goal did not succeed ({result})")
            print("SEQUENCE SUCCESS; server and client remain active", flush=True)


if __name__ == "__main__":
    GoalSequence.parse().run()
