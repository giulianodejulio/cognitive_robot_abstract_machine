# %% imports

from __future__ import annotations

import argparse
from contextlib import ExitStack
from dataclasses import dataclass
from enum import StrEnum
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from threading import Thread

import rclpy

from experiments.tiago_monitor.observer import MonitorName, RobotObserver
from experiments.tiago_monitor.view import RobotView

# %% command line


class MonitorOption(StrEnum):
    """
    Supported command-line options for the read-only monitor.
    """

    PORT = "--port"
    DIRECTORY = "--directory"
    MAXIMUM_AGE = "--maximum-age"
    PERIOD = "--period"


@dataclass
class MonitorOptions:
    """
    The local server and observation settings.
    """

    port: int = 8766
    """
    The loopback HTTP port.
    """

    directory: Path = Path("/tmp/tiago-monitor")
    """
    The generated browser files.
    """

    maximum_age: float = 2.0
    """
    The tolerated input timestamp age, in seconds.
    """

    period: float = 0.5
    """
    The time between observation cycles, in seconds.
    """

    @classmethod
    def parse(cls) -> MonitorOptions:
        """
        Read the supported options from the command line.
        """
        defaults = cls()
        parser = argparse.ArgumentParser(description=__doc__)
        parser.add_argument(MonitorOption.PORT, type=int, default=defaults.port)
        parser.add_argument(
            MonitorOption.DIRECTORY, type=Path, default=defaults.directory
        )
        parser.add_argument(
            MonitorOption.MAXIMUM_AGE, type=float, default=defaults.maximum_age
        )
        parser.add_argument(MonitorOption.PERIOD, type=float, default=defaults.period)
        options = parser.parse_args()
        if options.period <= 0 or options.maximum_age <= 0:
            parser.error("period and maximum-age must be positive")
        return cls(
            port=options.port,
            directory=options.directory,
            maximum_age=options.maximum_age,
            period=options.period,
        )


# %% process lifecycle


def main() -> None:
    """
    Serve a read-only robot view and receive ROS inputs until interrupted.
    """
    options = MonitorOptions.parse()
    with ExitStack() as cleanup:
        rclpy.init()
        cleanup.callback(rclpy.try_shutdown)
        node = rclpy.create_node(MonitorName.NODE)
        cleanup.callback(node.destroy_node)
        observer = RobotObserver(node=node, maximum_age=options.maximum_age)
        node.get_logger().info(
            f"Publisher topics: {node.get_publisher_names_and_types_by_node(node.get_name(), node.get_namespace())}"
        )
        view = RobotView(world=observer.world, directory=options.directory)
        view.create()
        view.write(observer.observe(view))
        server = ThreadingHTTPServer(
            ("127.0.0.1", options.port),
            partial(SimpleHTTPRequestHandler, directory=str(options.directory)),
        )
        cleanup.callback(server.server_close)
        Thread(target=server.serve_forever, daemon=True).start()
        cleanup.callback(server.shutdown)
        node.create_timer(options.period, lambda: view.write(observer.observe(view)))
        node.get_logger().info(f"Read-only viewer: http://127.0.0.1:{options.port}/")
        rclpy.spin(node)


if __name__ == "__main__":
    main()
