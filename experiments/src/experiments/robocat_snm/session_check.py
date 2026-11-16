"""
Refuse controller preparation while an existing Giskard server is present.
"""

# %% imports
from contextlib import ExitStack
import time

import rclpy


# %% read-only local graph check
def main() -> None:
    """
    Inspect the isolated graph without publishing commands or changing robot state.
    """
    with ExitStack() as cleanup:
        rclpy.init()
        cleanup.callback(rclpy.shutdown)
        node = rclpy.create_node("robocat_session_check")
        cleanup.callback(node.destroy_node)
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        names = node.get_node_names()
        services = node.get_service_names_and_types()
        if "giskard" in names or any(
            name.startswith("/giskard/command/") for name, _ in services
        ):
            print(
                "An existing Giskard server is present. Stop the previous server/client before preparing the controller.",
                flush=True,
            )
            raise SystemExit(1)
        print("Local check: no Giskard server discovered; no goal sent.", flush=True)


if __name__ == "__main__":
    main()
