"""
Provide a finite native-status peer for local pipe transport tests.
"""

# %% imports
import sys

# %% owned pipe lifecycle
if __name__ == "__main__":
    print("succeeded\nidle\nrouting_ready", flush=True)
    for command in sys.stdin:
        if command.strip() == "heartbeat":
            print("succeeded\nidle\nrouting_ready", flush=True)
