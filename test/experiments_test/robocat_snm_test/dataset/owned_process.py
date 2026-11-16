"""
Remain idle until the owned process receives an interrupt.
"""

# %% imports
import signal
import sys


# %% graceful interrupt
def interrupted(number: int, frame: object) -> None:
    """
    Report the graceful signal and exit without running robot code.
    """
    print("INTERRUPTED", flush=True)
    sys.exit(0)


signal.signal(signal.SIGINT, interrupted)
print("READY", flush=True)
while True:
    signal.pause()
