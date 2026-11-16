"""
Keep running when interrupted to exercise inconclusive cancellation.
"""

# %% imports
import signal

# %% unacknowledged cancellation
signal.signal(signal.SIGINT, signal.SIG_IGN)
print("READY", flush=True)
while True:
    signal.pause()
