#!/opt/ros/cram-env/bin/python
"""
Report successful cleanup after the display terminal disappears.
"""

# %% imports
import signal
import sys

# %% detached cleanup output
if "-it" in sys.argv:
    print("TEST TERMINAL READY", flush=True)
    while True:
        signal.pause()
else:
    print("SNM session processes stopped; infrastructure retained.", flush=True)
