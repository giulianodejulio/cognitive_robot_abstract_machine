#!/opt/ros/cram-env/bin/python
"""
Record recovery orchestration against isolated command substitutes.
"""

# %% imports
import json
import os
import subprocess
import sys
from pathlib import Path

# %% command substitute
command = Path(sys.argv[0]).name
arguments = sys.argv[1:]
directory = Path(os.environ["ROBOCAT_RECOVERY_TEST"])
with (directory / "commands.jsonl").open("a") as output:
    output.write(json.dumps([command, arguments]) + "\n")
if command == "byobu-tmux":
    if arguments[0] == "show-option":
        print(directory / "infra")
    elif arguments[0] == "respawn-pane":
        (directory / "infra/giskard.ready").touch()
    else:
        sys.exit(1)
elif command == "docker":
    if "flock" in arguments:
        print("RECOVERY_LOCKED", flush=True)
        sys.stdin.read()
    elif "scripts/robocat_stop_owned_process.sh" in arguments:
        (directory / "infra/giskard.closed").touch()
    else:
        print(
            "python\nscripts/robocat_giskard_bringup.py\n--max-linear\n0.15\n--max-angular\n0.2"
        )
elif command == "ssh":
    if "-O" in arguments:
        sys.exit(0)
    words = (
        subprocess.check_output(
            [
                "bash",
                str(Path(sys.argv[0]).with_name("recovery_arguments.sh")),
                arguments[-1],
            ]
        )
        .decode()
        .rstrip("\0")
        .split("\0")
    )
    script = words[2]
    operation = words[4]
    if "motor_recovery" in script:
        sys.stdin.buffer.read()
        if operation == "settings":
            print("NAV2_SETTINGS: 0.03 0.0872665")
        if operation == "drivers":
            counter = directory / "driver_checks"
            count = int(counter.read_text()) if counter.exists() else 0
            counter.write_text(str(count + 1))
            if count == 0 and os.environ.get("ROBOCAT_RECOVERY_RESTART") == "yes":
                sys.exit(1)
        if os.environ.get("ROBOCAT_RECOVERY_FAIL") == operation:
            sys.exit(1)
    elif "native_preparation" in script:
        sys.stdin.buffer.read()
        if os.environ.get("ROBOCAT_RECOVERY_FAIL") == "preparation":
            sys.exit(1)
    elif "exec pal module" in script:
        if os.environ.get("ROBOCAT_RECOVERY_FAIL") == "pal":
            sys.exit(1)
    else:
        sys.exit(1)
else:
    sys.exit(1)
