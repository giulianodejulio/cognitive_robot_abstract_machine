"""Keep relevant DDS diagnostics visible during preparation."""

# %% imports
import subprocess
from pathlib import Path


# %% optional interface notice
def test_only_missing_optional_wlan_notice_is_removed() -> None:
    """Filter the known optional interface notice without hiding real errors."""
    root = Path(__file__).parents[3]
    script = root / "experiments/src/experiments/robocat_snm/preparation_log.sed"
    result = subprocess.run(
        ["sed", "-u", "-f", str(script)],
        input=(
            b"1794485330.805447 [102] python3: wlan0: optional interface was not found.\n"
            b"DDS error: connection lost\n"
            b"eth0: required interface was not found.\n"
        ),
        capture_output=True,
        check=False,
    )
    assert result.returncode == 0
    assert result.stdout == (
        b"DDS error: connection lost\n" b"eth0: required interface was not found.\n"
    )
