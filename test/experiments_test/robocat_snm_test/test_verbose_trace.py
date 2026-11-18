"""
Verbose entry tracing is explicit, bounded and exposes stopping evidence.
"""

# %% imports
from experiments.robocat_snm.application import EntryOptions
from experiments.robocat_snm.verbose import EntryTrace, SectionTrace, TraceLogger
from experiments.robocat_wall import WallSample, WallStopSettings


# %% explicit CLI and output gating
def test_verbose_is_opt_in() -> None:
    """
    The regular command retains quiet output by default.
    """
    assert EntryOptions.parse([]).verbose is False
    assert EntryOptions.parse(["--verbose"]).verbose is True


def test_verbose_trace_reports_limits_timing_and_unknown_sections(capsys) -> None:
    """
    A failure snapshot contains the quantities required to identify its guard.
    """
    settings = WallStopSettings(0.936)
    wall = WallSample(
        False,
        "Front wall changed discontinuously",
        10.0,
        10.1,
        distance=0.95,
        error=0.014,
        distance_jump=0.12,
        normal_change=0.02,
    )
    trace = EntryTrace(
        observed_at=10.2,
        scan_stamp=10.0,
        wheel_stamp=10.1,
        wall=wall,
        wall_settings=settings,
        sections=[SectionTrace(1.2, None), SectionTrace(1.3, False)],
    )
    logger = TraceLogger()
    logger.emit(trace, now=1.0, problem="stopping")
    text = capsys.readouterr().out
    assert "distance_jump=0.1200" in text
    assert f"limit={settings.maximum_plane_jump:.4f}" in text
    assert "scan_age=0.200" in text
    assert "unknown_sections=[1.200]" in text
    assert "blocked_sections=[1.300]" in text
    logger.emit(trace, now=1.1, problem="stopping")
    assert capsys.readouterr().out == ""
    logger.emit(trace, now=1.2, problem="failed", force=True)
    assert "VERBOSE" in capsys.readouterr().out
