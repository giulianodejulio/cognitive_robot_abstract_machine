"""
Terminal emphasis must never contaminate recorded ROS event messages.
"""

# %% imports
from unittest.mock import Mock

import pytest

from experiments.robocat_snm import application
from experiments.robocat_snm.application import EntryEvents, TerminalColor


# %% terminal and recording output
def test_problem_is_red_only_in_terminal(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    """
    An interactive error is red while its published event remains plain text.
    """
    monkeypatch.setattr(application.sys.stdout, "isatty", lambda: True)
    publish = Mock()
    events = EntryEvents(publish)
    events.report("laser feedback stale", problem=True)
    plain = publish.call_args.args[0].data
    assert (
        capsys.readouterr().out == f"{TerminalColor.RED}{plain}{TerminalColor.RESET}\n"
    )
    assert "\x1b" not in plain


def test_redirected_problem_has_no_ansi(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    """
    Redirected logs remain readable without terminal escape sequences.
    """
    monkeypatch.setattr(application.sys.stdout, "isatty", lambda: False)
    publish = Mock()
    EntryEvents(publish).report("laser feedback stale", problem=True)
    assert capsys.readouterr().out == publish.call_args.args[0].data + "\n"


def test_ordinary_event_keeps_default_color(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    """
    Healthy state transitions do not acquire error formatting.
    """
    monkeypatch.setattr(application.sys.stdout, "isatty", lambda: True)
    publish = Mock()
    EntryEvents(publish).report("Entry conditions stable")
    assert capsys.readouterr().out == publish.call_args.args[0].data + "\n"
