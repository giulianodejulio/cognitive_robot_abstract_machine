"""
Keep expected startup notices quiet without hiding operational diagnostics.
"""

# %% imports
from io import StringIO
import logging

from experiments.robocat_startup import RobocatStartupOutput


# %% selective startup output
def test_known_parser_extensions_are_quiet_but_other_messages_remain() -> None:
    """
    Suppress only the known Robocat parser notices.
    """
    messages = RobocatStartupOutput()
    hidden = "Unknown tag \"ros2_control\" in /robot[@name='robocat']"
    visible = "Unknown tag \"unexpected\" in /robot[@name='robocat']"
    messages.output.write(hidden + "\n" + visible + "\n")
    destination = StringIO()
    messages.emit(destination)
    assert destination.getvalue() == visible + "\n"


def test_missing_optional_database_notice_only_is_filtered() -> None:
    """
    Other Giskard warnings and errors remain visible.
    """
    messages = RobocatStartupOutput()
    optional = logging.LogRecord(
        "giskard", logging.WARNING, "", 0, messages.database_notice, (), None
    )
    operational = logging.LogRecord(
        "giskard", logging.ERROR, "", 0, "Goal aborted: QP is infeasible", (), None
    )
    assert messages.filter(optional) is False
    assert messages.filter(operational) is True
