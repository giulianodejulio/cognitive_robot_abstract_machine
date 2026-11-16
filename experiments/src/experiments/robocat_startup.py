"""Select expected Robocat startup notices without hiding operational errors."""

# %% imports
from __future__ import annotations

import logging
import re
from dataclasses import dataclass, field
from io import StringIO

from typing_extensions import TextIO


# %% expected startup notices
@dataclass
class RobocatStartupOutput(logging.Filter):
    """Preserve startup diagnostics except known optional model notices."""

    output: StringIO = field(default_factory=StringIO)
    """Captured parser output during server setup only."""

    database_notice: str = (
        "Model reload synchronization not available because "
        '"SEMANTIC_DIGITAL_TWIN_DATABASE_URI" is not set.'
    )
    """Expected message when optional database synchronization is not configured."""

    parser_notice: str = (
        r'''^Unknown (?:tag "(?:material|kinematic|gravity|ros2_control)"'''
        r"""|attribute "(?:iyx|izx|izy|type)") in /robot\[@name='robocat'\]"""
    )
    """Known ignored Robocat URDF fields; unexpected parser messages remain visible."""

    def filter(self, record: logging.LogRecord) -> bool:
        """Hide only the expected optional-database warning."""
        return not (
            record.levelno == logging.WARNING
            and record.getMessage() == self.database_notice
        )

    def emit(self, destination: TextIO) -> None:
        """Flush unexpected captured diagnostics, including on setup failure."""
        for line in self.output.getvalue().splitlines(keepends=True):
            if re.match(self.parser_notice, line) is None:
                destination.write(line)
        destination.flush()
