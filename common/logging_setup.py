"""common/logging_setup.py — shared logging bootstrap for all bridges.

Every bridge process must log to both stdout and a module-local .log file.
This is framework-only code: no business logic belongs here.
"""

import logging
from pathlib import Path

_FORMAT = "%(asctime)s [%(levelname)s] %(name)s: %(message)s"


def setup_logger(name: str, logfile: str) -> logging.Logger:
    """Configure and return a logger that writes to stdout and `logfile`.

    Safe to call more than once with the same name: handlers are only
    attached the first time.
    """
    logger = logging.getLogger(name)
    if logger.handlers:
        return logger

    logger.setLevel(logging.INFO)
    formatter = logging.Formatter(_FORMAT)

    stream_handler = logging.StreamHandler()
    stream_handler.setFormatter(formatter)
    logger.addHandler(stream_handler)

    log_path = Path(logfile)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    file_handler = logging.FileHandler(log_path, encoding="utf-8")
    file_handler.setFormatter(formatter)
    logger.addHandler(file_handler)

    logger.propagate = False
    return logger
