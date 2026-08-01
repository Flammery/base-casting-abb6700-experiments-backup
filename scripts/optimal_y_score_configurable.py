"""Backward-compatible entry point for the configurable experiment runner.

The implementation moved to ``configurable_experiment_runner.py``. Historical
commands may keep using this filename while the UI and new integrations use the
generic runner name.
"""

from configurable_experiment_runner import *  # noqa: F401,F403
from configurable_experiment_runner import main


if __name__ == "__main__":
    main()
