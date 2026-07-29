"""Compatibility entry point for the renamed configurable experiment runner.

New UI, documentation, and integrations should use
``configurable_experiment_runner.py``. This wrapper remains so historical
commands and external scripts continue to work without changing behavior.
"""

from configurable_experiment_runner import *  # noqa: F401,F403
from configurable_experiment_runner import main


if __name__ == "__main__":
    main()
