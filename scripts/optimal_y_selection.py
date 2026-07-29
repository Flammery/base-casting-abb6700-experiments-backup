"""Compatibility imports for the renamed mixed candidate-selection module.

The historical Optimal-Y metric remains ``score_max_abs_world_y``. General
ordinary/avoidance selection now lives in ``candidate_selection.py``.
"""

from candidate_selection import *  # noqa: F401,F403
