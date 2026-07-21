"""Make the kinematics prototype package importable in tests without install."""

import os
import sys

sys.path.insert(
    0,
    os.path.join(os.path.dirname(__file__), "kinematics", "src"),
)
