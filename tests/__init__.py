"""
Test package for humanoid skill simulator
"""

# Test package initialization
__version__ = "1.0.0"
__author__ = "Humanoid Skill Simulator Team"

# Import test modules for easier access
from . import skill_tests
from . import behavior_tree_tests
from . import simulation_tests
from . import scenario_tests
from . import regression_tests

__all__ = [
    'skill_tests',
    'behavior_tree_tests', 
    'simulation_tests',
    'scenario_tests',
    'regression_tests'
]
