"""Outcome constants for Duburi FSM states.

Re-exports YASMIN 5.x basic_outcomes under Mongla naming.
Use these everywhere — never hardcode the string literals.
"""
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, FAIL as FAILED, TIMEOUT

__all__ = ['SUCCEED', 'FAILED', 'TIMEOUT', 'ABORT']
