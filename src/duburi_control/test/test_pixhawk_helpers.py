"""Static-method tests for the Pixhawk class.

These don't touch a real autopilot -- they cover the two pure helpers
that the rest of the stack relies on (PWM conversion + heading-error
wrap).
"""

import pytest

from duburi_control.pixhawk import Pixhawk


def test_percent_to_pwm_neutral():
    assert Pixhawk.percent_to_pwm(0) == 1500


def test_percent_to_pwm_full_range():
    assert Pixhawk.percent_to_pwm(+100) == 1900
    assert Pixhawk.percent_to_pwm(-100) == 1100


def test_percent_to_pwm_clamps():
    assert Pixhawk.percent_to_pwm(+999) == 1900
    assert Pixhawk.percent_to_pwm(-999) == 1100


def test_heading_error_zero():
    assert Pixhawk.heading_error(90.0, 90.0) == pytest.approx(0.0)


def test_heading_error_short_path_positive():
    assert Pixhawk.heading_error(10.0, 350.0) == pytest.approx(20.0)


def test_heading_error_short_path_negative():
    assert Pixhawk.heading_error(350.0, 10.0) == pytest.approx(-20.0)


def test_heading_error_180_boundary():
    # 180 deg apart -> the wrap returns -180 (the formula is
    # (delta + 540) % 360 - 180; 540 % 360 == 180 -> 180 - 180 == 0 -> -180).
    # We pin the sign here so a refactor doesn't accidentally flip it.
    assert Pixhawk.heading_error(180.0, 0.0) == pytest.approx(-180.0)
