"""vision_thrust_check names the vision verb's result codes from a copy.

mongla_vision does not depend on mongla_control, so the copy stays; this
keeps it equal to the table the verb itself reports with.
"""
from mongla_control.motion_vision import control
from mongla_vision.utils import check_thrust


def test_the_copy_equals_the_verbs_own_table():
    assert check_thrust.OUTCOME == control._CODE_NAME
