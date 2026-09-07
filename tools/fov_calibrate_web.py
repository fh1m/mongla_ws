#!/usr/bin/env python3
"""Shim. The real thing is `duburi_vision.calibration.guide`.

Calibration moved INTO the package because recalibrating a camera on
competition ground is a mission capability, not a bench errand -- so it
installs with `colcon`, is covered by the package's tests, and is reached
the way every other subsystem is:

    ros2 run duburi_vision calibrate         # the guided capture
    ros2 run duburi_vision calibrate_solve   # the solver alone

This file stays only so an existing command or a running session does not
break mid-calibration. It holds no logic; a second copy of the geometry is
how the simulator's scorer came to grade a board that no longer existed.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'src', 'duburi_vision'))
from duburi_vision.calibration.guide import main   # noqa: E402

if __name__ == '__main__':
    sys.exit(main())
