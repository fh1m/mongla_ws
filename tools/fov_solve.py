#!/usr/bin/env python3
"""Shim. The real thing is `mongla_vision.calibration.solver`.

    ros2 run mongla_vision calibrate         # the guided capture
    ros2 run mongla_vision calibrate_solve   # the solver alone

⛔ LOADS BY PATH, NOT BY PACKAGE IMPORT. The first version did
`from mongla_vision.calibration.solver import main` after putting the source
tree on sys.path, which drags in `mongla_vision/__init__.py` -> `preflight`
-> `rclpy`. Without a sourced ROS that is a ModuleNotFoundError, and it hit
the operator on the Solve button with a full capture set already on disk.
The calibration maths needs cv2 and numpy and nothing else; making it need
ROS was an accident of the import path, not a requirement.

This file holds no logic. A second copy of the geometry is how the
simulator's scorer came to grade a board that no longer existed.
"""
import importlib.util
import os
import sys

_M = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src',
                  'mongla_vision', 'mongla_vision', 'calibration', 'solver.py')
_spec = importlib.util.spec_from_file_location('mongla_calib_solver',
                                               os.path.abspath(_M))
_mod = importlib.util.module_from_spec(_spec)
sys.modules['mongla_calib_solver'] = _mod
_spec.loader.exec_module(_mod)

if __name__ == '__main__':
    sys.exit(_mod.main())
