#!/usr/bin/env python3
"""Thin wrapper -- the implementation is the installed node so
`ros2 run mongla_vision water_check` and this script cannot drift.
"""
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..',
                                'src', 'mongla_vision'))
from mongla_vision.utils.water_check import main

if __name__ == '__main__':
    sys.exit(main())
