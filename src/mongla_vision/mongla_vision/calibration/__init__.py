"""Camera calibration as a VEHICLE capability, not a bench script.

Why this is in the package and not in `tools/`. On competition ground we
may have to recalibrate a camera -- a lens knocked, a unit swapped, a spare
fitted -- and if we cannot, the vision uplink aims with the wrong focal
length and the downward camera's velocity scale is wrong by the same factor.
That is a mission capability, so it ships with the package, is installed by
`colcon`, is covered by the package's tests, and is reachable the way every
other subsystem is:

    ros2 launch mongla_batighor calibrate.launch.py device_path:=<camera>

  `solver`          the maths. Detection, the two fitting arms, k-fold
                    held-out scoring, Max ERE and next-best-pose; also
                    `ros2 run mongla_vision calibrate_solve <captures>`.
  `guide_capture`   the pose script and the detect and assess threads.
  `guide_files`     which camera is which, and where calibrations live.
  `calibrate_node`  the node that runs a capture beside the camera node;
                    Batighor's `calibrate` layout and panel are its page.

The capture is guided because unguided calibration measurably does not work for a
non-expert: Richardson et al. (AprilCal, IROS 2013) measured novices at
0.229 px mean / 1.651 px WORST with guidance against 0.728 / 38.646 without.
The worst-case number is the one that breaks a bearing, and it differs by
23x.
"""
