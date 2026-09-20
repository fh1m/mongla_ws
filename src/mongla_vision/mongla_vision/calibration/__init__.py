"""Camera calibration as a VEHICLE capability, not a bench script.

⛔ WHY THIS IS IN THE PACKAGE AND NOT IN `tools/`. On competition ground we
may have to recalibrate a camera -- a lens knocked, a unit swapped, a spare
fitted -- and if we cannot, the vision uplink aims with the wrong focal
length and the downward camera's velocity scale is wrong by the same factor.
That is a mission capability, so it ships with the package, is installed by
`colcon`, is covered by the package's tests, and is reachable the way every
other subsystem is:

    ros2 run mongla_vision calibrate --device 3

Two modules:
  `solver` -- the maths. Detection, the two fitting arms, k-fold held-out
              scoring, Max ERE and next-best-pose.
  `guide`  -- the operator-facing browser page that drives a capture.

Both exist because unguided calibration measurably does not work for a
non-expert: Richardson et al. (AprilCal, IROS 2013) measured novices at
0.229 px mean / 1.651 px WORST with guidance against 0.728 / 38.646 without.
The worst-case number is the one that breaks a bearing, and it differs by
23x.
"""
