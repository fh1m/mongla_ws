# Mongla vision box -- Raspberry Pi 5 + AI HAT+ (Hailo-8), Ubuntu 24.04 / ROS 2 Jazzy
# Source this in every terminal.  `source ~/pi_env.sh`
#
# THIS FILE IS THE SETUP, NOT A CONVENIENCE. Two of the three lines below exist
# because of failures that are silent, not loud.

source /opt/ros/jazzy/setup.bash
[ -f ~/duburi_ws/install/setup.bash ] && source ~/duburi_ws/install/setup.bash

# Hailo models live outside the workspace so a rebuild cannot delete them.
# A .hef carries NO class names -- without the .yaml sidecar beside it the
# allowlist is empty and the detector returns [] on EVERY frame, with one
# warning and no error.
export DUBURI_HEF_DIR=~/hailo_models

# The HEFs are baked at nms_scores_th=0.05 so runtime conf can be TIGHTENED
# (it can never be loosened below the baked value). INT8 scores ~0.08 lower
# than fp32, measured, so run 0.12-0.15 where the Jetson path uses 0.20.
export DUBURI_HAILO_CONF=0.15

# Camera: 640x360 @ 210 fps MJPG.
#   - the fps is a SETTING, not a bandwidth limit: 30->30 Hz, 210->125 Hz
#   - 640x360 has the SAME field of view as 1280x720 (measured: scale 1.01,
#     corr 0.98) and the net input is a 640x640 letterbox either way, so 720p
#     was being downscaled to 640x360 anyway -- it cost half the frame rate
#     for nothing.
#   - THESE NUMBERS CAME FROM THE BENCH MICRODIA. The vehicle's Blue Robotics
#     cameras are unmeasured; re-measure before trusting 210 there.
export DUBURI_CAM_W=640 DUBURI_CAM_H=360 DUBURI_CAM_FPS=210
