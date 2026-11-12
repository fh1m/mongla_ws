# shellcheck shell=bash disable=SC1090
# Kutum's shell environment: ROS 2 Jazzy, the deployed workspace, and the few
# variables the stack reads. Sourced from ~/.zshrc and ~/.bashrc (kutum/setup.sh
# adds the line), so every terminal on the vehicle is the same terminal.
#
#   source ~/mongla_ws/kutum/env.sh
#
# Works in zsh and bash. The login shell on the vehicle is zsh, and ROS ships a
# setup file per shell: sourcing setup.bash from zsh half-works and fails later.

if [ -n "${ZSH_VERSION:-}" ]; then _kutum_sh=zsh; else _kutum_sh=bash; fi

source "/opt/ros/jazzy/setup.$_kutum_sh"
[ -f ~/mongla_ws/install/setup.$_kutum_sh ] && source ~/mongla_ws/install/setup.$_kutum_sh
unset _kutum_sh

# XFeat's HEF and ONNX live outside the workspace, so a clean build cannot
# delete them. Read by detection/yolo.py and lock_setup.py.
export MONGLA_HEF_DIR=~/hailo_models

# HailoRT writes hailort.log into whatever directory a process starts in, which
# left copies in $HOME and in the workspace. One place instead.
export HAILORT_LOGGER_PATH=~/.hailo/hailort

# Where the HailoRT service listens (kutum/system/systemd/hailort.service). The
# library reads it once, when it loads.
export HAILORT_SERVICE_ADDRESS=unix:///run/hailo/hailort_uds.sock
