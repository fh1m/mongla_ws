#!/usr/bin/env bash
# Build duburi packages and fix two Debian-vs-ament install quirks:
#   1. pip installs to local/lib/.../dist-packages -> copy to lib/.../site-packages
#   2. Scripts land in bin/ -> symlink into lib/<pkg>/ for ros2 run
#
# Build order matters: duburi_interfaces first (ament_cmake, generates Move.action),
# then the Python packages (control, sensors, manager, planner) which depend on
# the generated types and on duburi_control's COMMANDS registry.
set -e
cd "$(dirname "$0")"

source /opt/ros/humble/setup.bash 2>/dev/null || true

# Wipe stale Python package dirs so colcon sees a clean slate.
# duburi_interfaces is ament_cmake so its install dir is managed correctly by CMake.
for pkg in duburi_control duburi_manager duburi_sensors duburi_planner duburi_vision; do
    rm -rf "install/$pkg" "build/$pkg"
done

# Step 1: build the interface package first so generated types are available
colcon build --packages-select duburi_interfaces "$@"
source install/setup.bash

# Step 2: build the Python packages (control + sensors + manager + planner + vision)
colcon build --packages-select \
    duburi_control duburi_sensors duburi_manager duburi_planner duburi_vision "$@"

INSTALL="$(pwd)/install"
PY=python3.10

for pkg in duburi_control duburi_manager duburi_sensors duburi_planner duburi_vision; do
    PREFIX="$INSTALL/$pkg"
    DIST="$PREFIX/local/lib/$PY/dist-packages"
    SITE="$PREFIX/lib/$PY/site-packages"

    # Fix 1: copy Python package to ament-expected site-packages
    if [ -d "$DIST/$pkg" ]; then
        mkdir -p "$SITE"
        cp -r "$DIST/$pkg" "$SITE/"
    fi

    # Fix 2: expose executables in lib/<pkg>/ for ros2 run
    if [ -d "$PREFIX/bin" ]; then
        mkdir -p "$PREFIX/lib/$pkg"
        for exe in "$PREFIX/bin/"*; do
            [ -f "$exe" ] && ln -sf "$exe" "$PREFIX/lib/$pkg/$(basename "$exe")"
        done
    fi
done

echo ""
echo "Build done. Source with:"
echo "  source install/setup.bash"
echo "Then:"
echo "  ros2 run duburi_manager auv_manager --ros-args -p mode:=sim"
echo "  ros2 run duburi_planner duburi <cmd> [--field value ...]"
echo "  ros2 run duburi_planner mission square_pattern"
echo "Vision (laptop webcam + YOLO26 person detector + viewer):"
echo "  ros2 launch duburi_vision webcam_demo.launch.py"
