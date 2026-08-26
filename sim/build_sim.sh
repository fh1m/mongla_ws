#!/usr/bin/env bash
# Build the Mongla simulator workspace (duburi_ws/sim).
#
# Why --base-paths src, and not a bare `colcon build`:
#   sim/COLCON_IGNORE exists so that `colcon build` / `colcon test` run from the
#   duburi_ws root skip this subtree entirely -- the autonomy workspace must stay
#   exactly six packages with no Gazebo dependency. But colcon checks the ignore
#   marker against the BASE PATH too (colcon_core/package_identification/ignore.py),
#   so `cd sim && colcon build` would ignore itself. Pointing the base path at
#   src/ steps past the marker. Removing COLCON_IGNORE to "simplify" this
#   re-contaminates the root build -- don't.
#
# The autonomy workspace must be built and sourced first: stack.launch.py
# includes duburi_manager's and duburi_vision's launch files by share directory.
set -e
cd "$(dirname "$0")"

# Autonomy is OPTIONAL for building the simulator and REQUIRED for driving it:
# no duburi_sim_* package build-depends on duburi_ws, but `stack`, `smoke` and
# every mission do. In duburi_ws/sim the parent is always there; in the standalone
# mirror it usually is not, and hard-failing there would block the one build path
# that repo has. So: warn, don't refuse.
DUBURI_WS="${DUBURI_WS:-$(cd .. && pwd)}"
HAVE_AUTONOMY=0
if [ -d "$DUBURI_WS/src/duburi_manager" ]; then
    if [ -f "$DUBURI_WS/install/setup.bash" ]; then
        HAVE_AUTONOMY=1
    else
        echo "warning: autonomy at $DUBURI_WS is not built yet." >&2
        echo "         run ./build_dubomini.sh there before \`duburi_sim stack\`." >&2
    fi
else
    echo "warning: no autonomy workspace at $DUBURI_WS (expected src/duburi_manager)." >&2
    echo "         \`duburi_sim sim\` and the lab will work; \`stack\`, \`smoke\` and" >&2
    echo "         missions will not. Set DUBURI_WS=<path to duburi_ws> if you have one." >&2
fi

# shellcheck disable=SC1090,SC1091
. /opt/ros/humble/setup.bash
if [ "$HAVE_AUTONOMY" = "1" ]; then
    # shellcheck disable=SC1090,SC1091
    . "$DUBURI_WS/install/setup.bash"
fi

colcon build --base-paths src --symlink-install "$@"

echo ""
echo "Sim build done. Source ROS, then autonomy (if present), then this workspace:"
echo "  source /opt/ros/humble/setup.bash"
if [ "$HAVE_AUTONOMY" = "1" ]; then echo "  source $DUBURI_WS/install/setup.bash"; fi
echo "  source $(pwd)/install/setup.bash"
echo "Then:"
echo "  ros2 run duburi_sim_bringup duburi_sim stop"
echo "  ros2 run duburi_sim_bringup duburi_sim sim"
echo "  ros2 run duburi_sim_bringup duburi_sim stack --no-vision"
echo "  ros2 run duburi_sim_bridge contract_check"
