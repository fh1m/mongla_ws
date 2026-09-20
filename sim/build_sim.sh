#!/usr/bin/env bash
# Build the Mongla simulator workspace (mongla_ws/sim).
#
# Why --base-paths src, and not a bare `colcon build`:
#   sim/COLCON_IGNORE exists so that `colcon build` / `colcon test` run from the
#   mongla_ws root skip this subtree entirely -- the autonomy workspace must stay
#   exactly six packages with no Gazebo dependency. But colcon checks the ignore
#   marker against the BASE PATH too (colcon_core/package_identification/ignore.py),
#   so `cd sim && colcon build` would ignore itself. Pointing the base path at
#   src/ steps past the marker. Removing COLCON_IGNORE to "simplify" this
#   re-contaminates the root build -- don't.
#
# The autonomy workspace must be built and sourced first: stack.launch.py
# includes mongla_manager's and mongla_vision's launch files by share directory.
set -e
cd "$(dirname "$0")"

# Autonomy is OPTIONAL for building the simulator and REQUIRED for driving it:
# no mongla_sim_* package build-depends on mongla_ws, but `stack`, `smoke` and
# every mission do. In mongla_ws/sim the parent is always there; in the standalone
# mirror it usually is not, and hard-failing there would block the one build path
# that repo has. So: warn, don't refuse.
MONGLA_WS="${MONGLA_WS:-$(cd .. && pwd)}"
HAVE_AUTONOMY=0
if [ -d "$MONGLA_WS/src/mongla_manager" ]; then
    if [ -f "$MONGLA_WS/install/setup.bash" ]; then
        HAVE_AUTONOMY=1
    else
        echo "warning: autonomy at $MONGLA_WS is not built yet." >&2
        echo "         run ./build_mongla.sh there before \`mongla_sim stack\`." >&2
    fi
else
    echo "warning: no autonomy workspace at $MONGLA_WS (expected src/mongla_manager)." >&2
    echo "         \`mongla_sim sim\` and the lab will work; \`stack\`, \`smoke\` and" >&2
    echo "         missions will not. Set MONGLA_WS=<path to mongla_ws> if you have one." >&2
fi

# shellcheck disable=SC1090,SC1091
. /opt/ros/humble/setup.bash
if [ "$HAVE_AUTONOMY" = "1" ]; then
    # shellcheck disable=SC1090,SC1091
    . "$MONGLA_WS/install/setup.bash"
fi

colcon build --base-paths src --symlink-install "$@"

echo ""
echo "Sim build done. Source ROS, then autonomy (if present), then this workspace:"
echo "  source /opt/ros/humble/setup.bash"
if [ "$HAVE_AUTONOMY" = "1" ]; then echo "  source $MONGLA_WS/install/setup.bash"; fi
echo "  source $(pwd)/install/setup.bash"
echo "Then:"
echo "  ros2 run mongla_sim_bringup mongla_sim stop"
echo "  ros2 run mongla_sim_bringup mongla_sim sim"
echo "  ros2 run mongla_sim_bringup mongla_sim stack --no-vision"
echo "  ros2 run mongla_sim_bridge contract_check"
