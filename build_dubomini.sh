#!/usr/bin/env bash
# Build Dubomini/Mongla packages and fix two Debian-vs-ament install quirks:
#   1. pip installs to local/lib/.../dist-packages -> copy to lib/.../site-packages
#   2. Scripts land in bin/ -> symlink into lib/<pkg>/ for ros2 run
#
# Build order matters: duburi_interfaces first (ament_cmake, generates Move.action),
# then the Python packages (control, sensors, manager, planner) which depend on
# the generated types and on duburi_control's COMMANDS registry.
#
# ── Device-local models + missions sync (kept OUT of git) ────────────────────
# The competition YOLO weights (*.pt/*.engine/*.onnx/*.hef) and the personal test
# missions (rakib_*) change too fast to live in git and are big/private, so the
# source-of-truth for them lives OUTSIDE the repo on the Jetson:
#     ~/models    -> src/duburi_vision/models/   (put the Pi's .hef here too --
#                    the sync copies every regular file, so nothing to change)
#     ~/missions  -> src/duburi_planner/duburi_planner/missions/
# Every build mirrors those folders INTO the tree first, so a fresh `git clone`
# gets them back on the next `./build_dubomini.sh` with nothing to copy by hand.
# Override the source dirs with DUBOMINI_MODELS_SRC / DUBOMINI_MISSIONS_SRC.
# On a dev box without those folders the sync is skipped (build still works).
set -e
cd "$(dirname "$0")"

MODELS_SRC="${DUBOMINI_MODELS_SRC:-$HOME/models}"
MISSIONS_SRC="${DUBOMINI_MISSIONS_SRC:-$HOME/missions}"
MODELS_DST="src/duburi_vision/models"
MISSIONS_DST="src/duburi_planner/duburi_planner/missions"

# sync_dir <src> <dst> <label> [exclude]
#   Copy every regular file from <src> into <dst> (additive; never deletes).
#   Missing <src> is a soft skip (dev box). `exclude` = also append each copied
#   file to .git/info/exclude (per-clone, local) so untracked personal files
#   never show up in `git status` or get committed by accident.
sync_dir() {
    local src="$1" dst="$2" label="$3" do_exclude="${4:-}"
    if [ ! -d "$src" ]; then
        echo "  [skip] $label: '$src' not found (dev box? nothing to sync)"
        return 0
    fi
    mkdir -p "$dst"
    local n=0 f base pat exclude_file=".git/info/exclude"
    # No nullglob needed: an empty dir leaves the literal glob, which fails the
    # `-f` test below, so n stays 0. Keeps this portable across bash/zsh/sh.
    for f in "$src"/*; do
        [ -f "$f" ] || continue          # flat folders; ignore any subdirs
        cp -p -- "$f" "$dst/"
        n=$((n + 1))
        if [ "$do_exclude" = "exclude" ] && [ -d .git ]; then
            base="$(basename -- "$f")"
            pat="/$dst/$base"
            grep -qxF -- "$pat" "$exclude_file" 2>/dev/null \
                || echo "$pat" >> "$exclude_file"
        fi
    done
    echo "  [ok]   $label: synced $n file(s)  $src -> $dst"
}

echo "Syncing device-local models + missions into the tree:"
# Models: binaries are gitignored by extension; the small class-index *.yaml are
# tracked and identical, so no per-file git-exclude is needed here.
sync_dir "$MODELS_SRC"   "$MODELS_DST"   "models"
# Missions: personal rakib_* scripts are NOT gitignored -> exclude each per-clone.
sync_dir "$MISSIONS_SRC" "$MISSIONS_DST" "missions" exclude
echo ""

source /opt/ros/humble/setup.bash 2>/dev/null || true

# Wipe stale Python package dirs so colcon sees a clean slate.
# duburi_interfaces is ament_cmake so its install dir is managed correctly by CMake.
for pkg in duburi_control duburi_manager duburi_sensors duburi_planner duburi_vision; do
    rm -rf "install/$pkg" "build/$pkg"
done

# Drop now-nonexistent entries from the colon-lists colcon validates. When you
# `rm -rf` / re-`git clone` the workspace and rebuild in a shell that had a prior
# `source install/setup.bash`, AMENT_PREFIX_PATH / CMAKE_PREFIX_PATH still point
# at install/ dirs that no longer exist -> colcon prints a wall of
#   "The path '.../install/<pkg>' ... doesn't exist" WARNINGs
# (harmless, but noisy enough to hide a real error). Run this AFTER the wipe so
# the just-removed dirs are pruned too; the ROS underlay (/opt/ros/humble, which
# exists) and any live prefix survive. `source install/setup.bash` below re-adds
# each package cleanly as it gets built.
_prune_missing_paths() {  # echo $1 (a ':'-list) minus empty/nonexistent dirs
    local out='' e
    local IFS=':'
    for e in $1; do
        [ -n "$e" ] && [ -d "$e" ] || continue
        out="${out:+$out:}$e"
    done
    printf '%s' "$out"
}
export AMENT_PREFIX_PATH="$(_prune_missing_paths "${AMENT_PREFIX_PATH:-}")"
export CMAKE_PREFIX_PATH="$(_prune_missing_paths "${CMAKE_PREFIX_PATH:-}")"

# Step 1: build the interface package first so generated types are available.
# --cmake-args -Wno-dev silences the CMP0148 "warning for project developers"
# lines that ROS's own rosidl_generator_py cmake emits (not our code) — the
# exact lever the warning text names. Our CMakeLists.txt sets a >=3.10 minimum
# so the cmake_minimum_required deprecation is gone too.
colcon build --packages-select duburi_interfaces --cmake-args -Wno-dev "$@"
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
echo "  ros2 run duburi_planner mission demo_square"
echo "Vision (single camera + YOLO11 detector + tracker + viewer):"
echo "  ros2 launch duburi_vision vision.launch.py camera:=forward"
echo "Vision (dual camera, competition):"
echo "  ros2 launch duburi_vision vision_dual.launch.py"
