#!/usr/bin/env bash
# Reproducible setup for the Pi 5 + AI HAT+ (Hailo-8) vision box.
# Ubuntu 24.04 (Noble) + ROS 2 Jazzy.  Idempotent -- safe to re-run.
#
# Full rationale and every measurement: .claude/context/pi-hailo-vision-box.md
# NOT `set -u`: ROS's own setup.bash references unbound variables
# (AMENT_TRACE_SETUP_FILES), so nounset kills the script at step 5.
set -eo pipefail

say() { printf '\n\033[1m== %s\033[0m\n' "$*"; }

say "1/6  Python: undo the two traps that break cv_bridge"
# Noble ships numpy 1.26.4 and cv2 4.6.0, and ROS Jazzy's cv_bridge is compiled
# against numpy 1.x. A pip numpy>=2 or a pip opencv-python in ~/.local shadows
# them and cv_bridge dies with "_ARRAY_API not found". jetson-and-env-traps.md E1-E3.
python3 -m pip uninstall -y -q --break-system-packages numpy opencv-python 2>/dev/null || true
echo 'numpy<2' > ~/duburi_constraints.txt

say "2/6  Python: vision deps, pinned so pip cannot drag numpy 2 back in"
# --no-deps on BOTH: supervision and trackers each hard-require opencv-python,
# which would re-shadow the system cv2. supervision 0.30.x uses np.long
# (numpy-2 only), so 0.26.1 is the ceiling on numpy 1.26.4.
python3 -m pip install -q --break-system-packages -c ~/duburi_constraints.txt \
        scipy filterpy pymavlink pyserial
python3 -m pip install -q --break-system-packages --no-deps \
        'supervision==0.26.1' 'trackers==2.4.0'

say "3/6  ROS packages duburi_ws declares"
ROS_PKGS="ros-jazzy-yasmin ros-jazzy-yasmin-ros ros-jazzy-vision-msgs
          ros-jazzy-cv-bridge ros-jazzy-image-transport ros-jazzy-foxglove-bridge
          ros-jazzy-rosbag2-storage-mcap ros-jazzy-web-video-server
          ros-jazzy-rosbridge-server python3-colcon-common-extensions python3-rosdep"
MISSING=""
for p in $ROS_PKGS; do
  dpkg -s "$p" >/dev/null 2>&1 || MISSING="$MISSING $p"
done
# Only reach for sudo if something is actually absent, so a re-run needs no
# password and the script stays idempotent under automation.
if [ -n "$MISSING" ]; then
  echo "  installing:$MISSING"
  sudo apt-get install -y -q $MISSING
else
  echo "  all present, nothing to do"
fi

say "4/6  HailoRT Python bindings (NOT on PyPI -- built from source)"
if ! python3 -c 'import hailo_platform' 2>/dev/null; then
  # The wheel is behind a Hailo Developer Zone login. Build it against the
  # ALREADY-INSTALLED libhailort; the versions must match exactly.
  # Two traps: -DHAILO_BUILD_PYBIND=ON on the top-level cmake does NOTHING
  # (the bindings have their own setup.py), and -j4 OOMs a 4 GB Pi.
  V=$(hailortcli --version | grep -oE '[0-9]+\.[0-9]+\.[0-9]+' | head -1)
  [ -d ~/hailort_src ] || git clone --depth 1 -b "v$V" \
      https://github.com/hailo-ai/hailort.git ~/hailort_src
  cd ~/hailort_src && cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
      && cmake --build build -j2
  cd ~/hailort_src/hailort/libhailort/bindings/python/platform \
      && python3 -m pip install --break-system-packages --no-build-isolation .
fi

say "5/6  Verify -- every import together, no PYTHONPATH hacks"
source /opt/ros/jazzy/setup.bash
python3 - <<'PY'
import sys
mods = ['numpy','cv2','scipy','filterpy','supervision','trackers','pymavlink',
        'serial','yaml','hailo_platform','rclpy','cv_bridge','vision_msgs.msg']
bad = []
for m in mods:
    try: __import__(m, fromlist=['x'])
    except Exception as e: bad.append(f'{m}: {type(e).__name__}')
import numpy, cv2
print(f'  numpy {numpy.__version__}  cv2 {cv2.__version__} '
      f'({"system" if "dist-packages" in cv2.__file__ else "PIP -- SHADOWING, WRONG"})')
if numpy.__version__.startswith('2'):
    sys.exit('  FAIL: numpy 2 is back; cv_bridge will break')
if bad: sys.exit('  FAIL: ' + ', '.join(bad))
print(f'  {len(mods)}/{len(mods)} imports OK')
PY

say "6/6  Build duburi_ws"
# Interfaces first, then the rest. PLAIN colcon build -- mixing a plain build
# with --symlink-install makes CMake try to replace a real directory with a
# symlink and the build fails.
cd ~/duburi_ws
colcon build --packages-select duburi_interfaces
source install/setup.bash
colcon build

say "Done.  'source ~/pi_env.sh' in every terminal."
echo "  HEFs belong in ~/hailo_models, each WITH its .yaml sidecar --"
echo "  a .hef has no class names, and without the sidecar the detector"
echo "  returns [] on every frame with only a warning."
