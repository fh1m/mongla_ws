#!/usr/bin/env bash
# One ROS distro, from a clean ros:<distro>-ros-base container: the declared
# dependencies, a clean build, and every test (#65). .github/workflows/ros.yml
# runs it for humble and jazzy.
#
#   bash tools/ci_ros.sh            inside the container, from the repo root
#
# No step here may turn a failure into a pass: no `|| true`, no -k/-m/-x, no
# deselection. test/test_ros_ci_workflow.py holds this file to that.
set -euo pipefail

distro="${ROS_DISTRO:?run inside a ros:<distro> container}"
export DEBIAN_FRONTEND=noninteractive PIP_DISABLE_PIP_VERSION_CHECK=1 PIP_PROGRESS_BAR=off
summary="${GITHUB_STEP_SUMMARY:-/dev/null}"

echo "== ROS $distro at $(git rev-parse --short HEAD 2>/dev/null || echo unknown)"
apt-get update -qq
apt-get install -y -qq python3-venv python3-pip > /dev/null

echo "== the dependencies the package.xml files declare (rosdep)"
rosdep update --rosdistro "$distro" > /dev/null
rosdep install --from-paths src --ignore-src -y -q --rosdistro "$distro"

# A venv that still sees the system's ROS and apt Python packages. On Noble
# pip may not touch the system site (PEP 668), and numpy must stay the one
# cv_bridge was built against: kutum/constraints.txt caps it below 2.
echo "== the pip requirements, under kutum/constraints.txt"
python3 -m venv --system-site-packages /opt/ci-venv
# shellcheck source=/dev/null
source /opt/ci-venv/bin/activate
# torch only backs test_detector's tensor shims; the CPU wheel is a tenth of
# the default one and needs no GPU.
python3 -m pip install -q --index-url https://download.pytorch.org/whl/cpu torch torchvision
python3 -m pip install -q -r requirements.txt -c kutum/constraints.txt
python3 - <<'EOF'
import numpy, sys
print(f'python {sys.version.split()[0]}, numpy {numpy.__version__}')
assert int(numpy.__version__.split('.')[0]) < 2, 'numpy 2 would break cv_bridge'
EOF

echo "== clean colcon build"
set +u
# shellcheck source=/dev/null
source "/opt/ros/$distro/setup.bash"
set -u
rm -rf build install log
colcon build --symlink-install --event-handlers console_cohesion+
set +u
# shellcheck source=/dev/null
source install/setup.bash
set -u

echo "== every test"
report="$(mktemp)"
set +e
python3 -m pytest -q -p no:cacheprovider -rs src/*/test test 2>&1 | tee "$report"
rc=${PIPESTATUS[0]}
set -e

{
  echo "### ROS $distro"
  echo
  echo "Commit \`$(git rev-parse HEAD 2>/dev/null || echo unknown)\`, $(python3 --version)."
  echo
  echo '```'
  tail -1 "$report"
  echo '```'
  echo
  echo "Skipped (hardware or data not in CI; not counted as passing):"
  echo
  grep -E '^SKIPPED' "$report" | sed 's/^/- /' || echo "- none"
  echo
  echo "Hardware validation not performed here: the board, the Hailo-8 and the cameras run on Kutum."
} >> "$summary"
exit "$rc"
