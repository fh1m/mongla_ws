#!/usr/bin/env bash
# One ROS distro, from a clean ros:<distro>-ros-base container: the declared
# dependencies, a clean build, and every test (#65). .github/workflows/ros.yml
# runs it for humble and jazzy.
#
#   bash tools/ci/ci_ros.sh            inside the container, from the repo root
#
# No step here may turn a failure into a pass: no `|| true`, no -k/-m/-x, no
# deselection. test/test_ros_ci_workflow.py holds this file to that.
set -euo pipefail

distro="${ROS_DISTRO:?run inside a ros:<distro> container}"
export DEBIAN_FRONTEND=noninteractive PIP_DISABLE_PIP_VERSION_CHECK=1 PIP_PROGRESS_BAR=off
summary="${GITHUB_STEP_SUMMARY:-/dev/null}"

# The checkout belongs to the runner's user and this container runs as root;
# without this, git refuses the tree and every git-reading test skips.
git config --global --add safe.directory "$PWD"
echo "== ROS $distro at $(git rev-parse --short HEAD 2>/dev/null || echo unknown)"
# The Ubuntu mirror failed required checks twice on 2026-11-10: a dropped
# connection mid-download (iproute2), and an index that changed size during a
# mirror sync. apt retries a download itself; the index refresh is retried
# here, 30 s apart. Either still failing after that fails the job.
echo 'Acquire::Retries "5";' > /etc/apt/apt.conf.d/80-retries
# An index that could not be fetched is an error, not a warning. By default
# `apt-get update` exits 0 when a mirror cannot be reached as long as one
# other source answered, so the retry below never ran: on 2026-11-11 the
# runner could not resolve the mirror, the refresh "passed", and the job died
# at the install with "no installation candidate".
echo 'APT::Update::Error-Mode "any";' >> /etc/apt/apt.conf.d/80-retries
# The archive mirror GitHub's own runners use, beside them in Azure: on
# 2026-11-10 archive.ubuntu.com took 33 of a 45 minute budget to serve the
# dependencies. Same archive, same packages; only the host changes.
for f in /etc/apt/sources.list /etc/apt/sources.list.d/ubuntu.sources; do
  if [[ -f "$f" ]]; then
    sed -i 's|http://archive.ubuntu.com/ubuntu|http://azure.archive.ubuntu.com/ubuntu|g' "$f"
  fi
done
for attempt in 1 2 3 4; do
  if apt-get update -qq; then break; fi
  if [[ "$attempt" == 4 ]]; then echo "ERROR: apt-get update failed 4 times"; exit 100; fi
  echo "WARNING: apt-get update failed (attempt $attempt); the mirror may be syncing"
  sleep 30
done
# The distro's own pytest: ROS's launch_testing plugin is built against it,
# and pip's newest (9) no longer accepts the hook argument it declares.
# procps (pgrep) and iproute2 (ss): the manager's process-tree teardown and
# port checks shell out to them, and ros-base ships neither.
apt-get install -y -qq python3-venv python3-pip python3-pytest procps iproute2 > /dev/null

echo "== the dependencies the package.xml files declare (rosdep)"
rosdep update --rosdistro "$distro" > /dev/null
# A package's 404 means the index is older than the pool: the security archive
# published libgphoto2 mid-run on 2026-11-10 and failed three PRs at once. The
# index is refreshed and the install tried again; a real missing dependency
# still fails, after the third try.
for attempt in 1 2 3; do
  if rosdep install --from-paths src --ignore-src -y -q --rosdistro "$distro"; then break; fi
  if [[ "$attempt" == 3 ]]; then echo "ERROR: rosdep install failed 3 times"; exit 100; fi
  echo "WARNING: rosdep install failed (attempt $attempt); refreshing the package index"
  sleep 30
  if ! apt-get update -qq; then echo "WARNING: the index refresh failed too"; fi
done

# Built with the SYSTEM Python, as on the vehicle, before any pip package
# exists: torch drags a new setuptools into the venv, and that broke the
# interface package's egg build against the system's packaging (Humble).
echo "== clean colcon build"
set +u
# shellcheck source=/dev/null
source "/opt/ros/$distro/setup.bash"
set -u
rm -rf build install log
colcon build --symlink-install --event-handlers console_cohesion+

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
# trackers 2.4.0 declares numpy>=2.0.2, which is spurious and unsatisfiable
# beside numpy<2, so `pip install -r requirements.txt` cannot resolve on a
# clean machine (first seen by this job). The vehicle installs it --no-deps
# (kutum/setup.sh, pi-and-env-traps.md E3); so does this, from the same range.
req="$(mktemp)"
grep -v -E '^(trackers|pytest)' requirements.txt > "$req"
python3 -m pip install -q -r "$req" -c kutum/constraints.txt
python3 -m pip install -q --no-deps "$(grep -E '^trackers' requirements.txt | cut -d'#' -f1 | tr -d ' ')"
# The nodes were built with the system Python, so `ros2 launch` starts them
# under /usr/bin/python3, which cannot see this venv: a launched manager died
# on `import pymavlink` while every in-process test passed. The vehicle
# installs with `pip --user`, which the system Python does see; this puts the
# venv on the same footing.
PYTHONPATH="$(python3 -c 'import sysconfig; print(sysconfig.get_paths()["purelib"])')${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONPATH
python3 - <<'EOF'
import numpy, sys
print(f'python {sys.version.split()[0]}, numpy {numpy.__version__}')
assert int(numpy.__version__.split('.')[0]) < 2, 'numpy 2 would break cv_bridge'
EOF


set +u
# shellcheck source=/dev/null
source install/setup.bash
set -u

echo "== every test"
# No autoloaded plugins, as in the integrity gate: Jazzy's apt pytest loads
# ROS's launch_testing plugins, and the first run collected one module of
# 4759 tests. Nothing here uses a third-party plugin.
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
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
  echo
  # The bill of materials this result was measured against (#95): every pip
  # package at its exact version, and the installed ROS and Python apt
  # packages (dpkg also lists known, uninstalled ones with no version). The
  # vehicle's own is release.json, written per deploy on Kutum.
  pip_bom="$(python3 -m pip freeze --all 2>/dev/null | sort)"
  apt_bom="$(dpkg-query -W -f '${Package}=${Version}\n' "ros-$distro-*" 'python3-*' 2>/dev/null | grep -v '=$' | sort)"
  echo "<details><summary>Bill of materials: $(grep -c . <<<"$pip_bom") pip, $(grep -c . <<<"$apt_bom") apt</summary>"
  echo
  echo '```'
  echo "$pip_bom"
  echo
  echo "$apt_bom"
  echo '```'
  echo
  echo '</details>'
} >> "$summary"
exit "$rc"
