#!/usr/bin/env bash
# Turn a bare Raspberry Pi 5 + Hailo-8 into a Mongla vehicle, repeatably.
#
# ⛔ WHY THIS EXISTS. On 2026-09-22 the vehicle was found running a workspace
# 122 commits behind and on the far side of the project rename, so its package
# names no longer matched the ones the repo ships. The current stack had NEVER
# been built on the vehicle. A deploy path that has never been exercised is the
# thing that turns into "nothing works" on the day.
#
# Every step below is one that was actually performed and observed, not a
# plausible sequence. The measured result on that machine:
#
#     rsync      144 MB after the excludes below (3.8 GB before them)
#     colcon     7 packages finished [42.1s], zero failures
#     gate       bringup_check --srot ran and correctly FAILED on the absent Bar30
#
# Usage:
#     scripts/provision_vehicle.sh fh1m@192.168.0.103
#     scripts/provision_vehicle.sh fh1m@mongla.local --check-only
#
# It is IDEMPOTENT: run it again after any change and it re-syncs and rebuilds.
set -euo pipefail

TARGET="${1:-}"
MODE="${2:-full}"
REMOTE_DIR="mongla_ws"
ROS_DISTRO_WANT="jazzy"

if [[ -z "$TARGET" ]]; then
  echo "usage: $0 user@host [--check-only]" >&2
  exit 2
fi

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

say()  { printf '\n\033[1m== %s\033[0m\n' "$*"; }
ok()   { printf '  \033[32m[ok]\033[0m   %s\n' "$*"; }
warn() { printf '  \033[33m[warn]\033[0m %s\n' "$*"; }
die()  { printf '  \033[31m[STOP]\033[0m %s\n' "$*" >&2; exit 1; }

r() { ssh -o BatchMode=yes -o ConnectTimeout=10 "$TARGET" "$@"; }

# --------------------------------------------------------------------------- #
say "1. Can we reach it, and is it the right kind of machine?"

r true 2>/dev/null || die "cannot ssh to $TARGET with BatchMode (key auth). \
Copy a key first: ssh-copy-id $TARGET"
ok "ssh to $TARGET"

# ROS. `bash -lc` because a non-interactive ssh gets no profile, and colcon and
# rclpy both live behind it -- the first attempt at this failed silently for
# exactly that reason.
DISTRO="$(r bash -lc "'source /opt/ros/$ROS_DISTRO_WANT/setup.bash 2>/dev/null && echo \$ROS_DISTRO'" || true)"
[[ "$DISTRO" == "$ROS_DISTRO_WANT" ]] \
  || die "ROS 2 $ROS_DISTRO_WANT not found (got '${DISTRO:-nothing}'). \
The vehicle runs Jazzy; dev boxes run Humble, and the two do not interoperate."
ok "ROS 2 $DISTRO"

r bash -lc "'source /opt/ros/$ROS_DISTRO_WANT/setup.bash && python3 -c \"import rclpy\"'" >/dev/null 2>&1 \
  || die "rclpy does not import under the sourced environment"
ok "rclpy imports"

r "command -v colcon" >/dev/null 2>&1 || die "colcon is not installed"
ok "colcon present"

# The accelerator. Without it the detector has no flight path at all, so this is
# a refusal rather than a warning.
if r "ls /dev/hailo* >/dev/null 2>&1 || command -v hailortcli >/dev/null 2>&1"; then
  ok "Hailo accelerator present"
else
  die "no Hailo device and no hailortcli. This provisions a Pi + Hailo-8 vehicle."
fi

# Serial access. The board is a CH340; without dialout the manager cannot open
# it, and the failure looks like a dead board rather than a permissions problem.
if r "id -nG | tr ' ' '\n' | grep -qx dialout"; then
  ok "user is in dialout (can open the board)"
else
  warn "user is NOT in dialout -- the board will not open. Fix with:
           sudo usermod -aG dialout \$USER   (then log out and back in)"
fi

FREE_GB="$(r "df -BG --output=avail \$HOME | tail -1 | tr -dc '0-9'")"
[[ "${FREE_GB:-0}" -ge 5 ]] || die "only ${FREE_GB}G free in \$HOME; need >= 5G"
ok "${FREE_GB}G free"

if [[ "$MODE" == "--check-only" ]]; then
  say "check-only: stopping before any change"
  exit 0
fi

# --------------------------------------------------------------------------- #
say "2. Sync the tree"

# ⚠ THE EXCLUDES ARE LOAD-BEARING. The first real deploy shipped 3.8 GB of
# `logs/` and two .tlog captures to the vehicle before anyone looked. Anything
# that is generated, huge, or belongs to the dev box stays here.
rsync -az --delete --info=stats1 \
  --exclude 'build/' --exclude 'install/' --exclude 'log/' --exclude 'logs/' \
  --exclude '.git/' --exclude 'graphify-out/' --exclude 'sim/' \
  --exclude 'docs/' --exclude '__pycache__/' --exclude '*.pyc' \
  --exclude '*.tlog' --exclude '*.tlog.raw' --exclude '*.tar.gz' \
  --exclude '.pytest_cache/' \
  ./ "$TARGET:~/$REMOTE_DIR/"
ok "synced to $TARGET:~/$REMOTE_DIR"

# --------------------------------------------------------------------------- #
say "3. Build"

r bash -lc "'set -e
  cd ~/$REMOTE_DIR
  source /opt/ros/$ROS_DISTRO_WANT/setup.bash
  colcon build --symlink-install > build.log 2>&1
  tail -1 build.log'" || {
    echo
    warn "build failed; last 25 lines:"
    r "tail -25 ~/$REMOTE_DIR/build.log" || true
    die "colcon build failed"
  }
ok "colcon build finished"

# --------------------------------------------------------------------------- #
say "4. Gate it -- the vehicle grades itself"

# ⛔ bringup_check EXITS NON-ZERO ON A FAULT, and that is the point. We do not
# swallow it: a provisioning script that reports success on a vehicle its own
# gate refuses would be the exact defect this repo keeps finding.
set +e
r bash -lc "'cd ~/$REMOTE_DIR
  source /opt/ros/$ROS_DISTRO_WANT/setup.bash
  source install/setup.bash
  timeout 120 ros2 run mongla_manager bringup_check --srot'"
GATE=$?
set -e

echo
if [[ $GATE -eq 0 ]]; then
  say "PROVISIONED, and the gate passes."
else
  say "PROVISIONED, but THE GATE REFUSES (exit $GATE)."
  cat <<'NOTE'
  The build is good; the vehicle is not flight-ready. Read the FAIL lines above.

  Two that are expected on a bare bench, and are NOT provisioning faults:
    * Bar30 unhealthy        no depth sensor connected -> DEPTH_HOLD/AUTO, and
                             therefore every move verb, are refused by design.
    * NO USB cameras         cameras unplugged.

  One that IS a provisioning gap, and has no fix inside this repo:
    * "Hailo present but NO .hef"
      ⛔ THE COMPILED MODELS ARE NOT IN THE TREE. They are gitignored, so a
      fresh vehicle provisioned from this repo has nothing to fly. Copy them in
      beside their <stem>.yaml sidecars:

          rsync -a ~/models/*.hef ~/models/*.yaml \
                TARGET:~/mongla_ws/src/mongla_vision/models/

      A .hef WITHOUT its .yaml sidecar is worse than no model: the class
      allowlist comes back empty and the detector returns [] every frame while
      the whole pipeline looks healthy. The gate fails on that deliberately.
NOTE
fi
exit $GATE
