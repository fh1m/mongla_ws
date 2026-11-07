#!/usr/bin/env bash
# Put this tree on Kutum (the vehicle's Pi 5 + Hailo-8), build it, and gate it.
#
# The machine itself is made once by kutum/setup.sh; this is the part you run
# after every change.
#
# Why this exists. On 2026-09-22 the vehicle was found running a workspace
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
#     gate       bringup_check ran and correctly FAILED on the absent Bar30
#
# Usage:
#     kutum/deploy.sh fh1m@mongla.local
#     kutum/deploy.sh fh1m@mongla.local --check-only     # the doctor, nothing changed
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
say "1. Can we reach it?"

r true 2>/dev/null || die "cannot ssh to $TARGET with BatchMode (key auth). \
Copy a key first: ssh-copy-id $TARGET"
ok "ssh to $TARGET"
r test -f /opt/ros/$ROS_DISTRO_WANT/setup.bash \
  || die "no ROS 2 $ROS_DISTRO_WANT on $TARGET -- install it, then kutum/setup.sh"
ok "ROS 2 $ROS_DISTRO_WANT"

if [[ "$MODE" == "--check-only" ]]; then
  say "check-only: the doctor, nothing changed"
  r bash -s < "$REPO_ROOT/kutum/doctor.sh"
  exit $?
fi

# --------------------------------------------------------------------------- #
say "2. Sync the tree"

# The excludes are load-bearing. The first real deploy shipped 3.8 GB of
# `logs/` and two .tlog captures to the vehicle before anyone looked. Anything
# that is generated, huge, or belongs to the dev box stays here: the vehicle
# carries the book (.claude/context) for reading at the pool, not agent tooling,
# CI or editor settings.
rsync -az --delete --info=stats1 \
  --exclude 'build/' --exclude 'install/' --exclude 'log/' --exclude 'logs/' \
  --exclude '.git/' \
  --exclude 'docs/' --exclude '__pycache__/' --exclude '*.pyc' \
  --exclude '*.tlog' --exclude '*.tlog.raw' --exclude '*.tar.gz' \
  --exclude '.pytest_cache/' --exclude '.kutum-deployed' \
  --exclude '.claude/skills/' --exclude '.claude/agents/' --exclude '.claude/hooks/' \
  --exclude '.claude/settings*.json' --exclude '.understand-anything/' \
  --exclude '.github/' --exclude '.vscode/' --exclude '.beads/' --exclude 'wiki/' \
  ./ "$TARGET:~/$REMOTE_DIR/"
ok "synced to $TARGET:~/$REMOTE_DIR"

# --------------------------------------------------------------------------- #
say "3. Is it Kutum? (kutum/doctor.sh, from the copy just synced)"

# After the sync, not before: a FAIL that only `setup.sh --system` can fix
# needs kutum/ on the machine first. Run from the synced copy, the doctor also
# compares the installed system files with the repo byte for byte.
r "bash ~/$REMOTE_DIR/kutum/doctor.sh" || die "the doctor refuses this machine (FAIL lines above). \
Most are fixed by, on the vehicle: sudo ~/$REMOTE_DIR/kutum/setup.sh --system -- then rerun"

# --------------------------------------------------------------------------- #
say "3. Build"

# Not under a running manager (2026-10-03). The build wipes install/, so a
# manager left running keeps executing code that is no longer on disk, and it
# holds the board's port -- the gate below then cannot read the board.
if r "pgrep -f '[l]ib/mongla_manager/start' >/dev/null"; then  # [l]: not match this ssh command line itself
  die "a mongla_manager is RUNNING on the vehicle -- stop it (Ctrl-C, so it disarms) and rerun"
fi

r bash -lc "'set -e
  cd ~/$REMOTE_DIR
  source /opt/ros/$ROS_DISTRO_WANT/setup.bash
  # Always a clean build. An incremental symlink-install tree keeps what
  # source no longer has: links to a deleted model broke the build on
  # 2026-10-03 and the gate then counted the retired model as installed.
  # 42 s buys a vehicle that runs exactly this tree and nothing older.
  rm -rf build install log .pytest_cache
  find src -name __pycache__ -type d -prune -exec rm -rf {} +
  rm -rf ~/.ros/log
  colcon build --symlink-install > build.log 2>&1
  tail -1 build.log'" || {
    echo
    warn "build failed; last 25 lines:"
    r "tail -25 ~/$REMOTE_DIR/build.log" || true
    die "colcon build failed"
  }
ok "colcon build finished"

# What is on the vehicle, in one line the doctor prints: the commit, whether
# the tree had uncommitted changes, and when.
sha="$(git -C "$REPO_ROOT" rev-parse --short HEAD 2>/dev/null || echo unknown)"
dirty="$(git -C "$REPO_ROOT" status --porcelain 2>/dev/null | grep -q . && echo ' +uncommitted' || true)"
r "echo '$sha$dirty $(date -u +%FT%TZ) from $(hostname)' > ~/$REMOTE_DIR/.kutum-deployed"

# --------------------------------------------------------------------------- #
say "4. Gate it -- the vehicle grades itself"

# bringup_check exits non-zero on a fault, and that is the point. We do not
# swallow it: a provisioning script that reports success on a vehicle its own
# gate refuses would be the exact defect this repo keeps finding.
set +e
r bash -lc "'cd ~/$REMOTE_DIR
  source /opt/ros/$ROS_DISTRO_WANT/setup.bash
  source install/setup.bash
  timeout 120 ros2 run mongla_manager bringup_check'"
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
      THE COMPILED MODELS ARE NOT IN THE TREE. They are gitignored, so a
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
