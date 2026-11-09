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
#     kutum/deploy.sh fh1m@mongla.local --list           # the releases on the vehicle
#     kutum/deploy.sh fh1m@mongla.local --rollback       # back to the previous release
#
# Each deploy is a new release beside the live one (kutum/release.sh, #64):
# nothing on the vehicle is touched until the stack is confirmed stopped and
# the board not armed, nothing switches until the new tree has built, and the
# switch is one rename. A failed or interrupted deploy leaves the active
# release exactly as it was.
set -euo pipefail

TARGET="${1:-}"
MODE="${2:-full}"
REMOTE_DIR="mongla_ws"            # a symlink to the active release
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
if [[ "$MODE" == "--list" ]]; then
  r bash -s -- list < "$REPO_ROOT/kutum/release.sh"
  exit $?
fi

# --------------------------------------------------------------------------- #
say "2. Is it safe to touch? (before anything on the vehicle changes)"

# Both checks are piped from this tree, so they run before a single file is
# synced. "Could not tell" stops the deploy the same as "armed".
r bash -s -- preflight < "$REPO_ROOT/kutum/release.sh" || exit 1
r python3 - < "$REPO_ROOT/kutum/armed_probe.py" || die "the board is armed, or its state cannot be read -- disarm it, or unplug it, and rerun"

if [[ "$MODE" == "--rollback" ]]; then
  r bash -s -- rollback < "$REPO_ROOT/kutum/release.sh"
  exit $?
fi

# --------------------------------------------------------------------------- #
say "3. Stage a new release beside the live one"

sha="$(git -C "$REPO_ROOT" rev-parse --short HEAD 2>/dev/null || echo unknown)"
# Dirty means the vehicle gets something no commit has. Untracked tool state
# that the excludes below never send (.codegraph/) does not count.
dirty="$(git -C "$REPO_ROOT" status --porcelain 2>/dev/null | grep -v '^?? \.codegraph/' \
         | grep -q . && echo '-dirty' || true)"
ID="${sha}${dirty}-$(date -u +%Y%m%dT%H%M%S)"
DESC="$sha${dirty:+ +uncommitted} from $(hostname)"
REL="mongla/releases/$ID"

# Unchanged files are hard links to the active release, so a release costs
# what changed, not 150 MB. rsync replaces a file by renaming a new copy into
# place, so a link is never written through into the release it came from.
LIVE_REAL="$(r 'readlink -f ~/mongla_ws 2>/dev/null || true')"
LINK=()
[[ -n "$LIVE_REAL" ]] && r test -d "$LIVE_REAL/src" && LINK=(--link-dest="$LIVE_REAL/")
r mkdir -p "$REL"

# The excludes are load-bearing. The first real deploy shipped 3.8 GB of
# `logs/` and two .tlog captures to the vehicle before anyone looked. Anything
# that is generated, huge, or belongs to the dev box stays here: the vehicle
# carries the book (.claude/context) for reading at the pool, not agent tooling,
# CI or editor settings.
rsync -az --delete --info=stats1 "${LINK[@]}" \
  --exclude 'build/' --exclude 'install/' --exclude 'log/' --exclude 'logs/' \
  --exclude '.git/' \
  --exclude 'docs/' --exclude '__pycache__/' --exclude '*.pyc' \
  --exclude '*.tlog' --exclude '*.tlog.raw' --exclude '*.tar.gz' \
  --exclude '.pytest_cache/' --exclude '.kutum-deployed' --exclude 'release.json' \
  --exclude '.build-ok' \
  --exclude '.claude/skills/' --exclude '.claude/agents/' --exclude '.claude/hooks/' \
  --exclude '.claude/settings*.json' --exclude '.understand-anything/' \
  --exclude '.github/' --exclude '.vscode/' --exclude '.beads/' --exclude 'wiki/' \
  --exclude '.codegraph/' \
  ./ "$TARGET:~/$REL/"
ok "staged $ID"

# --------------------------------------------------------------------------- #
say "4. Is it Kutum? (kutum/doctor.sh, from the release just staged)"

# After the sync, not before: a FAIL that only `setup.sh --system` can fix
# needs kutum/ on the machine first. Run from the staged copy, the doctor also
# compares the installed system files with the repo byte for byte.
r "bash ~/$REL/kutum/doctor.sh" || die "the doctor refuses this machine (FAIL lines above). \
Most are fixed by, on the vehicle: sudo ~/$REL/kutum/setup.sh --system -- then rerun"

# --------------------------------------------------------------------------- #
say "5. Build the release, then switch to it"

# Built in its own directory, so the live release keeps running whatever
# happens here; always from clean, since the directory is new.
r "bash ~/$REL/kutum/release.sh build $ID" || die "build failed; the active release is unchanged"
r "bash ~/$REL/kutum/release.sh activate $ID '$DESC'" || die "not activated; the active release is unchanged"
r "bash ~/$REL/kutum/release.sh prune 5" || warn "prune failed (nothing active was touched)"

# --------------------------------------------------------------------------- #
say "6. Gate it -- the vehicle grades itself"

# bringup_check exits non-zero on a fault, and that is the point. We do not
# swallow it: a provisioning script that reports success on a vehicle its own
# gate refuses would be the exact defect this repo keeps finding.
set +e
GATE_LOG="$(mktemp)"
# Through env.sh, so the gate runs in the same environment as every terminal
# on the vehicle (and HailoRT logs to ~/.hailo, not into ~).
r bash -lc "'cd ~/$REMOTE_DIR
  source kutum/env.sh
  timeout 120 ros2 run mongla_manager bringup_check'" | tee "$GATE_LOG"
GATE=${PIPESTATUS[0]}
set -e
# Recorded beside the release: "deployed" and "the vehicle's gate passed"
# are different facts, and a release must say which it has.
r "python3 ~/$REL/kutum/release_manifest.py gate ~/$REL $GATE" || warn "could not record the gate verdict"

# Explain only the FAIL lines this run printed. A fixed list explained faults
# the vehicle did not have and missed the one it did.
saw() { grep -q -- "$1" "$GATE_LOG"; }

echo
if [[ $GATE -eq 0 ]]; then
  say "PROVISIONED, and the gate passes."
else
  say "PROVISIONED, but THE GATE REFUSES (exit $GATE)."
  echo "  The build is good; the vehicle is not flight-ready. Read the FAIL lines above."
  echo
  saw 'no vehicle HEARTBEAT' && cat <<'NOTE'
  * no vehicle HEARTBEAT    the board is not connected or not powered. Expected
                            on a bare bench; plug in the Type-C cable and re-run
                            `ros2 run mongla_manager bringup_check`.
NOTE
  saw 'Bar30 unhealthy' && cat <<'NOTE'
  * Bar30 unhealthy         no working depth sensor, so DEPTH_HOLD/AUTO and
                            therefore every move verb are refused by design.
NOTE
  saw 'NO .hef' && cat <<'NOTE'
  * Hailo present but NO .hef   a provisioning gap with no fix inside this repo.
      The compiled models are gitignored and travel with this tree, so put
      them in THIS dev box's src/mongla_vision/models/, beside their
      <stem>.yaml sidecars, and deploy again. (A release on the vehicle is
      never edited in place; a model copied into it would vanish with it.)

      A .hef without its .yaml sidecar is worse than no model: the class
      allowlist comes back empty and the detector returns [] every frame while
      the pipeline looks healthy. The gate fails on that deliberately.
NOTE
fi
rm -f "$GATE_LOG"
exit $GATE
