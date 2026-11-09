#!/usr/bin/env bash
# Releases on Kutum: one immutable directory per deploy, and an atomic switch.
#
#   ~/mongla/releases/<id>/   a full tree, built in place, never edited again
#   ~/mongla_ws               a symlink to the active release (env.sh, docs and
#                             habits all keep using this path)
#   ~/mongla/previous         the release that was active before this one
#
# Run on the vehicle. kutum/deploy.sh drives it over ssh; by hand:
#
#   release.sh preflight            refuse if the manager, a launch or a mission
#                                   is running (exit 1); touches nothing
#   release.sh build ID             build ~/mongla/releases/ID in place
#   release.sh activate ID DESC     preflight again, then switch to ID and
#                                   write its manifest
#   release.sh rollback             switch back to ~/mongla/previous
#   release.sh list                 every release, the active one marked
#   release.sh prune [KEEP]         remove all but the newest KEEP built releases
#                                   (default 5), never the active or previous one
#
# Why (#64). The old deploy synced over the live tree, then deleted build/ and
# install/, and only afterwards checked whether a manager was running. A
# deploy during a run rewrote the code under it; a failed build left no
# working install at all. Here a release is staged and built beside the live
# one, nothing switches until the build succeeds, and the switch is a rename.
set -euo pipefail

HOME_DIR="${HOME:?}"
BASE="$HOME_DIR/mongla"
RELEASES="$BASE/releases"
LIVE="$HOME_DIR/mongla_ws"
PREVIOUS="$BASE/previous"
ROS_SETUP="${KUTUM_ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
# The manifest runs hailortcli; keep its log out of the release and out of ~.
export HAILORT_LOGGER_PATH="${HAILORT_LOGGER_PATH:-$HOME_DIR/.hailo/hailort}"

ok()  { printf '  ok   %s\n' "$*"; }
die() { printf '  FAIL %s\n' "$*" >&2; exit 1; }

valid_id() { [[ "$1" =~ ^[A-Za-z0-9][A-Za-z0-9._-]{0,80}$ ]] || die "bad release id: $1"; }

# Everything that runs the stack. A launch, a mission or a manager started any
# way at all: the deployed code may be under each of them.
preflight() {
  local busy mine pid
  # Not this script nor the shells that started it: over ssh the remote shell's
  # own command line names the workspace, and a bare pgrep matched itself on
  # the vehicle (2026-11-09).
  mine=" $$ "; pid=$$
  while [[ "$pid" -gt 1 ]]; do
    pid="$(ps -o ppid= -p "$pid" 2>/dev/null | tr -d ' ')"; [[ -n "$pid" ]] || break
    mine+="$pid "
  done
  busy="$(pgrep -af 'mongla_manager|mongla_planner|mongla_vision|ros2 launch mongla' \
          | while read -r p rest; do [[ "$mine" == *" $p "* ]] || printf '%s %s\n' "$p" "$rest"; done || true)"
  if [[ -n "$busy" ]]; then
    printf '%s\n' "$busy" | sed 's/^/       /' >&2
    die "the stack is RUNNING on the vehicle -- stop it (Ctrl-C the manager, so it disarms) and rerun"
  fi
  ok "nothing of the stack is running"
}

active() { [[ -L "$LIVE" ]] && basename "$(readlink -f "$LIVE")" || echo ""; }

build() {
  local id="$1" dir="$RELEASES/$1"
  valid_id "$id"
  [[ -d "$dir/src" ]] || die "$dir has no src/ -- the sync did not finish"
  [[ -e "$dir/.build-ok" ]] && { ok "$id is already built"; return 0; }
  ( cd "$dir"
    set +u
    # shellcheck source=/dev/null
    source "$ROS_SETUP"
    set -u
    rm -rf build install log
    colcon build --symlink-install > build.log 2>&1 ) || {
      tail -25 "$dir/build.log" >&2 || true
      die "colcon build failed in $id; the active release is unchanged"
    }
  date -u +%FT%TZ > "$dir/.build-ok"
  ok "built $id: $(tail -1 "$dir/build.log")"
}

# Point $LIVE at a release with a rename, which is atomic: a shell that sources
# env.sh sees the old tree or the new one, never half of each.
switch_to() {
  local dir="$1"
  ln -sfn "$dir" "$LIVE.next"
  mv -T "$LIVE.next" "$LIVE"
}

# The first deploy under this scheme finds ~/mongla_ws as a plain directory.
# Keep it whole, as a release that is never pruned and never rolled back to
# (its install/ was built at the old path), and replace it with the symlink.
migrate() {
  if [[ -e "$LIVE" && ! -L "$LIVE" ]]; then
    local legacy
    legacy="$RELEASES/legacy-$(date -u +%Y%m%dT%H%M%S)"
    mv "$LIVE" "$legacy"
    ok "kept the old workspace as $(basename "$legacy") (never pruned)"
  fi
}

activate() {
  local id="$1" desc="$2" dir="$RELEASES/$1" cur
  valid_id "$id"
  preflight
  [[ -e "$dir/.build-ok" ]] || die "$id was never built successfully; refusing to activate it"
  mkdir -p "$BASE"
  migrate
  cur="$(active)"
  if [[ -n "$cur" && "$cur" != "$id" ]]; then
    printf '%s\n' "$cur" > "$PREVIOUS.next" && mv -T "$PREVIOUS.next" "$PREVIOUS"
  fi
  switch_to "$dir"
  ok "active release: $id (previous: ${cur:-none})"
  python3 "$dir/kutum/release_manifest.py" write "$dir" "$desc"
}

rollback() {
  preflight
  [[ -f "$PREVIOUS" ]] || die "no previous release recorded"
  local prev cur
  prev="$(cat "$PREVIOUS")"; valid_id "$prev"
  [[ "$prev" != legacy-* ]] || die "$prev predates releases; its install/ points at the old path"
  [[ -e "$RELEASES/$prev/.build-ok" ]] || die "previous release $prev is not a built release"
  cur="$(active)"
  switch_to "$RELEASES/$prev"
  [[ -n "$cur" ]] && { printf '%s\n' "$cur" > "$PREVIOUS.next"; mv -T "$PREVIOUS.next" "$PREVIOUS"; }
  ok "rolled back: $prev is active (previous: ${cur:-none})"
}

list() {
  local cur prev d id mark
  cur="$(active)"; prev="$(cat "$PREVIOUS" 2>/dev/null || true)"
  [[ -d "$RELEASES" ]] || { echo "  no releases"; return 0; }
  for d in "$RELEASES"/*/; do
    id="$(basename "$d")"; mark="  "
    [[ "$id" == "$cur" ]] && mark="* "
    [[ "$id" == "$prev" ]] && mark="p "
    if [[ -e "$RELEASES/$id/.build-ok" ]]; then state=built
    elif [[ "$id" == legacy-* ]]; then state=legacy
    else state=incomplete; fi
    printf '  %s%-48s %s %s\n' "$mark" "$id" "$state" \
      "$(cat "$RELEASES/$id/.kutum-deployed" 2>/dev/null | cut -d' ' -f1-2 || true)"
  done
  echo "  (* active, p previous)"
}

# Only what this script made: built releases beyond KEEP, and staging that
# never built. Never the active or previous release, never a legacy tree, and
# nothing outside ~/mongla/releases (runs, data, models and calibration live
# elsewhere, see kutum/backup.sh).
prune() {
  local keep="${1:-5}" cur prev n=0 d
  cur="$(active)"; prev="$(cat "$PREVIOUS" 2>/dev/null || true)"
  [[ -d "$RELEASES" ]] || return 0
  # Newest first, by modification time; ids never contain newlines (valid_id).
  local by_age
  by_age="$(find "$RELEASES" -mindepth 1 -maxdepth 1 -type d -printf '%T@ %f\n' | sort -rn | cut -d' ' -f2)"
  for d in $by_age; do
    [[ "$d" == "$cur" || "$d" == "$prev" || "$d" == legacy-* ]] && continue
    if [[ -e "$RELEASES/$d/.build-ok" ]]; then
      n=$((n + 1)); (( n <= keep )) && continue
    fi
    rm -rf -- "${RELEASES:?}/$d"
    ok "pruned $d"
  done
}

cmd="${1:-}"; shift || true
case "$cmd" in
  preflight) preflight ;;
  build)     build "${1:?release id}" ;;
  activate)  activate "${1:?release id}" "${2:?commit description}" ;;
  rollback)  rollback ;;
  list)      list ;;
  prune)     prune "${1:-5}" ;;
  *) sed -n '2,20p' "$0" 2>/dev/null || true; exit 2 ;;
esac
