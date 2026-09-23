#!/usr/bin/env bash
# Compile the board's control code for the host, as a shared library.
#
# ⛔ IT COMPILES THE FIRMWARE CHECKOUT IN PLACE and copies nothing. If the
# firmware moves, this fails loudly rather than building a stale copy that would
# silently disagree with the board. A bench built from a copy is a bench that
# drifts, and a drifting bench is worse than none.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FW="${SROT_FIRMWARE:-$HERE/../../../../Mongla_others/srot-control-board}"

if [[ ! -f "$FW/src/control/mixer.cpp" ]]; then
  echo "no firmware checkout at: $FW" >&2
  echo "set SROT_FIRMWARE to the srot-control-board directory" >&2
  exit 2
fi
FW="$(cd "$FW" && pwd)"
echo "firmware: $FW"

# Record WHICH firmware this was built from. A bench that cannot say which
# commit it represents cannot be trusted after the firmware moves.
REV="$(git -C "$FW" rev-parse --short HEAD 2>/dev/null || echo unknown)"
DIRTY=""
git -C "$FW" diff --quiet 2>/dev/null || DIRTY=" (dirty)"
echo "revision: ${REV}${DIRTY}"
printf '%s\n' "${REV}${DIRTY}" > "$HERE/.firmware_rev"

SRC=(
  "$HERE/bench_api.cpp"
  "$FW/src/control/mixer.cpp"
  "$FW/src/control/thrust_trim.cpp"
  "$FW/src/control/attitude_control.cpp"
  "$FW/src/control/feedforward.cpp"
  "$FW/src/control/depth_control.cpp"
)

# -ffp-contract=off matters: the board is a 32-bit ESP32 without FMA, so letting
# the host fuse multiply-adds would change the last bits of every thrust-curve
# evaluation. The bench is judged against the board to under one DShot count of
# 999; that tolerance does not leave room for a different rounding model.
g++ -O2 -fPIC -shared -ffp-contract=off \
    -I"$HERE/shim" -I"$FW/include" -I"$FW/src" \
    "${SRC[@]}" -o "$HERE/libcontrolbench.so"

echo "built: $HERE/libcontrolbench.so"
