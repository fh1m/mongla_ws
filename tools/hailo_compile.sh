#!/usr/bin/env bash
# The permanent Hailo compile harness. Idempotent: run it again any time.
#
# This exists because the toolchain kept being stood up by hand and thrown
# away, and each rebuild rediscovered the same two traps at the same cost.
#
#   ./tools/hailo_compile.sh gate_sharks --nms-score-th 0.05
#   ./tools/hailo_compile.sh --check            # verify the toolchain only
#
# ⛔ TRAP 1 -- PYTHONPATH. A sourced ROS overlay puts its own site-packages
# AHEAD of the venv's, so the venv is shadowed and the DFC imports the wrong
# onnx (1.22 rather than 1.16). It surfaces as
#     ModuleNotFoundError: No module named 'onnx.mapping'
# which reads like a dependency bug and is not: pinning onnx changes nothing,
# because pip writes to a directory Python never reaches. Hence `env -u`.
#
# ⛔ TRAP 2 -- the DFC and HailoRT are version-locked. The vehicle runs
# HailoRT 4.24.0, which pairs with DFC 3.x. The 5.3.0 wheel sitting beside the
# 3.34.0 one pairs with HailoRT 5.x and yields a HEF the Pi refuses to load,
# reporting an architecture mismatch that looks like a hardware fault.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

log() { printf '[hailo] %s\n' "$*"; }

# ⛔ $HOME IS NOT THE USER'S HOME HERE. Inside the dev container it is
# `/home/fh1m/Envs/dockers/auv-ros2`, so `~/Downloads` and `~/hailo-dfc-venv`
# both resolve to paths that do not exist while looking perfectly reasonable.
# Resolve against real candidates instead of trusting one variable.
_first_dir() { for d in "$@"; do [ -d "$d" ] && { echo "$d"; return; }; done; }
_first_file() { for f in "$@"; do [ -f "$f" ] && { echo "$f"; return; }; done; }

REAL_HOME="$(_first_dir "/home/$(id -un)" "$HOME" "$(getent passwd "$(id -un)" | cut -d: -f6)")"
VENV="${HAILO_DFC_VENV:-$REAL_HOME/hailo-dfc-venv}"
WHEEL="${HAILO_DFC_WHEEL:-$(_first_file \
    "$REAL_HOME"/Downloads/hailo_dataflow_compiler-3.*-py3-none-linux_x86_64.whl \
    "$HOME"/Downloads/hailo_dataflow_compiler-3.*-py3-none-linux_x86_64.whl \
    /run/host/home/*/Downloads/hailo_dataflow_compiler-3.*-py3-none-linux_x86_64.whl)}"

if [ ! -x "$VENV/bin/python" ]; then
    log "creating venv at $VENV"
    python3 -m venv "$VENV"
fi

# ⚠ Probe with the SAME cleaned environment the compile uses. Probing with the
# ambient one would find the ROS overlay's packages and report a working
# toolchain that then fails the moment it matters.
if ! env -u PYTHONPATH "$VENV/bin/python" -c 'import hailo_sdk_client' 2>/dev/null; then
    if [ ! -f "$WHEEL" ]; then
        log "ERROR: DFC wheel not found at $WHEEL"
        log "       Set HAILO_DFC_WHEEL. Must be a 3.x wheel: the vehicle runs"
        log "       HailoRT 4.24.0 and a 5.x HEF will not load on it."
        exit 1
    fi
    log "installing $(basename "$WHEEL") -- several minutes, ~2 GB"
    env -u PYTHONPATH "$VENV/bin/python" -m pip install -q --upgrade pip setuptools wheel
    env -u PYTHONPATH "$VENV/bin/python" -m pip install -q "$WHEEL"
fi

# Calibration reads real video and the class sidecar, and neither ships with
# the DFC. Pinned below 4.11 because the DFC holds numpy at 1.26.
if ! env -u PYTHONPATH "$VENV/bin/python" -c 'import cv2, yaml' 2>/dev/null; then
    log "adding calibration deps (opencv, pyyaml)"
    env -u PYTHONPATH "$VENV/bin/python" -m pip install -q \
        "opencv-python-headless<4.11" pyyaml
fi

env -u PYTHONPATH PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python \
    "$VENV/bin/python" -c '
import hailo_sdk_client as c, onnx
from hailo_sdk_client import ClientRunner
ClientRunner(hw_arch="hailo8")
print(f"[hailo] DFC {c.__version__}, onnx {onnx.__version__}, hailo8 OK")
if not c.__version__.startswith("3."):
    raise SystemExit(
        f"[hailo] REFUSING: DFC {c.__version__} does not pair with the "
        f"vehicle HailoRT 4.24.0. Install a 3.x wheel.")
'

if [ "${1:-}" = "--check" ]; then
    log "toolchain OK"
    exit 0
fi

exec env -u PYTHONPATH PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python \
    "$VENV/bin/python" "$HERE/hailo_compile.py" "$@"
