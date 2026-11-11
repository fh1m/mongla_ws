#!/usr/bin/env bash
# Kutum's health check: is this machine the compute module Kutum declares?
#
#   kutum/doctor.sh                      # on the vehicle
#   ssh fh1m@mongla.local bash -s < kutum/doctor.sh     # from the dev box
#
# Read-only. It changes nothing and never needs sudo. Every line is one of
#   ok        as declared
#   WARNING   runs, but worse than declared (slow, noisy, or a trap armed)
#   FAIL      the stack cannot run on this machine as it is
# and it exits 1 on any FAIL, so `kutum/deploy.sh` can stop on it.
#
# What it does NOT grade: the board, the cameras' images, the models. Those are
# the vehicle's, and `ros2 run mongla_manager bringup_check` grades them.
# This grades the machine underneath: the OS, ROS, the Hailo, Python, the
# system files kutum/system/ installs, and what boots.
#
# The declared system files are compared byte for byte when the repo is beside
# this script (on the vehicle, ~/mongla_ws/kutum/). Piped over ssh there is no
# repo, so those checks compare against the deployed copy instead.
set -uo pipefail
# hailortcli below would otherwise leave hailort.log in the caller's directory
# (env.sh sets the same path for interactive shells).
export HAILORT_LOGGER_PATH="${HAILORT_LOGGER_PATH:-$HOME/.hailo/hailort}"

FAILS=0
ok()   { printf '  ok        %s\n' "$*"; }
warn() { printf '  WARNING   %s\n' "$*"; }
fail() { printf '  FAIL      %s\n' "$*"; FAILS=$((FAILS + 1)); }
say()  { printf '\n%s\n' "$*"; }

HERE="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" 2>/dev/null && pwd)"
if [ -d "$HERE/system" ]; then KUTUM="$HERE"; else KUTUM="$HOME/mongla_ws/kutum"; fi

# --------------------------------------------------------------------------- #
say "The machine"
. /etc/os-release
model="$(tr -d '\0' < /proc/device-tree/model 2>/dev/null || echo unknown)"
case "$model" in
  *"Raspberry Pi 5"*) ok "$model" ;;
  *) warn "$model -- Kutum is a Raspberry Pi 5; numbers measured there may not hold" ;;
esac
[ "$VERSION_ID" = "24.04" ] && ok "$PRETTY_NAME" \
  || fail "$PRETTY_NAME -- ROS 2 Jazzy's binaries are built for Ubuntu 24.04"
ok "kernel $(uname -r), $(free -g | awk '/Mem:/{print $2}') GB RAM"

free_gb="$(df -BG --output=avail "$HOME" | tail -1 | tr -dc '0-9')"
if   [ "${free_gb:-0}" -ge 5 ]; then ok "${free_gb} GB free in \$HOME"
else fail "only ${free_gb} GB free in \$HOME -- a clean build and one bag need 5"; fi

thr="$(vcgencmd get_throttled 2>/dev/null | cut -d= -f2)"
temp="$(( $(cat /sys/class/thermal/thermal_zone0/temp 2>/dev/null || echo 0) / 1000 ))"
if [ -z "$thr" ]; then warn "vcgencmd missing -- throttling cannot be read"
elif [ "$thr" = "0x0" ]; then ok "never throttled since boot, ${temp} C now"
else warn "throttled=$thr since boot (${temp} C now): undervoltage or heat; the detector rate will not hold"; fi

# The clock setup asks for (measured-bars 82). The firmware reports what it
# was told, so a card rebuilt without the system half shows as stock here.
arm="$(vcgencmd get_config arm_freq 2>/dev/null | cut -d= -f2)"
if   [ -z "$arm" ]; then warn "vcgencmd missing -- the ARM clock cannot be read"
elif [ "$arm" = 2800 ]; then ok "ARM clock: 2800 MHz"
else warn "ARM clock: $arm MHz, not the 2800 setup.sh --system writes -- about 15 % less CPU and Hailo rate under full load"; fi

# Batighor's web app (src/mongla_batighor): the pinned Lichtblick build.
lbv="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["version"])' "$(dirname "${BASH_SOURCE[0]}")/../src/mongla_batighor/LICHTBLICK.json" 2>/dev/null)"
if [ -f "$HOME/.mongla/batighor/web/$lbv/index.html" ]; then ok "Batighor: Lichtblick $lbv installed"
else warn "Batighor: Lichtblick ${lbv:-?} not installed -- the operator center answers 503 (ros2 run mongla_batighor fetch)"; fi

gov="$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null)"
[ "$gov" = performance ] && ok "CPU governor: performance" \
  || warn "CPU governor: $gov -- the first milliseconds of every frame run at the idle clock (cpu-performance.service)"

# --------------------------------------------------------------------------- #
say "ROS 2"
if [ -f /opt/ros/jazzy/setup.bash ]; then ok "ROS 2 Jazzy installed"
else fail "no /opt/ros/jazzy -- the vehicle runs Jazzy (dev boxes run Humble; the two do not interoperate)"; fi
command -v colcon >/dev/null && ok "colcon" || fail "colcon missing (python3-colcon-common-extensions)"

login_shell="$(getent passwd "$USER" | cut -d: -f7)"
rc="$HOME/.$(basename "$login_shell")rc"
if grep -qs 'mongla_ws/kutum/env.sh' "$rc"; then ok "$rc sources kutum/env.sh"
else warn "$rc does not source kutum/env.sh -- an ssh terminal starts without ROS (run kutum/setup.sh)"; fi
for f in "$HOME/.bashrc" "$HOME/.zshrc"; do
  if grep -qsE '^[^#]*(duburi_ws|pi_env\.sh)' "$f"; then
    warn "$f still sources a retired workspace or ~/pi_env.sh"
  fi
done

# --------------------------------------------------------------------------- #
say "The Hailo-8"
if [ -e /dev/hailo0 ]; then ok "/dev/hailo0"
else fail "no /dev/hailo0 -- the detector has no flight path (driver not loaded, or the HAT not seated)"; fi
rt="$(hailortcli --version 2>/dev/null | grep -oE '[0-9]+\.[0-9]+\.[0-9]+' | head -1)"
fw="$(hailortcli fw-control identify 2>/dev/null | sed -n 's/^Firmware Version: \([0-9.]*\).*/\1/p')"
if [ -z "$rt" ]; then fail "hailortcli missing -- HailoRT is not installed"
elif [ -z "$fw" ]; then fail "HailoRT $rt, but the device does not answer (another process holds it?)"
elif [ "$rt" = "$fw" ]; then ok "HailoRT $rt, firmware $fw"
else fail "HailoRT $rt against firmware $fw -- they must match exactly"; fi
pydrv="$(python3 -c 'import hailo_platform; print(hailo_platform.__version__)' 2>/dev/null)"
if [ -z "$pydrv" ]; then fail "hailo_platform does not import (kutum/setup.sh builds it from source)"
elif [ "$pydrv" = "$rt" ]; then ok "hailo_platform $pydrv"
else fail "hailo_platform $pydrv against HailoRT $rt"; fi
link="/sys/bus/pci/devices/0000:01:00.0"
if [ -r "$link/current_link_speed" ]; then
  speed="$(cat "$link/current_link_speed")"
  case "$speed" in
    8.0*) ok "PCIe link $speed x$(cat "$link/current_link_width") (Gen 3)" ;;
    *)    warn "PCIe link $speed -- Gen 2 halves the Hailo's bandwidth (dtparam=pciex1_gen=3)" ;;
  esac
fi

# --------------------------------------------------------------------------- #
say "Python (the traps in .claude/context/kutum/pi-and-env-traps.md)"
read -r np cv2f trk < <(python3 - <<'PY' 2>/dev/null
import importlib.metadata as md
import numpy, cv2
try:
    t = md.version('trackers')
except Exception:
    t = 'absent'
print(numpy.__version__, cv2.__file__, t)
PY
)
case "$np" in
  1.*) ok "numpy $np" ;;
  "")  fail "numpy or cv2 does not import" ;;
  *)   fail "numpy $np -- cv_bridge is built against numpy 1 and every vision node dies (E1)" ;;
esac
case "$cv2f" in
  /usr/*) ok "cv2 is the system build" ;;
  "")     ;;
  *)      fail "cv2 from $cv2f shadows the system build (E2)" ;;
esac
case "$trk" in
  2.4.0) ok "trackers 2.4.0" ;;
  *)     warn "trackers $trk -- 2.5.0 on PyPI ships no code; OC-SORT silently falls back (E3)" ;;
esac
if readlink -f /usr/lib/aarch64-linux-gnu/libblas.so.3 | grep -q openblas; then ok "numpy BLAS is OpenBLAS"
else warn "numpy runs on the reference BLAS -- the anchor matcher is ~4.5x slower (libopenblas0-pthread)"; fi

# --------------------------------------------------------------------------- #
say "The board's serial link"
id -nG | tr ' ' '\n' | grep -qx dialout && ok "$USER is in dialout" \
  || fail "$USER is not in dialout -- the board will not open"
[ "$(dpkg-query -W -f='${Status}' brltty 2>/dev/null)" = "install ok installed" ] && warn "brltty installed -- it steals the board's CH340 (E5)" || ok "brltty absent"
modinfo ch341 >/dev/null 2>&1 && ok "ch341 driver present" || fail "no ch341 module -- the board gets no /dev/ttyUSB* (E5)"

# --------------------------------------------------------------------------- #
say "System files (kutum/system/)"
same() {   # declared file, installed path, what it is
  # A root-only file (netplan is 0600) is compared through sudo -n, which
  # never prompts: Kutum's owner has passwordless sudo (setup.sh --system).
  if [ ! -e "$2" ]; then warn "$3 not installed ($2)"
  elif [ ! -f "$1" ]; then ok "$3 installed (no repo copy here to compare)"
  elif [ ! -r "$2" ] && ! sudo -n true 2>/dev/null; then ok "$3 installed (root-only; not compared without sudo)"
  elif { [ -r "$2" ] && cmp -s "$1" "$2"; } || { [ ! -r "$2" ] && sudo -n cmp -s "$1" "$2"; }; then
    ok "$3 matches the repo"
  else warn "$3 differs from kutum/system ($2) -- run kutum/setup.sh --system"; fi
}
same "$KUTUM/system/udev/99-mongla-cameras.rules" /etc/udev/rules.d/99-mongla-cameras.rules "camera names"
same "$KUTUM/system/udev/51-hailo-udev.rules"     /etc/udev/rules.d/51-hailo-udev.rules     "Hailo device permissions"
same "$KUTUM/system/systemd/set-cpu-performance"  /usr/local/sbin/set-cpu-performance      "governor script"
same "$KUTUM/system/systemd/cpu-performance.service" /etc/systemd/system/cpu-performance.service "cpu-performance.service"
same "$KUTUM/system/netplan/60-kutum-eth.yaml"  /etc/netplan/60-kutum-eth.yaml          "Ethernet link-local"
systemctl is-enabled cpu-performance.service >/dev/null 2>&1 && ok "cpu-performance.service enabled" \
  || warn "cpu-performance.service not enabled"
[ -e /dev/srot ] && ok "/dev/srot (the board, pinned to its port)" \
  || warn "no /dev/srot -- the board is found by searching (kutum/system/udev/make_srot_rule.sh pins it)"

# --------------------------------------------------------------------------- #
say "Network"
for i in eth0 wlan0; do
  addr="$(ip -4 -br addr show "$i" 2>/dev/null | awk '{print $3}')"
  if [ -n "$addr" ]; then ok "$i $addr"
  elif [ "$i" = eth0 ]; then warn "eth0 has no IPv4 address (no cable, or no DHCP and no link-local)"
  else warn "wlan0 has no address -- not joined (kutum/setup.sh --system asks for a network)"; fi
done
systemctl is-active avahi-daemon >/dev/null 2>&1 && ok "avahi-daemon: answers as $(hostname).local" \
  || fail "avahi-daemon not running -- $(hostname).local will not resolve from the laptop"
installed() { [ "$(dpkg-query -W -f='${Status}' "$1" 2>/dev/null)" = "install ok installed" ]; }
installed wpasupplicant && ok "wpasupplicant installed" \
  || warn "wpasupplicant missing -- Wi-Fi cannot join anything"

# --------------------------------------------------------------------------- #
say "The clock"
# The Pi 5 RTC keeps time through a power-off only on its battery. Without
# it, a vehicle booted at a pool with no network starts in the past, and every
# log, bag and run folder is stamped with the wrong day.
if [ -e /sys/class/rtc/rtc0 ]; then
  ok "RTC: $(cat /sys/class/rtc/rtc0/name)"
  chg=/sys/class/rtc/rtc0/charging_voltage
  v="$(cat "$chg" 2>/dev/null || echo 0)"
  if [ "${v:-0}" -gt 0 ]; then ok "RTC battery charging at $((v / 1000)) mV"
  else warn "RTC battery not charged (dtparam=rtc_bbat_vchg) -- a rechargeable cell runs flat"; fi
  # The cell itself, read by the PMIC. 2.88 V on 2026-11-07, charging to 3.0.
  bv="$(vcgencmd pmic_read_adc BATT_V 2>/dev/null | sed -n 's/.*=\([0-9.]*\)V.*/\1/p')"
  if [ -z "$bv" ]; then warn "RTC battery voltage unreadable (vcgencmd pmic_read_adc BATT_V)"
  elif awk -v v="$bv" 'BEGIN{exit !(v < 2.5)}'; then
    warn "RTC battery at $(printf %.2f "$bv") V -- below 2.5 V the clock may not survive a power-off"
  else ok "RTC battery at $(printf %.2f "$bv") V"; fi
  # Once network time has set the clock, the kernel writes it to the RTC every
  # 11 minutes, so the two should agree to a second.
  rtc_s="$(cat /sys/class/rtc/rtc0/since_epoch 2>/dev/null || echo)"
  if [ -n "$rtc_s" ]; then
    d=$(( $(date +%s) - rtc_s )); d=${d#-}
    if [ "$d" -le 2 ]; then ok "RTC and system clock agree (${d} s)"
    else warn "RTC is ${d} s from the system clock"; fi
  fi
else
  warn "no RTC -- time after a power-off comes only from the network"
fi
if timedatectl show -p NTPSynchronized --value 2>/dev/null | grep -qx yes; then ok "system clock synchronised"
else warn "system clock NOT synchronised (no network time since boot; running on the RTC)"; fi

# --------------------------------------------------------------------------- #
say "What boots"
# A unit whose program does not exist fails with 203/EXEC and is restarted
# for ever. One did, and its stop command kills whatever holds the camera and
# the Hailo -- so it is a FAIL, not tidiness.
for u in $(systemctl list-units --type=service --all --no-legend --plain 2>/dev/null \
            | awk '$3 == "activating" || $3 == "failed" {print $1}'); do
  exe="$(systemctl show -p ExecStart --value "$u" | sed -n 's/.*path=\([^ ;]*\).*/\1/p' | head -1)"
  if [ "$(systemctl show -p ExecMainStatus --value "$u")" != 203 ]; then
    warn "$u is $(systemctl is-active "$u")"
  elif [ "$(systemctl show -p SubState --value "$u")" = auto-restart ]; then
    fail "$u restarts for ever: $exe does not exist (sudo systemctl disable --now $u)"
  else
    warn "$u cannot start: $exe does not exist"
  fi
done

# --------------------------------------------------------------------------- #
say "The deployed tree"
stamp="$HOME/mongla_ws/.kutum-deployed"
if [ ! -e "$HOME/mongla_ws" ]; then
  # A rebuilt card: deploy runs this doctor before it builds the first release,
  # so a FAIL here would stop the one command that fixes it.
  warn "no release is active yet -- kutum/deploy.sh builds and activates one"
else
  if [ -L "$HOME/mongla_ws" ]; then
    ok "active release: $(basename "$(readlink -f "$HOME/mongla_ws")") (kutum/release.sh list)"
    gate="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1])).get("vehicle_gate","?"))' \
            "$HOME/mongla_ws/release.json" 2>/dev/null)"
    # The machine can be sound while the vehicle is not; that is the gate's
    # verdict to give, so it is a WARNING here, never a FAIL.
    case "$gate" in
      pass) ok "its vehicle gate passed" ;;
      "")   ;;
      *)    warn "its vehicle gate: $gate -- run ros2 run mongla_manager bringup_check" ;;
    esac
  else
    warn "$HOME/mongla_ws is a plain directory -- the next deploy keeps it as a legacy release"
  fi
  if [ -f "$stamp" ]; then ok "deployed: $(cat "$stamp")"
  else warn "no deploy stamp -- ~/mongla_ws was not put there by kutum/deploy.sh"; fi
  [ -f "$HOME/mongla_ws/install/setup.bash" ] && ok "workspace built" || fail "$HOME/mongla_ws is not built"
fi
# Everything the package.xml files declare is installed (rosdep reads them).
if [ -d "$HOME/mongla_ws/src" ] && command -v rosdep >/dev/null; then
  if rd="$(rosdep check --rosdistro jazzy --from-paths "$HOME/mongla_ws/src" --ignore-src 2>&1)"; then
    ok "rosdep: every package.xml dependency installed"
  else warn "rosdep: $(echo "$rd" | grep -m1 -iE 'apt|missing|init|update' | cut -c1-90)"; fi
fi

echo
if [ "$FAILS" -eq 0 ]; then echo "Kutum: no FAIL."; exit 0; fi
echo "Kutum: $FAILS FAIL."; exit 1
