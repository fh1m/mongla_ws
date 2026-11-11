#!/usr/bin/env bash
# Make a Raspberry Pi 5 + Hailo-8 into Kutum, the vehicle's compute module.
# Ubuntu 24.04 (Noble) + ROS 2 Jazzy. Idempotent: run it again after any change.
#
#   sudo kutum/setup.sh --system the machine half: apt packages, the Hailo
#                                driver, firmware and runtime, the files in
#                                kutum/system/, what boots -- then the user
#                                half below. A fresh machine needs only this.
#   kutum/setup.sh               the user half alone: Python pins, HailoRT
#                                bindings, the shell lines. No sudo.
#
# Then `kutum/deploy.sh` (from the dev box) syncs, builds and gates the stack,
# and `kutum/doctor.sh` says whether the machine still matches this file.
#
# The traps each step avoids are written up in
# .claude/context/kutum/pi-and-env-traps.md (E1-E5).
#
# Not `set -u`: ROS's own setup files reference unbound variables
# (AMENT_TRACE_SETUP_FILES), and nounset kills the script on them.
set -eo pipefail

KUTUM="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# One version for the driver, the firmware, the runtime and the Python
# bindings. They must match exactly; the doctor fails a machine where they do not.
HAILORT_V=4.24.0
# Wi-Fi regulatory domain (ISO 3166). Bangladesh; change it where the vehicle goes.
KUTUM_WIFI_COUNTRY="${KUTUM_WIFI_COUNTRY:-BD}"
say() { printf '\n== %s\n' "$*"; }

# What `--system` returns is the doctor's verdict on the finished machine
# (issue #69): it used to swallow a FAIL and exit 0, so a machine that could
# not run the stack read as provisioned. $1 is the doctor's exit code, $2 the
# steps that only take effect on a reboot. A FAIL before such a reboot may be
# one of them, so it gets its own code and an instruction, never a 0.
verdict() {
  if [ "$1" -eq 0 ]; then
    if [ -n "$2" ]; then echo "Kutum is ready. Reboot to apply:$2"
    else echo "Kutum is ready."; fi
    return 0
  fi
  if [ -n "$2" ]; then
    echo "The doctor FAILed, and this setup changed something that takes effect" \
         "on reboot:$2. Reboot, then run kutum/doctor.sh: a FAIL then is real." >&2
    return 3
  fi
  echo "The doctor FAILed: this machine cannot run the stack as it is." \
       "Fix each FAIL above and run setup again." >&2
  return 1
}

# =========================================================================== #
if [ "${1:-}" = "--system" ]; then
  [ "$(id -u)" -eq 0 ] || { echo "run as: sudo $0 --system" >&2; exit 2; }
  OWNER="${SUDO_USER:?run through sudo, so the vehicle user is known}"
  OWNER_HOME="$(getent passwd "$OWNER" | cut -d: -f6)"
  STAMP="$(date +%F)"

  say "1/8  apt: exactly what the package.xml files declare, plus operator tools"
  # A stale entry here is a package the vehicle installs and nothing imports.
  # mosh and tmux are off the mission path: a mission runs inside tmux over
  # mosh so a dropped tether never kills it (kutum/remote-access.md).
  PKGS="ros-jazzy-vision-msgs ros-jazzy-cv-bridge ros-jazzy-foxglove-bridge
        ros-jazzy-rosbag2-storage-mcap ros-jazzy-web-video-server
        python3-numpy python3-opencv python3-pil python3-serial python3-yaml
        python3-colcon-common-extensions python3-rosdep
        libopenblas0-pthread mosh tmux wpasupplicant avahi-daemon"
  MISSING=""
  # dpkg-query, not `dpkg -s`: a removed package that left its config files
  # behind ("deinstall ok config-files") passes `dpkg -s`, and wpasupplicant
  # stayed missing that way on 2026-11-07.
  for p in $PKGS; do
    [ "$(dpkg-query -W -f='${Status}' "$p" 2>/dev/null)" = "install ok installed" ] \
      || MISSING="$MISSING $p"
  done
  if [ -n "$MISSING" ]; then apt-get install -y -q $MISSING
  else echo "  all present"; fi
  # rosdep reads the package.xml files, so it is the check that they are
  # complete (test_package_xml_declares_imports.py keeps them so). The doctor
  # runs `rosdep check`; it needs the index initialised once.
  [ -e /etc/ros/rosdep/sources.list.d/20-default.list ] || rosdep init
  sudo -u "$OWNER" -H rosdep update -q

  # brltty claims every CH340 (1a86:7523) as a braille display and holds it
  # through usbfs, so the SROT board never gets a /dev/ttyUSB* node (E5).
  if [ "$(dpkg-query -W -f='${Status}' brltty 2>/dev/null)" = "install ok installed" ]; then
    apt-get purge -y -q brltty
  fi
  id -nG "$OWNER" | tr ' ' '\n' | grep -qx dialout || usermod -aG dialout "$OWNER"

  say "2/8  sudo without a password for $OWNER"
  # The owner's decision (2026-11-07): this is a single-purpose compute module
  # on the vehicle's own network, and a password prompt in the middle of a
  # pool session costs more than it protects. Delete the file to undo it.
  rule="/etc/sudoers.d/90-kutum-$OWNER"
  echo "$OWNER ALL=(ALL) NOPASSWD: ALL" > "$rule.tmp"
  chmod 0440 "$rule.tmp"
  if visudo -cf "$rule.tmp" >/dev/null; then mv "$rule.tmp" "$rule"; echo "  $rule"
  else rm -f "$rule.tmp"; echo "  visudo refused the rule; nothing changed" >&2; exit 1; fi

  say "3/8  the Hailo-8: driver, firmware, runtime ($HAILORT_V, from source)"
  # All three come from Hailo's public repositories, so a replacement Pi
  # needs no Developer Zone login. This is how the current vehicle was built
  # (by hand, 2026-08-28); as one script it has not yet run on a fresh card.
  as_owner() { sudo -u "$OWNER" -H "$@"; }
  if ! dkms status "hailo_pci/$HAILORT_V" -k "$(uname -r)" 2>/dev/null | grep -q installed; then
    apt-get install -y -q build-essential cmake git dkms wget "linux-headers-$(uname -r)"
    [ -d "$OWNER_HOME/hailort_drivers" ] || as_owner git clone --depth 1 -b "v$HAILORT_V" \
        https://github.com/hailo-ai/hailort-drivers.git "$OWNER_HOME/hailort_drivers"
    make -C "$OWNER_HOME/hailort_drivers/linux/pcie" install_dkms
    (cd "$OWNER_HOME/hailort_drivers" && as_owner ./download_firmware.sh)
    install -D -m 0644 "$OWNER_HOME/hailort_drivers/hailo8_fw.$HAILORT_V.bin" \
        /lib/firmware/hailo/hailo8_fw.bin
    modprobe hailo_pci
  else echo "  driver hailo_pci $HAILORT_V present for $(uname -r)"; fi
  if [ "$(hailortcli --version 2>/dev/null | grep -oE '[0-9]+\.[0-9]+\.[0-9]+' | head -1)" != "$HAILORT_V" ]; then
    [ -d "$OWNER_HOME/hailort_src" ] || as_owner git clone --depth 1 -b "v$HAILORT_V" \
        https://github.com/hailo-ai/hailort.git "$OWNER_HOME/hailort_src"
    # -j2: a 4 GB Pi runs out of memory at -j4.
    as_owner cmake -S "$OWNER_HOME/hailort_src" -B "$OWNER_HOME/hailort_src/build" \
        -DCMAKE_BUILD_TYPE=Release
    as_owner cmake --build "$OWNER_HOME/hailort_src/build" -j2
    cmake --install "$OWNER_HOME/hailort_src/build"
    ldconfig
  else echo "  HailoRT $HAILORT_V present"; fi

  say "4/8  kutum/system/ -> the machine"
  install -m 0644 "$KUTUM/system/udev/99-mongla-cameras.rules" /etc/udev/rules.d/
  install -m 0644 "$KUTUM/system/udev/51-hailo-udev.rules"     /etc/udev/rules.d/
  install -m 0755 "$KUTUM/system/systemd/set-cpu-performance"  /usr/local/sbin/
  install -m 0644 "$KUTUM/system/systemd/cpu-performance.service" /etc/systemd/system/
  udevadm control --reload-rules
  udevadm trigger --subsystem-match=video4linux --subsystem-match=hailo_chardev
  systemctl daemon-reload
  systemctl enable --now cpu-performance.service

  say "5/8  units that start a program that no longer exists"
  # hailo-web.service (a model switcher from before this repo) pointed into
  # ~/hailo, which is gone: it failed with 203/EXEC and was restarted every
  # 3 s for ever, and its ExecStop kills whatever holds /dev/video0 and
  # /dev/hailo0. Disabled and moved aside, not deleted.
  for unit in /etc/systemd/system/*.service; do
    # Only unit files written here. The rest are symlinks into /usr/lib --
    # aliases, some dangling once their package is removed -- and reading a
    # dangling one under `set -e` stopped this script on 2026-11-07.
    [ -f "$unit" ] && [ ! -L "$unit" ] || continue
    exe="$(sed -n 's/^ExecStart=-\{0,1\}\([^ ]*\).*/\1/p' "$unit" | head -1)"
    case "$exe" in
      "$OWNER_HOME"/*)
        if [ ! -e "$exe" ]; then
          name="$(basename "$unit")"
          echo "  $name starts $exe, which does not exist: disabling"
          systemctl disable --now "$name" || true
          mkdir -p "$OWNER_HOME/attic/$STAMP/systemd"
          mv "$unit" "$OWNER_HOME/attic/$STAMP/systemd/"
          chown -R "$OWNER:$OWNER" "$OWNER_HOME/attic/$STAMP"
        fi ;;
    esac
  done
  systemctl daemon-reload
  systemctl reset-failed

  say "6/8  network: Wi-Fi on boot, and mongla.local on every interface"
  # Ethernet stays as cloud-init wrote it (DHCP); 60-kutum-eth.yaml only adds a
  # link-local address, so a cable straight to a laptop with no DHCP server
  # still answers on mongla.local. A laptop sharing its internet over that
  # cable (NetworkManager: IPv4 "Shared to other computers") hands out an
  # address by DHCP, and the Pi takes it like any router's.
  install -m 0600 "$KUTUM/system/netplan/60-kutum-eth.yaml" /etc/netplan/
  # The Wi-Fi password is typed here, by whoever runs this, and lands only in
  # /etc/netplan (mode 0600) -- never in the repo. Rerun to change networks.
  wifi=/etc/netplan/61-kutum-wifi.yaml
  if [ ! -f "$wifi" ] && [ -t 0 ]; then
    read -r -p "  Wi-Fi network to join on boot (empty: skip): " ssid
    if [ -n "$ssid" ]; then
      read -r -s -p "  password for $ssid: " psk; echo
      umask 077
      printf 'network:\n  version: 2\n  wifis:\n    wlan0:\n      optional: true\n      dhcp4: true\n      regulatory-domain: "%s"\n      access-points:\n        "%s":\n          password: "%s"\n' \
        "$KUTUM_WIFI_COUNTRY" "$ssid" "$psk" > "$wifi"
      umask 022
      unset psk
    fi
  fi
  rfkill unblock wifi 2>/dev/null || true
  netplan generate
  # apply re-reads every interface; on eth0 that can drop this ssh session
  # for a second or two. It comes back on the same address.
  REBOOT=""
  netplan apply || { echo "  netplan apply failed: the change takes effect on reboot"
                     REBOOT="$REBOOT netplan"; }
  systemctl enable --now avahi-daemon

  say "7/8  the RTC: keep its battery charged"
  # The Pi 5's clock keeps time through a power-off only on its battery, and
  # the firmware leaves charging off. The official RTC battery (a rechargeable
  # ML-2020) is charged at 3.0 V, Raspberry Pi's documented value. Do NOT
  # leave this on with a non-rechargeable cell. Takes effect on the next boot.
  cfg=/boot/firmware/config.txt
  if ! grep -q '^dtparam=rtc_bbat_vchg=' "$cfg"; then
    printf '\n# kutum: charge the RTC battery (rechargeable ML-2020 only)\ndtparam=rtc_bbat_vchg=3000000\n' >> "$cfg"
    echo "  charging enabled in $cfg; reboot to apply"
    REBOOT="$REBOOT rtc-charging"
  else echo "  already set: $(grep '^dtparam=rtc_bbat_vchg=' "$cfg")"; fi

  say "8/8  the user half, as $OWNER"
  sudo -u "$OWNER" -H bash "$KUTUM/setup.sh"
  rc=0
  sudo -u "$OWNER" -H bash "$KUTUM/doctor.sh" || rc=$?
  verdict "$rc" "$REBOOT"
  exit $?
fi

# =========================================================================== #
[ "$(id -u)" -ne 0 ] || { echo "the user half runs as the vehicle user, not root" >&2; exit 2; }

say "1/5  Python: undo the two traps that break cv_bridge (E1, E2)"
# Noble ships numpy 1.26.4 and cv2 4.6.0, and ROS Jazzy's cv_bridge is built
# against numpy 1. A pip numpy>=2 or a pip opencv-python in ~/.local shadows
# them and every vision node dies with "_ARRAY_API not found".
python3 -m pip uninstall -y -q --break-system-packages numpy opencv-python \
        opencv-python-headless 2>/dev/null || true

say "2/5  Python: the vision dependencies, pinned (kutum/requirements.txt)"
# --no-deps on the two that hard-require opencv-python, which would re-shadow
# the system cv2. supervision 0.30 needs numpy 2; trackers 2.5.0 on PyPI ships
# no code (E3).
python3 -m pip install -q --user --break-system-packages \
        -c "$KUTUM/constraints.txt" -r "$KUTUM/requirements.txt"
python3 -m pip install -q --user --break-system-packages --no-deps \
        'supervision==0.26.1' 'trackers==2.4.0'

say "3/5  HailoRT Python bindings (not on PyPI: built from source)"
if [ "$(python3 -c 'import hailo_platform; print(hailo_platform.__version__)' 2>/dev/null)" != "$HAILORT_V" ]; then
  # The wheel is behind a Hailo Developer Zone login, so build it from the
  # same tree the runtime came from (--system step 2). -DHAILO_BUILD_PYBIND
  # on the top-level cmake does nothing: the bindings have their own setup.py.
  command -v hailortcli >/dev/null || { echo "no HailoRT: run sudo $0 --system first" >&2; exit 1; }
  [ -d ~/hailort_src ] || git clone --depth 1 -b "v$HAILORT_V" \
      https://github.com/hailo-ai/hailort.git ~/hailort_src
  (cd ~/hailort_src/hailort/libhailort/bindings/python/platform \
      && python3 -m pip install --user --break-system-packages --no-build-isolation .)
fi

say "4/5  Every shell sources kutum/env.sh"
# The login shell is zsh, and ROS ships a setup file per shell; ~/.zshrc used to
# source nothing, so every ssh terminal started without ROS. Lines that source
# a retired workspace or ~/pi_env.sh are commented out, not deleted.
line='[ -f ~/mongla_ws/kutum/env.sh ] && source ~/mongla_ws/kutum/env.sh   # kutum'
for rc in ~/.zshrc ~/.bashrc; do
  [ -f "$rc" ] || continue
  sed -i -E 's@^([^#].*(duburi_ws|pi_env\.sh|/opt/ros/jazzy/setup\.bash|mongla_ws/install/setup\.bash).*)$@# replaced by kutum/env.sh: \1@' "$rc"
  grep -qF 'mongla_ws/kutum/env.sh' "$rc" || printf '\n%s\n' "$line" >> "$rc"
done
mkdir -p ~/.hailo/hailort

say "5/5  Batighor: the Lichtblick web build the operators' browsers load"
# The pinned release, checked against its hash (src/mongla_batighor/LICHTBLICK.json),
# into ~/.mongla/batighor/web/. Run from the source tree: this half can come
# before the workspace is built.
PYTHONPATH="$KUTUM/../src/mongla_batighor${PYTHONPATH:+:$PYTHONPATH}" \
  python3 -m mongla_batighor.fetch \
  || echo "  WARNING: Lichtblick not installed; batighor:=true will say so. Retry with network."

echo
echo "User half done. On a fresh machine the machine half comes first:"
echo "    sudo $KUTUM/setup.sh --system     (it runs this half too)"
