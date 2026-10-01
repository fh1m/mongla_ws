#!/usr/bin/env bash
# Pin THIS SROT board to /dev/srot by its PHYSICAL USB port (issue #20).
#
# The board's CH340 has no serial number, so its /dev/serial/by-id name is
# shared by every CH340 on the machine. The physical port is the only stable
# identity it has. Run this ONCE on the vehicle with the board plugged into the
# port it will live in; `find_srot_serial()` then prefers /dev/srot, and refuses
# to guess when two CH340s are present without it.
#
#   scripts/udev/make_srot_rule.sh [/dev/ttyUSBx]      # default: the one CH340
#
# Moving the board to a different USB port means running this again.
set -euo pipefail
dev="${1:-}"
if [ -z "$dev" ]; then
  mapfile -t cands < <(ls /dev/serial/by-id/ 2>/dev/null | grep -E '1a86|USB_Serial|CH340' | sed 's#^#/dev/serial/by-id/#')
  if [ "${#cands[@]}" -ne 1 ]; then
    echo "found ${#cands[@]} CH340 devices: ${cands[*]:-none}" >&2
    echo "unplug the others, or pass the board's /dev/ttyUSBx explicitly" >&2
    exit 1
  fi
  dev="$(readlink -f "${cands[0]}")"
fi
path="$(udevadm info -q property -n "$dev" | sed -n 's/^ID_PATH=//p')"
[ -n "$path" ] || { echo "no ID_PATH for $dev" >&2; exit 1; }
rule="SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"1a86\", ENV{ID_PATH}==\"$path\", SYMLINK+=\"srot\", MODE=\"0660\", GROUP=\"dialout\""
echo "$rule"
if [ "${DRY_RUN:-0}" = "1" ]; then exit 0; fi
echo "$rule" | sudo tee /etc/udev/rules.d/99-srot.rules >/dev/null
sudo udevadm control --reload-rules && sudo udevadm trigger --subsystem-match=tty
sleep 1; ls -l /dev/srot
