#!/usr/bin/env bash
#
# install_ch341_driver.sh — make the CH340 payload board enumerate as a
# serial port (/dev/ttyUSB*) on this Jetson, durably across reboots.
#
# WHY THIS EXISTS
#   The payload ESP32 board uses a QinHeng CH340 USB-serial chip (1a86:7523).
#   After the 2026-07 carrier-board + SSD swap the payload stopped connecting
#   even though `lsusb` still showed the CH340.  Two independent causes:
#
#     1. The new Tegra kernel (5.15.185-tegra) shipped with
#        `# CONFIG_USB_SERIAL_CH341 is not set` — no ch341.ko anywhere in
#        /lib/modules — so the chip enumerates on the bus but the kernel
#        never creates /dev/ttyUSB*.  pyserial list_ports then finds nothing
#        and payload auto-detect silently fails.
#
#     2. Ubuntu ships `brltty` (braille display driver), which greedily
#        claims ANY 1a86:7523 as a braille display via a udev rule and holds
#        it through usbfs — blocking ch341 even once the module exists.
#
#   This script fixes both so "plug the payload into any USB port and run
#   `ros2 run mongla_manager start ... -p payload_port:=auto`" just works.
#
# IDEMPOTENT — safe to re-run.  Re-run after any kernel update or SSD reflash.
#
# Requires: kernel headers (/lib/modules/$(uname -r)/build), gcc, make, curl.
set -euo pipefail

KREL="$(uname -r)"
KBUILD="/lib/modules/${KREL}/build"
SERIAL_DIR="/lib/modules/${KREL}/kernel/drivers/usb/serial"
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

echo "==> Jetson CH340/ch341 payload-serial fixup (kernel ${KREL})"

# ---------------------------------------------------------------------------
# 1. Purge brltty (steals the CH340 via usbfs).  --------------------------
# ---------------------------------------------------------------------------
if dpkg -l brltty 2>/dev/null | grep -q '^ii'; then
    echo "==> Removing brltty (grabs 1a86:7523 as a braille display)"
    sudo apt-get purge -y brltty || true
    sudo udevadm control --reload-rules || true
else
    echo "==> brltty not installed — good"
fi

# ---------------------------------------------------------------------------
# 2. Ensure ch341.ko exists for this kernel; build it if missing.  --------
# ---------------------------------------------------------------------------
if modinfo ch341 >/dev/null 2>&1; then
    echo "==> ch341 module already available"
else
    echo "==> ch341 module missing — building out-of-tree against ${KBUILD}"
    [ -d "$KBUILD" ] || { echo "!! kernel headers not found at $KBUILD"; exit 1; }

    # ch341.c matching the 5.15 stable API (driver is stable across 5.15.y).
    SRC_URL="https://raw.githubusercontent.com/torvalds/linux/v5.15/drivers/usb/serial/ch341.c"
    echo "==> Fetching ch341.c ($SRC_URL)"
    curl -fsSL -o "$WORK/ch341.c" "$SRC_URL"

    cat > "$WORK/Makefile" <<EOF
obj-m := ch341.o
all:
	\$(MAKE) -C ${KBUILD} M=$WORK modules
EOF
    echo "==> Compiling"
    make -C "$KBUILD" M="$WORK" modules

    echo "==> Installing to $SERIAL_DIR"
    sudo cp "$WORK/ch341.ko" "$SERIAL_DIR/"
    sudo depmod -a
fi

# ---------------------------------------------------------------------------
# 3. Load it now + bind any already-attached CH340.  ----------------------
# ---------------------------------------------------------------------------
echo "==> Loading ch341"
sudo modprobe ch341 || true

# The chip may have enumerated before ch341 existed → force a re-probe so it
# binds without a physical replug.
for vfile in /sys/bus/usb/devices/*/idVendor; do
    d="$(dirname "$vfile")"
    [ "$(cat "$vfile" 2>/dev/null)" = "1a86" ] || continue
    base="$(basename "$d")"
    for intf in "$d/$base":*; do
        [ -e "$intf" ] || continue
        i="$(basename "$intf")"
        # Drop any stale usbfs/no-driver claim, then let the bus re-probe.
        sudo sh -c "echo -n '$i' > /sys/bus/usb/drivers/usbfs/unbind" 2>/dev/null || true
        sudo sh -c "echo -n '$i' > /sys/bus/usb/drivers_probe" 2>/dev/null || true
    done
done
sleep 1

# ---------------------------------------------------------------------------
# 4. Report.  -------------------------------------------------------------
# ---------------------------------------------------------------------------
if ls /dev/ttyUSB* >/dev/null 2>&1; then
    echo "==> SUCCESS — payload serial node(s):"
    ls -l /dev/ttyUSB*
    echo "    (autoloads on any port at boot/replug via modules.alias 1a86:7523)"
else
    echo "!! No /dev/ttyUSB* yet. Replug the payload board, or check 'lsusb | grep 1a86'."
    exit 1
fi
