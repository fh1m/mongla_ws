#!/usr/bin/env bash
#
# setup_remote_access.sh — make the Jetson pleasant + robust to work on remotely
# from the ground-station laptop, replacing the laggy/fragile xrdp workflow.
#
# WHY THIS EXISTS
#   Testing is driven over a Fathom-X tether by remoting into the Jetson
#   (192.168.2.69). Plain xrdp gives: constant polkit password popups, laggy
#   OpenCV/desktop, typing lag, and a session that wedges to a black screen after
#   a drop (needing a reboot — deadly mid-run). The fix is to stop using ONE
#   remote desktop for everything and split the work by layer:
#
#     1. Terminal (mission authoring/CLI/launches) -> mosh + tmux   (instant echo,
#        survives drops; a mission in tmux keeps running if the GUI dies).
#     2. Vision viewing -> Foxglove/Lichtblick + web_video_server    (no desktop
#        pixels; already built — this script just confirms web_video_server).
#     3. Full desktop (BlueOS calib is web already; QGC; the odd GUI) -> NoMachine
#        (NX) — the Jetson-recommended remote desktop, reconnects cleanly.
#     4. Passwords -> a polkit .pkla granting `mongla_agile` ResultAny=yes for the
#        noisy desktop actions (Ubuntu 22.04 = polkit 0.105 = .pkla, not rules.d).
#
#   Full guide + laptop-side steps + emergency recovery:
#     .claude/context/platform/remote-access.md
#
# IDEMPOTENT — safe to re-run. Re-run after an SSD reflash to restore all of it.
#
# USAGE
#   tools/setup_remote_access.sh [--dummy-display] [--autologin] [/path/to/nomachine_*.deb]
#     --dummy-display : also install the software virtual-display Xorg snippet
#                       (ONLY for a truly headless boot with no monitor AND no
#                       HDMI dummy plug; overrides the real GPU otherwise).
#     --autologin     : enable GNOME autologin for mongla_agile so a desktop session
#                       exists at boot for NoMachine to attach to.
#     <deb path>      : install NoMachine from this local .deb. If omitted, the
#                       script looks in ~/Downloads and honours $NOMACHINE_URL.
#
# Does NOT restart the display manager (that would drop your session). It prints
# the one manual command to finish. sudo password is prompted by sudo itself.
set -uo pipefail   # NOT -e: each step reports and continues so one failure
                   # (e.g. no NoMachine .deb) never blocks the rest.

REPO_DIR="$(cd "$(dirname "$0")/.." && pwd)"
RA_DIR="$REPO_DIR/tools/remote-access"
TARGET_USER="${SUDO_USER:-$(id -un)}"   # the human user, even if run via sudo
PKLA_DST="/etc/polkit-1/localauthority/50-local.d/50-mongla-nopasswd.pkla"

WANT_DUMMY=0 WANT_AUTOLOGIN=0 NOMACHINE_DEB=""
for a in "$@"; do
    case "$a" in
        --dummy-display) WANT_DUMMY=1 ;;
        --autologin)     WANT_AUTOLOGIN=1 ;;
        *.deb)           NOMACHINE_DEB="$a" ;;
        -h|--help)       sed -n '2,40p' "$0"; exit 0 ;;
        *) echo "!! unknown arg: $a (see --help)"; exit 2 ;;
    esac
done

ok()   { echo "  [ok]   $*"; }
warn() { echo "  [warn] $*"; }
step() { echo; echo "==> $*"; }

echo "== Jetson remote-access setup (user: $TARGET_USER) =="

# --- sanity: this is the polkit-0.105 / pkla world we designed for -----------
step "Environment check"
POLKIT_VER="$(pkaction --version 2>/dev/null | awk '{print $NF}')"
case "$POLKIT_VER" in
    0.105*) ok "polkit $POLKIT_VER — using .pkla (correct for Ubuntu 22.04)" ;;
    "")     warn "could not read polkit version — assuming .pkla" ;;
    *)      warn "polkit $POLKIT_VER is newer than 0.105: it may want a JS rule in"
            warn "/etc/polkit-1/rules.d/ instead of .pkla — verify the popups stop." ;;
esac

# --- 1. terminal layer: mosh (+ dummy driver pkg; web_video_server verify) ---
step "Installing mosh (+ virtual-display driver, web_video_server)"
sudo apt-get update -qq || warn "apt update failed (offline?) — trying installs anyway"
sudo apt-get install -y mosh xserver-xorg-video-dummy \
    && ok "mosh + xserver-xorg-video-dummy installed" \
    || warn "apt install failed — install mosh manually when online"
if dpkg -l ros-humble-web-video-server 2>/dev/null | grep -q '^ii'; then
    ok "web_video_server already present (browser MJPEG of image_debug)"
else
    sudo apt-get install -y ros-humble-web-video-server \
        && ok "web_video_server installed" \
        || warn "web_video_server not installed (Foxglove still covers vision)"
fi

# --- 4. kill the password popups: install the polkit .pkla -------------------
step "Installing polkit no-password rule for desktop nuisances"
if [ -f "$RA_DIR/50-mongla-nopasswd.pkla" ]; then
    sudo install -D -m 0644 -o root -g root \
        "$RA_DIR/50-mongla-nopasswd.pkla" "$PKLA_DST" \
        && ok "installed $PKLA_DST (takes effect for NEW sessions; no reboot)" \
        || warn "could not install pkla — check sudo"
else
    warn "missing $RA_DIR/50-mongla-nopasswd.pkla"
fi

# --- 3. NoMachine (NX) — the xrdp replacement --------------------------------
step "NoMachine (NX) remote desktop"
if command -v /usr/NX/bin/nxserver >/dev/null 2>&1; then
    ok "NoMachine already installed: $(/usr/NX/bin/nxserver --version 2>/dev/null | head -1)"
else
    [ -z "$NOMACHINE_DEB" ] && NOMACHINE_DEB="$(ls -1t "$HOME"/Downloads/nomachine_*arm64.deb 2>/dev/null | head -1 || true)"
    if [ -z "$NOMACHINE_DEB" ] && [ -n "${NOMACHINE_URL:-}" ]; then
        echo "  downloading NoMachine from \$NOMACHINE_URL ..."
        NOMACHINE_DEB="/tmp/nomachine_arm64.deb"
        curl -fSL -o "$NOMACHINE_DEB" "$NOMACHINE_URL" || { warn "download failed"; NOMACHINE_DEB=""; }
    fi
    if [ -n "$NOMACHINE_DEB" ] && [ -f "$NOMACHINE_DEB" ]; then
        sudo dpkg -i "$NOMACHINE_DEB" \
            && ok "NoMachine installed — server autostarts (TCP 4000)" \
            || warn "dpkg -i failed on $NOMACHINE_DEB"
    else
        warn "No NoMachine .deb found. Install it (ARM 64-bit DEB for Linux):"
        warn "   download: https://www.nomachine.com/download/download&id=114"
        warn "   then:     tools/setup_remote_access.sh ~/Downloads/nomachine_*_arm64.deb"
        warn "   (mosh + polkit + everything else above is already done.)"
    fi
fi

# --- 3b. NoMachine needs Xorg (not Wayland) ----------------------------------
step "Display server = Xorg (NoMachine + xrdp both need it)"
if [ -f /etc/gdm3/custom.conf ]; then
    if grep -qxE '\s*WaylandEnable\s*=\s*false' /etc/gdm3/custom.conf; then
        ok "WaylandEnable=false already set"
    elif grep -qE '^\s*#?\s*WaylandEnable' /etc/gdm3/custom.conf; then
        sudo sed -i -E 's|^\s*#?\s*WaylandEnable\s*=.*|WaylandEnable=false|' /etc/gdm3/custom.conf \
            && ok "set WaylandEnable=false" || warn "could not edit gdm3 custom.conf"
    else
        sudo sed -i -E 's|^\s*\[daemon\]|[daemon]\nWaylandEnable=false|' /etc/gdm3/custom.conf \
            && ok "added WaylandEnable=false under [daemon]" || warn "could not edit gdm3 custom.conf"
    fi
else
    warn "no /etc/gdm3/custom.conf (not gdm3?) — ensure the desktop runs on Xorg"
fi

# --- optional: autologin so a session exists at boot for NX to attach --------
if [ "$WANT_AUTOLOGIN" = 1 ]; then
    step "Enabling GNOME autologin for $TARGET_USER"
    if [ -f /etc/gdm3/custom.conf ]; then
        sudo sed -i -E \
            -e "s|^\s*#?\s*AutomaticLoginEnable\s*=.*|AutomaticLoginEnable=true|" \
            -e "s|^\s*#?\s*AutomaticLogin\s*=.*|AutomaticLogin=$TARGET_USER|" \
            /etc/gdm3/custom.conf
        grep -qE '^AutomaticLoginEnable=true' /etc/gdm3/custom.conf \
            && ok "autologin enabled for $TARGET_USER" \
            || warn "autologin lines not found to edit — add them under [daemon] manually"
    fi
fi

# --- optional: software virtual display (only if truly headless, no plug) ----
if [ "$WANT_DUMMY" = 1 ]; then
    step "Installing software virtual-display Xorg snippet"
    sudo install -D -m 0644 "$RA_DIR/xorg-dummy.conf" /etc/X11/xorg.conf.d/10-dummy.conf \
        && ok "installed /etc/X11/xorg.conf.d/10-dummy.conf (forces dummy GPU output!)" \
        || warn "could not install dummy Xorg conf"
    warn "This OVERRIDES real GPU output — remove it if you attach a monitor/dummy plug:"
    warn "   sudo rm /etc/X11/xorg.conf.d/10-dummy.conf && sudo systemctl restart gdm3"
fi

# --- firewall (only if ufw is active) ----------------------------------------
step "Firewall ports (mosh UDP 60000-61000, NoMachine TCP 4000)"
if command -v ufw >/dev/null 2>&1 && sudo ufw status 2>/dev/null | grep -q "Status: active"; then
    sudo ufw allow 60000:61000/udp >/dev/null 2>&1 && ok "opened mosh UDP 60000-61000"
    sudo ufw allow 4000/tcp        >/dev/null 2>&1 && ok "opened NoMachine TCP 4000"
else
    ok "ufw not active — no firewall ports to open"
fi

# --- next steps --------------------------------------------------------------
cat <<EOF

== Done. To finish (drops the current desktop session — reconnect after):
   sudo systemctl restart gdm3        # applies Xorg/autologin; NOT needed for the password fix

== Use it from the LAPTOP (192.168.2.1) — one layer per job:
   Terminal  : mosh $TARGET_USER@192.168.2.69      # then: tmux new -s run   (mission survives drops)
   Vision    : Lichtblick -> ws://192.168.2.69:8765   (ros2 launch ... foxglove:=true)
               or browser -> http://192.168.2.69:8080/stream_viewer?topic=/mongla/vision/forward/image_debug
               (run once:  ros2 run web_video_server web_video_server)
   Desktop   : NoMachine -> 192.168.2.69:4000  (login $TARGET_USER)

== Emergency (a wedged desktop) — over mosh/ssh, NO reboot, mission keeps running:
   sudo systemctl restart gdm3    # or: sudo pkill -KILL -u $TARGET_USER ; then reconnect

Full guide: .claude/context/platform/remote-access.md
EOF
