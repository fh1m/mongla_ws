# Ground-station remote access — the smooth, drop-proof workflow

> **What this is:** how to drive the Jetson (`192.168.2.69`) from the ground-station
> laptop over the Fathom-X tether **without** the old xrdp pain — laggy OpenCV, typing
> delay, constant password popups, and the dropped-session black screen that used to need a
> Jetson reboot mid-run. **Off the mission path** — none of this touches control/vision code.
>
> One-time install: **`tools/setup_remote_access.sh`** (idempotent; re-run after a reflash).
> Config it installs lives in `tools/remote-access/`.

## The one idea: don't remote-desktop *everything* — split by layer

xrdp was one laggy pipe for all jobs. Each job has a better tool:

| Layer | Job | Tool | Fixes |
|---|---|---|---|
| 1 | Terminal — write/run missions, CLI, launches, debug | **mosh + tmux** | typing lag, reconnects, "relaunch everything", **mission survives a GUI crash** |
| 2 | Watch vision / OpenCV | **Lichtblick** (`foxglove:=true`) or **`web_video_server`** in a browser | laggy OpenCV over RDP — now no desktop pixels at all |
| 3 | Full desktop (QGC, the odd GUI; BlueOS calib is already a web UI) | **NoMachine (NX)** | laggy desktop, **black-screen-on-reconnect** |
| 4 | Auth | **polkit `.pkla`** | the password popups |

> **Safety rule (the anti-reboot fix):** a mission must **never** depend on the GUI session.
> Launch it inside **tmux over mosh**. If the desktop wedges, you `mosh` back, `tmux attach`,
> and the run is still going — the GUI is only a convenience window.

Why not Sunshine/Moonlight (the usual "lowest-latency" answer)? **The Orin Nano has no
NVENC/NVDEC encoder silicon**, so it would software-encode (x264) and steal CPU from vision
inference. On this board NoMachine's NX is the right desktop; the real latency win is moving
terminal + vision off the desktop entirely (layers 1–2).

---

## Layer 1 — Terminal: mosh + tmux (where you'll live)

`mosh` = SSH for interactive use over UDP: **instant local echo** (no typing lag), and it
**survives** tether unplug/replug and roaming. `tmux` = persistent sessions, so a dropped
link never kills a running mission.

**From the laptop:**
```bash
mosh dubomini@192.168.2.69          # SSH keys / password both work; instant echo
tmux new -s run                     # or: tmux attach -t run  (after a drop)
# inside tmux — pin the run folder, then launch/mission as usual:
source scripts/pool_session.sh gate_am
ros2 launch duburi_manager bringup.launch.py mode:=pool vision:=true foxglove:=true
```
Drop the tether → `mosh` reconnects itself; if the whole link died, `mosh …` again then
`tmux attach -t run` → the mission is exactly where you left it. Drop
`tools/remote-access/tmux.conf.sample` into `~/.tmux.conf` for mouse + big scrollback.

> mosh uses **UDP 60000–61000**. No firewall on the Jetson today, so nothing to open; the
> setup script opens them automatically **if** `ufw` is ever enabled.

---

## Layer 2 — Vision without a desktop

Two ways to watch detections/`image_debug` natively on the laptop — **no remote-desktop
pixels**, so no lag:

- **Lichtblick / Foxglove** (recommended, already wired): launch with `foxglove:=true`, then
  on the laptop *Open connection → Foxglove WebSocket →* `ws://192.168.2.69:8765`. Full setup,
  the FPS "Gate A" check, and the offline-replay flow: [`foxglove-and-bags.md`](foxglove-and-bags.md).
- **Browser MJPEG** (`web_video_server`, installed by the setup script): one command on the
  Jetson, then open a URL — zero app needed:
  ```bash
  ros2 run web_video_server web_video_server        # on the Jetson (in tmux)
  ```
  Laptop browser: `http://192.168.2.69:8080/stream_viewer?topic=/duburi/vision/forward/image_debug`

> **FPS Gate A still applies:** Mongla is FPS-coupled. Confirm
> `ros2 topic hz /duburi/vision/forward/detections` is unchanged with a viewer attached, and
> view **one** camera at a time over the tether. See `foxglove-and-bags.md` §1.

---

## Layer 3 — NoMachine for a full desktop

Install once (ARM 64-bit DEB for Linux) — the setup script does it if you drop the `.deb` in
`~/Downloads`, else grab it from <https://www.nomachine.com/download/download&id=114> and:
```bash
tools/setup_remote_access.sh ~/Downloads/nomachine_*_arm64.deb
```
The NX server autostarts. From the laptop's **NoMachine client** → `192.168.2.69` (port
**4000**), log in as `dubomini`. It reconnects cleanly after a drop — no black-screen reboot.

**Headless display (no monitor attached):** the GPU needs a display signal or the desktop is
black. Primary fix = a **~$5 HDMI dummy plug** (a real signal, most robust). Software
alternative if you have no plug: `tools/setup_remote_access.sh --dummy-display` installs a
virtual-display Xorg snippet (`tools/remote-access/xorg-dummy.conf`) — but it **overrides the
real GPU**, so remove it if you later attach a monitor/dummy plug.

NoMachine needs **Xorg, not Wayland** — the script ensures `WaylandEnable=false` in
`/etc/gdm3/custom.conf` (already set on this Jetson; this also removes the old xrdp black
screen). Optional `--autologin` makes a desktop session exist at boot for NX to attach to.

---

## Layer 4 — Kill the password popups (polkit)

The "Authentication required…" dialogs appear **only over a remote desktop** because a remote
session isn't a "local-active" seat, so polkit's default `ResultActive` doesn't apply and it
prompts. The fix is a scoped `.pkla` granting `dubomini` `ResultAny=yes` for the nuisance
actions (color-manager — the main offender — plus NetworkManager, PackageKit, udisks2, upower,
suspend/hibernate). Installed to `/etc/polkit-1/localauthority/50-local.d/50-duburi-nopasswd.pkla`;
takes effect for **new** sessions (no reboot).

> Ubuntu 22.04 (JetPack 6.2) is **polkit 0.105** → `.pkla`, **not** the JS
> `/etc/polkit-1/rules.d/*.rules` you'll see in 24.04 guides (that engine doesn't exist here).
> The rule is scoped to one user + these desktop actions; it deliberately does **not** grant
> passwordless halt/reboot/power-off, and touches nothing on the vehicle (arming/thrusters/
> serial go over MAVLink/pyserial, not polkit).

---

## Emergency recovery (a wedged / black desktop) — NO reboot

Over the **mosh/ssh** link (never depends on the GUI):
```bash
sudo systemctl restart gdm3        # rebuild the desktop session
# if a zombie session blocks reconnect:
sudo pkill -KILL -u dubomini       # then reconnect NoMachine
```
Because the mission runs in **tmux over mosh**, neither touches a running mission. This is the
replacement for "reboot the Jetson mid-run", which was the dangerous part of the old workflow.

---

## Reproduce on a fresh Jetson (after a reflash)

```bash
cd ~/workspaces/duburi_ws
tools/setup_remote_access.sh [~/Downloads/nomachine_*_arm64.deb] [--autologin] [--dummy-display]
sudo systemctl restart gdm3        # applies Xorg/autologin; NOT needed for the password fix
cp tools/remote-access/tmux.conf.sample ~/.tmux.conf     # optional QoL
```
Then install the **NoMachine client** + **Lichtblick** on the laptop once. Everything else is
the per-layer usage above. The Jetson's static IP (`192.168.2.69`) and the laptop side
(`192.168.2.1`) are in [`hardware-setup.md`](hardware-setup.md) §2.

## What did NOT change
Sunshine/Moonlight (no NVENC on this board), RustDesk (fully-OSS alternative to NoMachine — a
fallback if the free NoMachine licence ever bites, not installed), and `ssh -X` X11 forwarding
(documented as laggy in `docs/JETSON_SETUP.md` §5b — superseded by layers 1–2). xrdp can stay
installed as a backstop but NoMachine is the day-to-day desktop.
