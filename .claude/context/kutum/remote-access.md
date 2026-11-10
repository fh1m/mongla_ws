# Ground-station remote access — drive the Pi without a remote desktop

> **What this is:** how to drive the vehicle's Pi from the ground-station laptop over the
> tether. **Off the mission path**: nothing here touches control or vision code.
> `sudo kutum/setup.sh --system` installs both tools below ([Kutum](README.md)).

## The one idea: a mission never depends on a GUI

A remote desktop is one laggy pipe for every job, and when it wedges it takes whatever runs
inside it down too. So nothing runs inside one. Two jobs, two tools:

| Job | Tool | Why |
|---|---|---|
| Terminal: launches, missions, the CLI, debugging | **mosh + tmux** | instant echo; survives a tether unplug; **the run outlives the link** |
| Watching vision | **Foxglove** (`foxglove:=true`) or **`web_video_server`** in a browser | no desktop pixels at all |

---

## The terminal: mosh + tmux

`mosh` is SSH for interactive use over UDP: local echo with no typing lag, and it reconnects
by itself across an unplug or a roam. `tmux` keeps the session alive on the Pi, so a dropped
link never kills a running mission.

```bash
mosh fh1m@mongla.local              # from the laptop
tmux new -s run                     # or, after a drop: tmux attach -t run
# inside tmux -- pin the run folder, then launch as usual
source tools/pool/pool_session.sh gate_am
ros2 launch mongla_manager bringup.launch.py vision:=true foxglove:=true
```

Drop the tether and `mosh` reconnects. If the whole link died, `mosh` again and
`tmux attach -t run`: the mission is where you left it.

> mosh uses **UDP 60000–61000**. If `ufw` is ever enabled on the Pi, open that range.

---

## Vision without a desktop

- **Foxglove / Lichtblick** (recommended, already wired): launch with `foxglove:=true`, then
  on the laptop *Open connection → Foxglove WebSocket →* `ws://mongla.local:8765`. Setup, the
  FPS check and offline replay: [`foxglove-and-bags.md`](../platform/foxglove-and-bags.md).
- **Browser MJPEG** (`web_video_server`): no app needed.
  ```bash
  ros2 run web_video_server web_video_server        # on the Pi, in tmux
  ```
  Laptop browser:
  `http://mongla.local:8080/stream_viewer?topic=/mongla/vision/forward/image_debug`

> **The viewer must not cost frames.** Confirm
> `ros2 topic hz /mongla/vision/forward/detections` is unchanged with a viewer attached, and
> view **one** camera at a time over the tether. [`foxglove-and-bags.md`](../platform/foxglove-and-bags.md) §1.

A cv2 window over SSH has no display to draw on: [`pi-and-env-traps.md`](pi-and-env-traps.md) E4.
