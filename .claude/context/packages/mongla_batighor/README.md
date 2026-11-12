# mongla_batighor — the lighthouse

**[`src/mongla_batighor`](../../../../src/mongla_batighor)**

Batighor (বাতিঘর) is a lighthouse: it watches the water and guides the vessel home. This
package is the operator center. Today it does two things: any browser that opens
`http://mongla.local:8000/` gets Lichtblick, the open-source fork of Foxglove Studio,
already connected to the vehicle's `foxglove_bridge` and opened on our layout. Nothing is
installed on a laptop, and a second operator costs Kutum only the bytes, because the app
renders in the browser; and it shows the cameras as they were sent, with what the vehicle
sees drawn over them.

```bash
ros2 run mongla_batighor fetch      # once per machine: the pinned Lichtblick build, hash-checked
ros2 launch mongla_manager bringup.launch.py vision:=true batighor:=true
# then open http://mongla.local:8000/            (our main layout)
#           http://mongla.local:8000/?layout=board_health
```

`kutum/setup.sh` runs `fetch` for you.

## How it is built

| piece | what it does |
|---|---|
| `LICHTBLICK.json` | the one Lichtblick version we serve: release URL and SHA-256. `fetch` refuses any other bytes |
| `fetch.py` | downloads (or takes `--from` a local tarball), checks the hash, leaves the source maps out (130 of 173 MB), and writes a gzip copy of every text asset |
| `serve.py` | a static server. `/` answers with a link to the app carrying `ds=foxglove-websocket`, the bridge address *as the browser reached Kutum* (tether or Wi-Fi alike), `workspace=mongla` and the layout's name. Hashed assets are cached for good, `index.html` never. `/api/` answers the part of Lichtblick's workspace API the app reads, read only: our layouts as the organisation's, and our panels as its extensions. `/layouts/<name>.json` is the same layout as a file, for the desktop app. A browser shows Kutum's copy of a layout until its operator rearranges it; then it keeps theirs, marked with a dot, until they choose Revert in the layout's menu |
| `panels.py`, `panels/` | our own Lichtblick panels, plain JavaScript with no build step, zipped into a `.foxe` as served. The release bundle names no API, so `panels.py` turns it on in the one line that does, and serve says `WARNING` and serves no panels if a new release moved that line. `panels/helm` is the helm: name, Request/Give/Release, STOP/SURFACE/DISARM, Arm behind two ticks, any verb with its own fields (the list comes from `commands.py`), missions, and the log |
| `overlay.py` | what the vision stack sees, as `foxglove_msgs/ImageAnnotations` on `/mongla/vision/<cam>/annotations`: detector boxes with class and score, track ids, and the lock drawn thicker. The browser draws them over the camera's own JPEG |
| `layouts/` | our Lichtblick layouts; `test_layouts.py` fails when one shows a topic nothing publishes. `weak_link` is the one for a slow connection |

`batighor:=true` in `bringup.launch.py` starts `foxglove_bridge` (port 8765), the server
(port 8000), the overlay and the helm, each respawned like every other process. The deck is
started by hand, in a terminal.

## Who drives: one pilot, everyone watches

Decided with the owner (bead `mongla_ws-102.25.3`). Each job gets the surface that suits it:

| surface | for | how |
|---|---|---|
| **GUI**, Lichtblick | what has to be seen: video and overlays, plots, the 3D scene, the planner | browsers drive through the helm, and only its pilot may |
| **TUI**, `ros2 run mongla_batighor deck` | the person at the pool, over mosh: the vehicle, the pilot, missions and the command log live; `s` stop, `u` surface, `d` disarm, `:` any verb | a terminal is trusted and sends to `/mongla/move` itself |
| **CLI**, `ros2 run mongla_planner mongla <verb>` | one verb, scripted | unchanged, and trusted |

**The helm** (`helm.py`, rules in `seat.py`, the browser half in `panels/helm`) is the only
way a browser drives. It serves
`/mongla/batighor/helm` (`mongla_interfaces/srv/Helm`), because `foxglove_bridge` relays service
calls but not actions and cannot tell browsers apart, so each operator types a name and the
log trusts it.

- One pilot. `disarm`, `stop` and `surface`, and aborting a mission, are open to every operator.
- To take over, ask: the pilot presses Give, or says nothing for 10 s and it passes.
- A pilot whose browser is silent for 3 s loses the helm, and a verb they sent by hand that is
  still running is followed by `stop`. A mission is not stopped: a dropped laptop must not end a
  run the vehicle flies by itself.
- `arm` needs two ticks from the pilot: propellers clear, kill switch manned.
- A verb is checked against `commands.py` (the verb exists, every field is its own and of the
  right type) before it is sent.
- The detectors' live settings (`conf`, `nms_floor`, `nms_iou`, `active_model`, `classes`, the
  table in `seat.py`) may be set by anyone named, never while a mission runs: a run is not
  retuned under the pilot. The helm reads each value once a second and shows it.
- The camera is the pilot's: choosing one publishes `/mongla/vision/active_camera`, pauses the
  other detector and resumes the chosen one, the way the DSL's `use_camera` does.
- Every request and every change of pilot goes to `/mongla/batighor/log` and
  `~/.mongla/batighor/helm.log`; the deck adds its own commands, marked `term:<user>`.

**What a browser may do at all** is set in `bringup.launch.py` (`BRIDGE_LIMITS`), because
the bridge cannot tell one browser from another and by default lets each call any service,
set any parameter and publish anything. With hidden topics on, that reached the action's
own `send_goal` and the manager's `shield_max_depth_m`, past the helm. Now a browser calls one
service, the helm; sets no parameter itself; and publishes nothing. The bridge cannot know when
a mission runs, so the detectors' live settings are changed through the helm, which can.
Checked on Kutum against the real bridge: only the helm offered, every parameter refused.
`test_the_bridge_only_lets_a_browser_drive_through_the_helm.py` holds it.
  `/mongla/batighor/helm_state` (latched) says who is pilot, who is asking, which mission runs,
  and the missions there are.

Seen on Kutum: from Chrome through the bridge, `ana` took the helm, `raf` was refused
`move_forward` with the reason, and `ana`, sending no heartbeat, lost it after 3 s; the deck in
tmux showed the same three lines and refused its own `stop` with "the manager is not answering
/mongla/move", since no manager ran. The browser panel that carries the heartbeat comes with our
Lichtblick build: the release cannot load server panels (its `apiUrl` is unset).

## Video, by link

Kutum never encodes a frame for an operator. `camera_node` republishes the JPEG the camera
sent, unchanged, on `<cam>/stream/compressed` (the camera's rate, capped at 30) and
`<cam>/preview/compressed` (5 a second), and builds neither while nobody watches; the
overlay is drawn by the browser. On Kutum's USB camera at 640x360 a frame is 41 KB and costs
0.062 ms to publish (`measured-bars` 84). Open `/` on the tether and `/?layout=weak_link` on
anything slower.

## Decided, and what comes next

The owner's decisions (2026-11-11) are recorded on bead `mongla_ws-102.25`: Lichtblick plus
our own panels; video switchable by link (the camera's JPEG passed through on the tether, a
lighter stream on a weak link); one pilot token with everyone watching, and disarm, stop and
surface from any operator; a mission planner with the pool map and the mission text side by
side; a three.js scene with the hull CAD; the Mongla look; and the five older tools ported and
then deleted. Each is a bead under it.

## Every module

<!-- modules:begin -->
Generated from each module's docstring by `tools/ci/gen_reference.py`; a module
with no summary line shows as *(no summary)* until it gets one.

| module | what it is |
|---|---|
| `deck.py` | The deck: Batighor in a terminal, for the person standing at the pool. |
| `fetch.py` | Put the pinned Lichtblick web build on this machine. |
| `helm.py` | The helm: the one way a browser drives the vehicle, and the log of who did. |
| `overlay.py` | What the vision stack sees, drawn by the browser over the camera's own picture. |
| `panels.py` | Our panels, served to Lichtblick as a workspace's extensions. |
| `pin.py` | The Lichtblick build Batighor serves: one pinned version, one hash. |
| `seat.py` | Who is the pilot: Batighor's token, as rules with no ROS in them. |
| `serve.py` | Serve Lichtblick and our layouts to every operator's browser. |
<!-- modules:end -->
