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

`kutum/setup.sh` runs `fetch` for you. On a bench with no board, the same center runs beside
any vision launch, cameras or recorded video:

```bash
ros2 launch mongla_vision vision_dual.launch.py fwd_video:=/path/gate.mp4 dwn_video:=/path/bin.mp4
ros2 launch mongla_batighor batighor.launch.py
```

`vision_dual` starts both detectors paused, as a mission expects: take the helm and choose a
camera to resume one, or add `paused:=false` for both.

To measure a camera, in air or in water, with the vision stack stopped (the calibration opens
the camera itself, at 1280x720):

```bash
ros2 launch mongla_batighor calibrate.launch.py device_path:=/dev/mongla_cam_downward applies_to:=pi_downward
# then open http://mongla.local:8000/?layout=calibrate
```

## How it is built

| piece | what it does |
|---|---|
| `LICHTBLICK.json` | the one Lichtblick version we serve: release URL and SHA-256. `fetch` refuses any other bytes |
| `fetch.py` | downloads (or takes `--from` a local tarball), checks the hash, leaves the source maps out (130 of 173 MB), and writes a gzip copy of every text asset |
| `serve.py` | a static server. `/` answers with a link to the app carrying `ds=foxglove-websocket`, the bridge address *as the browser reached Kutum* (tether or Wi-Fi alike), `workspace=mongla` and the layout's name. Hashed assets are cached for good, `index.html` never. `/api/` answers the part of Lichtblick's workspace API the app reads, read only: our layouts as the organisation's, and our panels as its extensions. `/layouts/<name>.json` is the same layout as a file, for the desktop app. A browser shows Kutum's copy of a layout until its operator rearranges it; then it keeps theirs, marked with a dot, until they choose Revert in the layout's menu. A layout changed on Kutum shows on a browser's second load: the app fetches the new copy after it has opened the one it holds |
| `panels.py`, `panels/` | our own Lichtblick panels, plain JavaScript with no build step, zipped into a `.foxe` as served. The release bundle names no API, so `panels.py` turns it on in the one line that does, and serve says `WARNING` and serves no panels if a new release moved that line. `panels/helm` is the helm: name, Request/Give/Release, STOP/SURFACE/DISARM, Arm behind two ticks, any verb with its own fields (the list comes from `commands.py`), missions, the camera, the detectors' settings with each detector's model registry offered for `active_model`, and the log. `panels/calibrate` is a running calibration: the pose wanted and the one seen, blur, Max ERE against its bar, fx and the field of view, the solver's output, and the vehicle's saved calibrations with an apply button each. `panels/mongla` is the whole vehicle at a glance, as its own nodes judge it: armed, mode, depth, heading and battery; each camera's lock ladder (detection, held, follow, anchor, lost) with the identity held and its authority; what the localization is aided by; the board link's share of the line; and every `/diagnostics` row, worst first. It reads only small topics, never an image or a detection stream, and redraws at most ten times a second. `panels/common.js` holds the helpers every panel shares (`el`, `num`, `mount`, `card`, the operator name), put above each panel as it is served. `test_panels.py` runs `node --check` on each panel as served, and fails a panel that carries its own CSS or helpers |
| `theme/batighor.css` | the site's look (`docs/assets/site.css`) on the operator's screen: its palette as `--mb-*` tokens, Zed Mono for every panel and Zed Sans for Lichtblick's own chrome, the font files served by Kutum from `/theme/fonts/` because the deck has no internet at the pool. `serve.py` links it into the app page, with Lichtblick's dark scheme as the default until an operator picks another. A new panel takes the look by using the classes (`mb`, `section`, `row`, `kv`, `danger`...), with no CSS of its own; `test_panels.py` checks the tokens against the site's and that every font asked for is installed |
| `overlay.py` | what the vision stack sees, as `foxglove_msgs/ImageAnnotations` on `/mongla/vision/<cam>/annotations`, in the site's palette: the detector's candidates faint beneath, its boxes with class and score, track ids, the contours a detector traced, the lock drawn thickest in its rung's colour (detection red, held amber, follow glass, anchor violet) with the identity it holds and its authority, and the anchor's matched features on the camera's pixels while they are fresh; and on `<cam>/flow_annotations` the flow node's tracked pairs, a segment from anchor to now; and on `/mongla/vision/calibration/annotations` a calibration's board corners, the cell its next pose belongs in and the point its fit is least sure of. The browser draws them over the camera's own JPEG. The overlay subscribes to a camera's streams only while a browser shows its annotations (`Watched`, polled twice a second), so no viewer costs the vehicle nothing |
| `layouts/` | our Lichtblick layouts; `test_layouts.py` fails when one shows a topic nothing publishes, or plots a topic with no header (`std_msgs`) without `timestampMethod: receiveTime`: Lichtblick plots by header stamp otherwise, and the plot stays empty while the topic publishes. `mongla_layout` is the cockpit: both cameras with everything drawn over them, the lock rungs, live camera, mode and aiding over time beneath, and the Mongla and Helm panels beside. `vision` is vision in depth: both cameras, every rung, identity and seeing state, lock authority, the fused target pose, the anchor's decomposition and the alignment error. `localization` is navigation: the 3D scene following `mongla` in `odom_ned`, aiding and motion, position with the prop fixes, odometry against flow velocity, heading from each source, floor height against depth, and the demand. `weak_link` is the one for a slow connection. `board_health` carries the manager's health board (`/diagnostics`): every subsystem graded, with its evidence, worst first. `board_link` draws `/mongla/board_link`: the link's bytes/s against the line's capacity, the board's drops and errors, the rate of every message type and every named channel with its age. `flow` is the downward camera as a velocity sensor: the floor with the point pairs the flow node tracked (`<cam>/flow_annotations`, from `flow_tracks`), velocity, distance travelled, quality, tracks and height. `calibrate` is a calibration: the camera with those annotations, its undistorted preview, and the Calibrate panel |

`launch/batighor.launch.py` starts `foxglove_bridge` (port 8765), the server (port 8000), the
overlay and the helm, each respawned like every other process; `batighor:=true` in
`bringup.launch.py` includes it. The deck is started by hand, in a terminal.

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
- A running calibration (`calibrate.launch.py`) takes its ops (skip, undo, reset, solve,
  apply, configure) by the same rule as the settings: anyone named, never during a mission.
- Every request and every change of pilot goes to `/mongla/batighor/log` and
  `~/.mongla/batighor/helm.log`; the deck adds its own commands, marked `term:<user>`.

**What a browser may do at all** is set in `batighor.launch.py` (`BRIDGE_LIMITS`), because
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
then deleted. Each is a bead under it. The guided calibration page is one of them: it is
`calibrate.launch.py`, the `calibrate` layout and panel, and the helm's `calibrate` op. The mission web console is the first deleted: its camera
switch and detector settings are the helm's, its video is the browser's, and its per-detection
table (with `vis_range`, when `depth_estimation` runs) is any Raw Messages panel on
`/mongla/vision/<cam>/detections` or `vis_range`.

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
