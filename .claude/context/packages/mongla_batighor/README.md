# mongla_batighor — the lighthouse

**[`src/mongla_batighor`](../../../../src/mongla_batighor)**

Batighor (বাতিঘর) is a lighthouse: it watches the water and guides the vessel home. This
package is the operator center. Today it does two things: any browser that opens
`http://kutum.local:8000/` gets Lichtblick, the open-source fork of Foxglove Studio,
already connected to the vehicle's `foxglove_bridge` and opened on our layout. Nothing is
installed on a laptop, and a second operator costs Kutum only the bytes, because the app
renders in the browser; and it shows the cameras as they were sent, with what the vehicle
sees drawn over them.

```bash
ros2 run mongla_batighor fetch      # once per machine: the pinned Lichtblick build, hash-checked
ros2 launch mongla_manager bringup.launch.py vision:=true batighor:=true
# then open http://kutum.local:8000/            (our main layout)
#           http://kutum.local:8000/?layout=board_health
```

`kutum/setup.sh` runs `fetch` for you. On a bench with no board, the same center runs beside
any vision launch, cameras or recorded video:

```bash
ros2 launch mongla_vision vision_dual.launch.py fwd_video:=/path/gate.mp4 dwn_video:=/path/bin.mp4
ros2 launch mongla_batighor batighor.launch.py
```

`vision_dual` starts both detectors paused, as a mission expects: take the helm and choose a
camera to resume one, or add `paused:=false` for both.

To see every view filled with no board, run the whole stack against the test suite's fake
board with `bench_motion=True`: a bench tremor under the ZUPT's stillness bar, a healthy IMU at the board's
rate and both battery packs, so the rate, gyro and battery plots move. The tests use it at
rest, with none of that.

```bash
cd src/mongla_manager/test && python3 -c "import fake_srot_board as f, time; \
  f.FakeSrotBoard(14555, bench_motion=True).start(); time.sleep(1e9)" &
ros2 launch mongla_manager bringup.launch.py batighor:=true mav_device:=udpin:127.0.0.1:14555
```

To measure a camera, in air or in water, with the vision stack stopped (the calibration opens
the camera itself, at 1280x720):

```bash
ros2 launch mongla_batighor calibrate.launch.py device_path:=/dev/mongla_cam_downward applies_to:=pi_downward
# then open http://kutum.local:8000/?layout=calibrate
```

## How it is built

| piece | what it does |
|---|---|
| `LICHTBLICK.json` | the one Lichtblick version we serve: release URL and SHA-256. `fetch` refuses any other bytes |
| `fetch.py` | downloads (or takes `--from` a local tarball), checks the hash, leaves the source maps out (130 of 173 MB), and writes a gzip copy of every text asset |
| `serve.py` | a static server. `/` answers with a link to the app carrying `ds=foxglove-websocket`, the bridge address *as the browser reached Kutum* (tether or Wi-Fi alike), `workspace=mongla` and the layout's name. Hashed assets are cached for good, `index.html` never. `/api/` answers the part of Lichtblick's workspace API the app reads, read only: our layouts as the organisation's, and our panels as its extensions. `/layouts/<name>.json` is the same layout as a file, for the desktop app. `/model/` serves the hull for the 3D scene from memory (`model.py`): a name is a key or it is not found. Lichtblick refuses a root path as a 3D layer's URL, so a layout keeps `/model/mongla.urdf` and Kutum writes in the host the browser used; it folds that host into the layout's date, because Lichtblick replaces its copy only when the date differs, and a deck that reaches Kutum by another name would otherwise keep URLs on the old one A browser shows Kutum's copy of a layout until its operator rearranges it; then it keeps theirs, marked with a dot, until they choose Revert in the layout's menu. A layout changed on Kutum shows on a browser's second load: the app fetches the new copy after it has opened the one it holds |
| `panels.py`, `panels/` | our own Lichtblick panels, plain JavaScript with no build step, zipped into a `.foxe` as served. The release bundle names no API, so `panels.py` turns it on in the one line that does, and serve says `WARNING` and serves no panels if a new release moved that line. `panels/control` is the **Control Center** (the helm's browser half): STOP/SURFACE/DISARM first and pinned to the top so no window size scrolls them away, always one row of three keys sized by the panel's width, each saying under its name what it does; then name, Request/Give/Release, Arm behind two ticks, any verb with its own fields (the list comes from `commands.py`), missions, and the log. The same code registers **Detectors**: the live camera and the detectors' settings with each detector's model registry offered for `active_model` (with the rest they needed 1042 px, and the Cockpit showed a third of it). `panels/instruments` is Mongla's own gauges, one Lichtblick panel each from one table: **Vehicle** (armed, mode, a compass, a depth gauge and the numbers, all growing with the panel, battery, the mission running by its beat, pilot, the flare order the board latched or `none latched`), **Vision** (each camera's lock ladder, the identity held, its authority as a bar, the model its detector runs and its epoch, what it is seeing, and every perception stage lit when it runs and struck through when off, carrying the bringup switch that turns it on; the two cameras side by side once the panel is 440 px wide), **Navigation** (a north-up plan view of the filter's last two minutes of track with the prop fixes, aiding, motion, speed), **Thrust** (the demand sent to the board as centred bars, one bar per ESC the board reports, never a zero for an ESC it has not announced), **Link** (the line's use against capacity, drops, errors, and tables of the frames per second by message and the board's named channels), **Health** (every `/diagnostics` row but Kutum's, worst first) and **Kutum** (Kutum's own rows), **Map** (the pool from the course file, north up: outline, grid and scale, floor soundings, props solid when measured and dashed amber when a rulebook nominal, unplaced ones listed and never put at the origin, the `go_to` target at its tolerance, and Mongla always on it: solid red with its sigma once the filter is anchored in the pool, hollow amber labelled "by its start" before that, placed by where it started on the course origin facing bearing 0, with an **At the start now** button to retake that), **Localization map** (the same pool read as the filter's: each prop fix with a line from where the filter was to where the fix put it, the filter's and the flow's velocity as two-second arrows, sigma, motion and input ages) and **Mission** (the mission by its beat, the task with its deadline and run budget counting down, the verb the manager runs with the fields it was given, and how each recent verb and task ended, refusals included). A control an instrument needs is built once above what it redraws, so a redraw never eats a click. A narrow panel wraps a long value instead of clipping it, and below 280 px the big numbers shrink. Each names only small topics, never an image or a detection stream, and redraws at most ten times a second. A new gauge is one entry in that table. `panels/calibrate` is a running calibration: the pose wanted and the one seen, blur, Max ERE against its bar, fx and the field of view, the solver's output, and the vehicle's saved calibrations with an apply button each. `panels/common.js` holds the helpers every panel shares (`el`, `num`, `mount`, `card`, the operator name), put above each panel as it is served. `test_panels.py` runs `node --check` on each panel as served, fails a panel that carries its own CSS or helpers, and fails an instrument that names a topic nothing publishes or a heavy one |
| `theme/batighor.css` | the site's look (`docs/assets/site.css`) on the operator's screen: its palette as `--mb-*` tokens, Zed Mono for every panel and Zed Sans for Lichtblick's own chrome, the font files served by Kutum from `/theme/fonts/` because the deck has no internet at the pool. `serve.py` links it into the app page, with Lichtblick's dark scheme as the default until an operator picks another, and the site's mark, the red diamond on its line (`theme/mongla.svg`), as the tab icon and on the app menu button in place of Lichtblick's, with the tab named Batighor. A new panel takes the look by using the classes (`mb`, `section`, `row`, `kv`, `danger`...), with no CSS of its own; `test_panels.py` checks the tokens against the site's and that every font asked for is installed |
| `overlay.py` | what the vision stack sees, as `foxglove_msgs/ImageAnnotations` on `/mongla/vision/<cam>/annotations`, with nothing left off and each thing saying what it is: every detection in its class's own colour (one per class, the same on every frame) with class, score and the one track following it (boxes and tracks pair one to one, best overlap first, so two boxes on one prop never both claim both tracks); a box below the detector's floor thin and faint with its class, score and `weak`; a track with nothing under it grey with `coasting` (the tracker predicting); every layer from the frame being drawn and no other, so a track or candidate from the frame before never sits beside a box; two lines of stats at the bottom left (camera, model, size; the detector's rate and how far its boxes are behind the frame; while the detector has said nothing for a second, as when it is paused until a mission asks, the overlay marks each frame itself on the camera's `camera_info` stamp and the second line says why, from the detector's own `paused` parameter (its parameter events, or one `get_parameters` call): `detector paused` while it waits for a mission, and in red capitals `DETECTOR SILENT ... NOT paused: stopped or stalled`, or `NO DETECTOR publishes ...` when no node publishes the camera's detections. So the Vision view, which shows a picture only with its annotations, still shows it, and a detector that died is never taken for one that waits); the contours a detector traced, the lock drawn thickest in its rung's colour (detection red, held amber, follow glass, anchor violet) with the identity it holds and its authority, and the anchor's matched features on the camera's pixels while they are fresh; and on `<cam>/flow_annotations` the flow node's tracked pairs, a segment from anchor to now, with their count just above the stats lines, clear of box labels pinned to the top edge; on `<cam>/floor_annotations` what the flow node read off the floor (`FloorRead`, from `mongla_vision/flow/floor_read.py`): every grout line the tile grating puts on the picture, in pale glass, and the lane line's centreline in amber, with the tile spacing, the height it gives and the lane's angle and offset in words; and on `/mongla/vision/calibration/annotations` a calibration's board corners, the cell its next pose belongs in and the point its fit is least sure of. The browser draws them over the camera's own JPEG. The overlay subscribes to a camera's streams only while a browser shows its annotations (`Watched`, polled twice a second), so no viewer costs the vehicle nothing. Once a second it also publishes `/mongla/batighor/stages`: each perception stage of each camera (camera, detector, below floor, tracker, lock ladder, anchor (XFeat), contours, pose (PnP), overlay, and flow on the downward camera only), on while a node publishes its topic, with the bringup argument that turns it on (a test holds that bringup takes each one). Counted publishers, because the graph also lists a topic only a subscriber asked for. For the 3D scene, `/mongla/demand` becomes `foxglove_msgs/SceneUpdate` on `/mongla/batighor/scene` (`demand_scene`): one arrow from the body origin in `mongla`, its length the demand's share of full force, and its words (`demand 72%`, `demand none`, `demand unknown` when the manager publishes NaN, as it does whenever the board owns the demand in AUTO) |
| `model.py` | the hull in the 3D scene. The site's CAD (`docs/assets/cad/hull.mgla`, one copy, installed beside the layouts) becomes one binary STL per livery colour and a URDF whose link is `mongla`, the body frame the localization publishes on `/tf`, with every measured thruster as a ring in its tunnel, read from `hull_geometry.yaml` itself (installed beside the hull), not from the allocator, which stays off the flight path until it is wired. Built and gzipped once at start, 0.15 s on Kutum, 4.2 MB of mesh sent as 1.4 MB. The CAD's glTF axes go to FRD as body = (z, -x, -y); `test_model.py` checks the extents (702 x 176 x 172 mm along x, y, z), the nose at +x and that the change is a rotation. The up sense rests on Onshape's Y-up export, as the site's viewer assumes; `hull_geometry.yaml` leaves signs to be checked against the vehicle |
| `chain.py` | the chain, photons to thrust, on `/mongla/batighor/chain` twice a second, only while something subscribes to it (subscribed for good it cost 17 % of a Pi core with nobody looking): for each stage (each camera's frame, detections, tracks and lock, the demand, the board's IMU, the ESC rpm) the rate over 5 s and the age, seen minus stamp on the host wall clock, p50 and p95. The IMU's stamp is the board clock mapped to the host, so its age is latency above the fitted offset. A stage with no stamp (`esc_rpm`) has a rate and `--`, and what the board does not send at all (its loop jitter, the mixer's saturation, a camera trigger) is listed with the upstream ask that would produce it. One row in `stages()` per stage. The Chain instrument draws it. `test_chain.py` reads back a synthetic chain whose delays were chosen: 0.5 ms and 2 % on the meter, 5 ms through ROS |
| `views.py`, `layouts/` | every layout is generated from one table, `views.py` (`ros2 run mongla_batighor views --write`); `test_views.py` fails when a committed layout is not what it generates, or when one was made by hand, and fails a topic the stack publishes that no view, panel or overlay layer shows, unless `NOT_SHOWN` in `views.py` names it with the reason (a command, a raw frame, a copy shown elsewhere). A camera added to `seat.CAMERAS` reaches every per-camera view, a sensor is one series in its aspect's view, an aspect is one function. Each aspect is its own layout in Lichtblick's layout menu: `mongla_layout`, the cockpit, in three columns: both cameras with everything drawn over them, the column as wide as their 16:9 pictures; the Control Center at full height; Vehicle with the hull over Vision. `test_views.py` sizes every panel on the reference screen and fails a camera panel more than 15 % empty around the vehicle profiles' picture shape around its picture, a Control Center under 600 px, and a state strip that cannot show all its rows. `vision`: detection and tracking in depth, each camera synchronised so every frame shows with its own boxes (boxes come about two frames after the picture; the cockpit stays unsynchronised, because a synchronised picture stops when the detector does), the lock's history, the visibility range and its map when `depth_estimation` runs, each camera's rung over time and the live camera (what each holds and sees now is in the Vision instrument), lock authority, the fused target pose, the anchor's decomposition and the alignment error; the Detectors panel sits beside the ladder, so a setting is read against what the ladder does, and the lock's authority sits under the Vision panel. `flow`: the downward camera as a speed sensor, the floor with the pairs tracked, flow velocity against the filter's, distance, where its velocity scale came from, quality, tracks and height. `localization`: navigation, the Localization map, Lichtblick's 3D scene following `mongla` in `odom_ned` with the hull and its thrusters drawn on it and the demand as an arrow from its origin, the plan view, whether it is usable, why not and what aids it, sigma, position with the prop fixes, odometry against flow, heading from each source, floor height against depth. `map`: planning a run, the Map at three fifths (the pool is drawn to its own shape, so wider only added black), with the Vehicle instrument and the hull over the Mission instrument beside it. `mission`: the Map, the MissionText instrument (the running mission's file, with the line that sent the verb marked) and the decision layer, the Mission instrument over the surge and sway demand it sends. `thrust`: the chain's far end, the demand, each ESC's rpm, the pack voltage and the body rates. `board_health`: every subsystem graded, the board's and Kutum's, each row once. `board_link`: the line's use and drops plotted, every message rate and named channel in the Link instrument. `chain`: photons to thrust, both cameras beside the Chain instrument. `downward`: everything under the hull, the downward camera at full height with its boxes beside its ladder, alignment error, heights and anchor. `calibrate`, and `weak_link` for a slow connection. Every plot and every state strip follows the last minute (`PLOT_WINDOW_S`; a state strip by `xAxisRange`, without which it spans the whole connection and its last minute is a sliver), and every state strip is tall enough to show all its rows on a 1080 p screen: Lichtblick draws each row 64 px and scrolls the rest out of sight (`test_every_state_row_is_visible_without_scrolling`). `test_layouts.py` fails when one shows a topic nothing publishes, or plots a topic with no header (`std_msgs`) without `timestampMethod: receiveTime`: Lichtblick plots by header stamp otherwise, and the plot stays empty while the topic publishes |

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

**The helm** (`helm.py`, rules in `seat.py`, the browser half the Control Center in `panels/control`) is the only
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

The heartbeat belongs to the page, not to the Control Center: opening a view without it
unmounts the panel, and Lichtblick refuses a service call from an unmounted panel, so the
beat goes through whichever Batighor panel is mounted (`callAny` in `common.js`; the Status
strip is above the tabs in every view). Kept in the panel, it stopped on a tab switch, the
pilot lost the helm 3 s later and the helm stopped their verb.

## Where things go on the screen

`views.py` places every panel by these rules. A view that breaks one says why in its docstring.

| rule | why | source |
|---|---|---|
| The picture is the largest thing, on the left; numbers and controls are on the right | the eye reads the scene first, and the hand goes to the controls beside it. QGroundControl's Fly View keeps the video large and puts its instrument panel at the right centre | [QGroundControl, Fly View](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/fly_view/fly_view.html) |
| One aspect per view, chosen from the layout menu; nothing is crammed into one panel | a glance answers one question. Workload and situational awareness are the factors a mission-control display is judged on | [DLR, UX/UI for Columbus operations (SpaceOps 2025)](https://elib.dlr.de/220842/1/SpaceOps2025-Paper-Investigating%20UX-UI%20design%20for%20Columbus%20Operations_FIN1.pdf) |
| Show what moves, and quieten what does not: health worst first, absence as `--`, never a reassuring zero | mission-control displays are designed to cut the detail of what is unimportant and draw attention to dynamic data | [NASA/JPL, Mars Observer mission-control displays](https://ntrs.nasa.gov/api/citations/19940019415/downloads/19940019415.pdf) |
| STOP, SURFACE and DISARM in the same place in every view that drives, one press for anyone | an operator under load must not search for the way out | the owner's rule (bead 102.25.3) |
| Every operator gets the same panels, from one place | a team that published its Foxglove extensions so every member had the latest GUI | [Foxglove, Duke Robotics spotlight](https://foxglove.dev/blog/spotlight-duke-robotics-clubs-journey-to-underwater-autonomy) |
| Every published topic is shown somewhere, or named with the reason it is not | an operator cannot reason about what they cannot see | `test_views.py` |

## The console

`batighor`, the desktop app's default, is every view in one window: a status strip across the
top and one tab per view below it (`views.CONSOLE`, `build('batighor')`). Only the open tab's
panels mount, so twelve views cost what one does. Each tab has its own glyph (`theme/batighor.css`).

The strip is the `Status` instrument (`statusOf`): vehicle, mode, depth, heading, battery,
mission, the verb running, each camera's lock rung, navigation, health and link. Health counts
errors, warnings and unknown rows apart, so a bench with no barometer reads "8 unknown", not
"8 faults". The same function marks the tabs: a dot on a tab says that view has something live
(sea), a warning (amber) or an error (red) in it. Red means look now, and only errors and an armed
vehicle use it.

**Do not rename a tab.** Clicking the tab that is already open starts Lichtblick's rename (its
title is selected). The glyph and the dot are found by the tab's title, so a renamed tab loses
both, and the edit marks the layout as the operator's, which stops it taking Kutum's. Escape
leaves the rename unchanged; Revert in the layout's menu undoes one that was made.

**The red alert in the top bar is not ours.** "Definition of schema 'sensor_msgs/msg/Range' has
changed during the server's runtime" appears on every connection to Kutum. Lichtblick 1.27
seeds its type table with its built-in Humble messages for any ROS 2 distro but foxy and
galactic, and Humble's `Range` has no `variance` field, which Jazzy's has; the first advertise
of `floor_height` then reads as a change. Every message is still decoded with the schema the
bridge sent, so the floor height is right; only the message-path completion lacks `variance`.

## Video, by link

### The desktop app

`python3 tools/pool/batighor_desktop.py [host]` on the operator's machine installs Kutum's
panels, writes every view into `~/.lichtblick-suite/layouts/`, and starts the Lichtblick
desktop app connected, on the cockpit (`--view <name>` for another). Lichtblick 1.27 imports a
folder layout once, by file name, and a layout opened from a link carries no file name, so the
next start imported the folder copy beside it; so no link names a layout, and the app selects
the view by name (`--defaultLayout`); the default is `batighor`, the console below.

**Keeping the app in step with Kutum.** The app keeps its first copy of a layout, so the
instruments panel brings them up to date itself: at start, in the desktop app only, it asks
`http://<host>:8000/layouts/index.json` on the host it last connected to, fetches each layout,
and replaces every copy that came from one of Kutum's files and that nobody edited, then reloads
once (`syncLayouts`, the pure decision is `layoutUpdates`). "Edited" means a value the file
holds was changed; the defaults the app writes into a panel when it mounts, and the tab an
operator last chose, are not edits, and are kept under Kutum's values (the console never updated
while they counted). Where a 3D panel's camera looks is not an edit either: the panel writes it
back as it renders, so it takes Kutum's camera. A real edit is kept and named in the console log; Revert in the layout's
menu takes Kutum's. `kutum/deploy.sh` runs the desktop step after a release when the app is
installed on the dev box and Batighor answers; with the stack down it says to run
`tools/pool/batighor_desktop.py kutum --no-launch` after bringup. On a dense screen the app is started at a scale read from
the screen's pixels and millimetres (`--force-device-scale-factor`, `--scale` to set it): a 4K
15-inch panel at scale 1 drew the panels' 12 px type 1 mm tall. On Linux, where the GPU in
use has its Vulkan driver, the app draws through Vulkan (`--use-angle=vulkan`;
`--no-vulkan` to refuse): through ANGLE's default OpenGL a tab switch blocked the page for
1.5-4.9 s while every WebGL panel compiled its shaders again, and on Vulkan 70-390 ms
(measured-bars 105). A blocked page holds back a STOP press too. If the app is already
open it keeps its view, and the tool says so.

Kutum does not re-encode a camera's frame for an operator. `camera_node` republishes the
JPEG the camera sent, unchanged, on `<cam>/stream/compressed` (the camera's rate, capped at
30) and `<cam>/preview/compressed` (5 a second), and builds neither while nobody watches;
the overlay is drawn by the browser. A source with no JPEG of its own (a recorded video, a
ROS topic, the OpenCV fallback a camera takes when its mailbox fails) is encoded once per
due frame, at quality 80, only while someone watches: before that it sent nothing, and a
camera that fell back showed the operator an empty panel. On Kutum's USB camera at 640x360 a frame is 41 KB and costs
0.062 ms to publish (`measured-bars` 84). Open `/` on the tether and `/?layout=weak_link` on
anything slower.

## Decided, and what comes next

The owner's decisions (2026-11-11) are recorded on bead `mongla_ws-102.25`: Lichtblick plus
our own panels; video switchable by link (the camera's JPEG passed through on the tether, a
lighter stream on a weak link); one pilot token with everyone watching, and disarm, stop and
surface from any operator; a mission planner with the pool map and the mission text side by
side (the `mission` view); a three.js scene with the hull CAD (the demand drawn on it; camera frusta and detections wait for the camera offsets and image-to-body signs, which are unmeasured); the Mongla look; and the five older tools ported and
then deleted. Each is a bead under it. The guided calibration page is one of them: it is
`calibrate.launch.py`, the `calibrate` layout and panel, and the helm's `calibrate` op. The mission web console is the first deleted: its camera
switch and detector settings are the helm's, its video is the browser's, and its per-detection
table is any Raw Messages panel on `/mongla/vision/<cam>/detections`.

## Every module

<!-- modules:begin -->
Generated from each module's docstring by `tools/ci/gen_reference.py`; a module
with no summary line shows as *(no summary)* until it gets one.

| module | what it is |
|---|---|
| `chain.py` | The chain, photons to thrust: every stage's rate and age, measured where it is seen. |
| `course.py` | The course on Batighor's map: the deck's file, as the vehicle would read it. |
| `deck.py` | The deck: Batighor in a terminal, for the person standing at the pool. |
| `fetch.py` | Put the pinned Lichtblick web build on this machine. |
| `helm.py` | The helm: the one way a browser drives the vehicle, and the log of who did. |
| `model.py` | The hull in Lichtblick's 3D scene: the site's CAD, in the body frame, with its thrusters. |
| `overlay.py` | What the vision stack sees, drawn by the browser over the camera's own picture. |
| `panels.py` | Our panels, served to Lichtblick as a workspace's extensions. |
| `pin.py` | The Lichtblick build Batighor serves: one pinned version, one hash. |
| `seat.py` | Who is the pilot: Batighor's token, as rules with no ROS in them. |
| `serve.py` | Serve Lichtblick and our layouts to every operator's browser. |
| `views.py` | Every Batighor view, from one table: `layouts/*.json` are generated here. |
<!-- modules:end -->
