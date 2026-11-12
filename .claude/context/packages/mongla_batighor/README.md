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
| `serve.py` | a static server. `/` answers with a link to the app carrying `ds=foxglove-websocket`, the bridge address *as the browser reached Kutum* (tether or Wi-Fi alike) and `layoutUrl`. Hashed assets are cached for good, `index.html` never |
| `overlay.py` | what the vision stack sees, as `foxglove_msgs/ImageAnnotations` on `/mongla/vision/<cam>/annotations`: detector boxes with class and score, track ids, and the lock drawn thicker. The browser draws them over the camera's own JPEG |
| `layouts/` | our Lichtblick layouts; `test_layouts.py` fails when one shows a topic nothing publishes. `weak_link` is the one for a slow connection |

`batighor:=true` in `bringup.launch.py` starts `foxglove_bridge` (port 8765), the server
(port 8000) and the overlay, each respawned like every other process.

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
| `fetch.py` | Put the pinned Lichtblick web build on this machine. |
| `overlay.py` | What the vision stack sees, drawn by the browser over the camera's own picture. |
| `pin.py` | The Lichtblick build Batighor serves: one pinned version, one hash. |
| `serve.py` | Serve Lichtblick and our layouts to every operator's browser. |
<!-- modules:end -->
