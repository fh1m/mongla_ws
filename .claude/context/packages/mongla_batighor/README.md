# mongla_batighor — the lighthouse

**[`src/mongla_batighor`](../../../../src/mongla_batighor)**

Batighor (বাতিঘর) is a lighthouse: it watches the water and guides the vessel home. This
package is the operator center. Today it does one thing well: any browser that opens
`http://mongla.local:8000/` gets Lichtblick, the open-source fork of Foxglove Studio,
already connected to the vehicle's `foxglove_bridge` and opened on our layout. Nothing is
installed on a laptop, and a second operator costs Kutum only the bytes, because the app
renders in the browser.

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
| `layouts/` | our Lichtblick layouts; `test_layouts.py` fails when one plots a topic nothing publishes |

`batighor:=true` in `bringup.launch.py` starts both `foxglove_bridge` (port 8765) and the
server (port 8000), each respawned like every other process.

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
| `pin.py` | The Lichtblick build Batighor serves: one pinned version, one hash. |
| `serve.py` | Serve Lichtblick and our layouts to every operator's browser. |
<!-- modules:end -->
