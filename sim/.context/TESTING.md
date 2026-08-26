# Testing and verification

> ℹ **Absorbed 2026-08-27.** This workspace is no longer the sibling tree
> `Ros_workspaces/duburi-sim_ws`; it lives inside the `duburi_ws` repo at
> `duburi_ws/sim/` and is under version control. Paths below have been
> updated; any remaining "sibling" phrasing is historical.

Assume workspaces sourced (humble + `duburi_ws` + `duburi_ws/sim`), `GZ_IP=127.0.0.1`,
sim + stack already up unless noted.

## Automated / CLI checks

### Contract (cameras + GT)

```zsh
ros2 run duburi_sim_bridge contract_check
# expect: contract satisfied
```

### MAVLink rates

```zsh
ros2 run duburi_sim_bridge mavlink_check
```

### Smoke (arm / depth / surge)

```zsh
ros2 run duburi_sim_bringup duburi_sim smoke
# expect: smoke OK; vehicle moved in GT / state
```

### Teleop delta (lab API)

```zsh
PORT=${DUBURI_LAB_PORT:-28765}
curl -sS -X POST http://127.0.0.1:$PORT/api/vehicle/arm
X0=$(curl -sS http://127.0.0.1:$PORT/api/sim/status | python3 -c "import sys,json;print(json.load(sys.stdin)['ground_truth']['x'])")
curl -sS -X POST http://127.0.0.1:$PORT/api/vehicle/teleop \
  -H 'content-type: application/json' -d '{"fwd":1,"gain":0.7}'
sleep 2.5
curl -sS -X POST http://127.0.0.1:$PORT/api/vehicle/teleop \
  -H 'content-type: application/json' -d '{"fwd":0}'
curl -sS http://127.0.0.1:$PORT/api/sim/status | python3 -c "import sys,json;d=json.load(sys.stdin);print(d['ground_truth'], d['teleop']['connected'])"
# expect: connected true; x moved vs X0
```

### Record + zip (10 s gate)

```zsh
PORT=$(cat /tmp/duburi_lab_port.txt 2>/dev/null || echo ${DUBURI_LAB_PORT:-28765})
curl -sS -X POST http://127.0.0.1:$PORT/api/record/start \
  -H 'content-type: application/json' \
  -d '{"name":"qa_clip","cameras":["front"],"fx":true,"frames":true,"labels":true}'
sleep 10
curl -sS -X POST http://127.0.0.1:$PORT/api/record/stop | python3 -m json.tool
# expect: ok true, record_dir set, meta.json counts > 0, fps_actual present
# expect: ffprobe duration ≈ meta.duration_s (±0.3 s)
```

CLI equivalent:

```zsh
ros2 run duburi_sim_bridge record_cameras --duration 10 --fx --frames --labels \
  --cameras front --label qa_clip
```

### Props

```zsh
curl -sS http://127.0.0.1:$PORT/api/props/catalog | python3 -m json.tool | head
curl -sS -X POST http://127.0.0.1:$PORT/api/props/spawn \
  -H 'content-type: application/json' \
  -d '{"model":"sauvc_qual_gate","name":"qa_gate","x":1,"y":0,"z":-1.5,"yaw":0.2}'
curl -sS http://127.0.0.1:$PORT/api/props/instances
curl -sS -X POST http://127.0.0.1:$PORT/api/props/move \
  -H 'content-type: application/json' \
  -d '{"name":"qa_gate","x":2,"y":0,"z":-1.5}'
curl -sS -X POST http://127.0.0.1:$PORT/api/props/remove/qa_gate
```

### Start already-running

```zsh
curl -sS -o /tmp/s.json -w '%{http_code}\n' -X POST http://127.0.0.1:$PORT/api/sim/start \
  -H 'content-type: application/json' \
  -d '{"course":"sauvc26_qualification","gui":true,"stack":true}'
# expect: 409 + message to use restart
```

## Lab browser QA checklist

- [ ] Operate: front cam has pixels (not endless black); link dots gz/sitl/mav/cams
- [ ] Preview stays relatively smooth (raw feed); FX checkbox for record
- [ ] D-pad arrows; arm center; turbidity slider to >1 shows thicker haze
- [ ] Record ≥10s → stop → toast + zip; Datasets shows duration + fps; ffprobe ≈ wall
- [ ] World: course default `sauvc26_qualification`; spawn with yaw; instances; move; remove
- [ ] Custom asset zip upload refreshes catalog (may need sim restart for first gz use)
- [ ] Datasets: newest first; zip link works; GT columns when meta has gt_*
- [ ] PlotJuggler hint visible on World tab

## World restart (long)

```zsh
# Via UI or:
curl -sS -X POST http://127.0.0.1:$PORT/api/sim/restart \
  -H 'content-type: application/json' \
  -d '{"course":"pool_empty","gui":true,"stack":true}'
# Poll /api/sim/status until phase ready or error (≤ ~90s+stack)
```

## Regression suite (short)

1. `stop` → `sim` → `stack --no-vision`  
2. `contract_check` + `mavlink_check` + `smoke`  
3. Lab health + teleop delta + 5s record + props spawn/remove  
4. Optional: one course restart  

Document failures in [AUDIT.md](AUDIT.md) / issues with logs.
