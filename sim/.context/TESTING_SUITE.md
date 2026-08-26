# Testing suite amenities (roadmap)

Beyond the current lab + PlotJuggler + Foxglove trio.

| Amenity | Why | Status |
|---------|-----|--------|
| PlotJuggler helper + layout | Live state/GT graphs | **shipped** (`duburi_sim plotjuggler`) |
| Foxglove layouts for sim | 3D + images | Use `duburi_ws` foxglove docs |
| Lab MCAP bag button | Offline PJ/Foxglove of a session | planned |
| Gate transit live score | Instant mission feedback | `gate_transit_check` exists; UI overlay planned |
| Hydro / thruster step panel | Vehicle ID from scripts | scripts in `duburi_sim_worlds` |
| Turbidity schedule presets | Domain randomization | FX API ready; presets planned |
| Multi-clip batch + train handoff | Dataset → YOLO in `duburi_ws` | docs only |
| Prop instance browser from gz | See baked-in world props | partial (lab-tracked only) |

Prioritize: MCAP button → gate score overlay → turbidity presets.
