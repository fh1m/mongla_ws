# World editing (sim environment freedom)

Customize the live Gazebo course from the lab **World** tab without leaving the browser.

## What you can do

| Action | How |
|--------|-----|
| Start / stop / restart sim | World → simulation buttons |
| Switch course | Select course → **restart / switch** (stop→start, ~90s) |
| Spawn prop | Model + name + x,y,z,yaw → **+ spawn** |
| Move prop | Select instance or type name → edit pose → **↻ move** |
| Remove prop | **− remove** |
| List lab-tracked instances | **instances** |
| Library catalog | **catalog** (registered props) |
| Upload custom model | **custom assets** → `.zip` with `model.sdf` |

## APIs

| Method | Path |
|--------|------|
| GET | `/api/props/catalog` — library + custom models on disk |
| GET | `/api/props/instances` — names spawned via lab this session |
| POST | `/api/props/spawn` `{model,name,x,y,z?,yaw?}` |
| POST | `/api/props/move` `{name,x,y,z?,yaw?}` |
| POST | `/api/props/remove/{name}` |
| POST | `/api/assets/upload` multipart zip |

CLI equivalents: `ros2 run duburi_sim_scenarios props {list,add,move,remove}`.

## Custom assets

Zip layout (either):

```text
my_buoy/
  model.sdf
  meshes/...
```

or flat:

```text
model.sdf
meshes/...
```

Files land in `src/duburi_sim_worlds/models/<id>/`. Refresh catalog after upload.
**First use** of a brand-new model may require a sim restart so Gazebo picks up
`GZ_SIM_RESOURCE_PATH`.

## Limits (v0.2)

- Course geometry (pool walls, baked-in props) still requires **course restart**, not hot-edit.
- Instance list is **lab-session tracked** (spawn/move/remove via UI/API). Props baked into the world file are not listed until you spawn extras.
- True Gazebo world hot-swap is out of scope — see `AUDIT.md` / `FUTURE_MERGE.md`.
