#!/usr/bin/env python3

"""Generate Gazebo world files from course layout YAML.

A course says where things go; everything else - physics, lighting, fog, the
buoyancy whitelist, the pool shell - comes from one shared template. That is the
whole point: the reference implementations we started from each carry a dozen
near-identical hand-written worlds, so changing the lighting means a dozen edits
and creating a variant means copying 250 lines.

Usage:
    scripts/gen_world.py --all                  regenerate textures, props, worlds
    scripts/gen_world.py courses/foo.yaml       one course
    scripts/gen_world.py --list                 show available courses

Course schema (see courses/ for worked examples):

    name:         world name; defaults to the file stem
    description:  free text, copied into the generated file header
    pool:         "sauvc" to use spec/arena.yaml, or an inline mapping with
                  length / width / depth / wall_thickness
    scene:        fog and lighting overrides, all optional
    physics:      max_step_size / real_time_factor / real_time_update_rate
    vehicle:      model, name, pose [x, y, z], rpy [r, p, y]
    props:        list of entries:
                    model:     registered prop name (required)
                    name:      instance name; defaults to the model name
                    xy:        [x, y] in the pool plane
                    yaw:       heading in radians, default 0
                    z:         explicit z, overriding the anchor
                    z_offset:  nudge relative to the anchor
                    ball:      false to suppress the automatic golf ball on a
                               bump flare
"""

import argparse
import glob
import math
import os
import subprocess
import sys

import yaml

import prop_library as pl

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
COURSES_DIR = os.path.join(PKG, "courses")
WORLDS_DIR = os.path.join(PKG, "worlds")
TEMPLATE = os.path.join(PKG, "templates", "world.sdf.template")

SCENE_DEFAULTS = {
    "ambient": [0.28, 0.40, 0.48],
    "background": [0.08, 0.26, 0.40],
    "fog_colour": [0.14, 0.34, 0.42],
    "fog_start": 1.2,
    "fog_end": 22.0,
    "fog_density": 0.028,
    "shadows": True,
    # plane | gerstner | none -- see WATER_SURFACES below.
    #
    # DEFAULT gerstner. The animated surface is verified to render in camera
    # SENSOR frames, not just the GUI, so every course gets it and a course
    # opts out rather than opting in. Courses were previously defaulting to the
    # static plane and only task_navigation asked for waves, which is why every
    # other course looked like it had no water at all.
    "water_surface": "gerstner",
    # Marine snow, particles per second. 0 disables it entirely.
    #
    # DENSITY is what reads on camera, not count. 60/s spread over a whole
    # 25 x 16 x 1.6 m pool put roughly one particle per 10 cubic metres -- a
    # handful of specks at infinity rather than anything a camera would call
    # particulate. The emitter is now a box around the VEHICLE (see below) and
    # the rate is per that much smaller volume, which is why this number can
    # rise a long way without becoming a blizzard.
    "snow": 900.0,
}

# How the water surface at z = 0 is drawn.
#
#   plane     a translucent generated texture. Static, cheap, always available.
#   gerstner  Gazebo's own animated Gerstner-wave surface, pulled from Fuel.
#             VERIFIED 2026-08-29 to render in CAMERA SENSOR frames, not just
#             the GUI -- a controlled A/B on one scene changed 48.7 % of pixels
#             (see TROUBLESHOOTING.md). That mattered because gz-sim renders two
#             separate scenes and the world's <fog> famously reaches only one of
#             them, so "it looks right in the GUI" proves nothing about datasets.
#   none      no surface at all; the sky shows raw.
#
# The Fuel URI is referenced rather than vendored: Gazebo downloads and caches
# it on first use, so nothing of unclear provenance lands in this repo. The
# trade is that the FIRST run of a gerstner course needs network. Courses using
# it should say so.
WATER_SURFACES = ("plane", "gerstner", "none")

# openrobotics/waves -- the first-party Gazebo asset. Its shaders carry the
# Apache-2.0 UUV Simulator header; other teams' copies of this model are copies
# of exactly this, and theirs are git-lfs pointers rather than usable meshes.
GERSTNER_URI = "https://fuel.gazebosim.org/1.0/openrobotics/models/waves"

# Named lighting / fog presets courses can pick with scene.lighting: clear|competition|murky
#
# WHERE THE TURBIDITY ACTUALLY COMES FROM. The <fog> numbers below are written
# into the world for completeness, but MEASURED 2026-08-28: gz-sim 8 (Harmonic)
# does NOT apply <scene><fog> to camera-sensor renders. Dropping fog_end from
# 18 m to 3 m left the 25 m far wall pixel-for-pixel crisp. So the SDF fog block
# is inert and cannot be the perception lever it was documented as.
#
# The lever is `underwater_fx`, which post-processes image_raw -> image_fx in
# ROS. Each preset therefore carries an "fx" block, emitted next to the world as
# <course>.fx.yaml and loaded by bridge.launch.py, so `lighting: murky` produces
# imagery that is actually murky instead of only a differently-worded world file.
LIGHTING_PRESETS = {
    "clear": {
        "ambient": [0.40, 0.50, 0.55],
        "fog_start": 3.0,
        "fog_end": 40.0,
        "fog_density": 0.01,
        "fx": {
            "turbidity": 0.15,
            "backscatter": 0.30,
            "blur_sigma": 0.3,
            "noise": 0.006,
            "vignette": 0.12,
            "atten_scale": 0.6,
        },
    },
    "competition": {
        "ambient": [0.28, 0.40, 0.48],
        "fog_start": 1.2,
        "fog_end": 22.0,
        "fog_density": 0.028,
        "fx": {
            "turbidity": 0.45,
            "backscatter": 0.55,
            "blur_sigma": 0.8,
            "noise": 0.012,
            "vignette": 0.25,
            "atten_scale": 1.0,
        },
    },
    "murky": {
        "ambient": [0.18, 0.30, 0.36],
        "fog_start": 0.6,
        "fog_end": 12.0,
        "fog_density": 0.05,
        "fog_colour": [0.10, 0.26, 0.30],
        "fx": {
            "turbidity": 0.80,
            "backscatter": 0.75,
            "blur_sigma": 1.4,
            "noise": 0.020,
            "vignette": 0.35,
            "atten_scale": 1.8,
        },
    },
}

# What a course gets when it names no preset: the same numbers underwater_fx
# already defaulted to, so an un-annotated course is unchanged by this wiring.
FX_DEFAULTS = dict(LIGHTING_PRESETS["competition"]["fx"])

PHYSICS_DEFAULTS = {
    "max_step_size": 0.001,
    "real_time_factor": 1.0,
    "real_time_update_rate": 1000,
    # collision_detector is a CORRECTNESS setting, not a speed one.
    #
    # DART's built-in detector does not support several primitive pairs. With
    # `dart` selected, gz floods the log with
    #     [DARTCollisionDetector] Attempting to check for an unsupported shape
    #     pair: [CylinderShape] - [BoxShape]. Returning false.
    # and "returning false" means exactly what it says: no contact is generated.
    # The hull's collision shape is a BOX and every pipe prop -- gate legs,
    # slalom pipes, flare poles, path markers -- is a CYLINDER, so the vehicle
    # drove through all of them.
    #
    # MEASURED, A/B on robosub26_full, identical thrust into a gate leg vs into
    # open water: with `dart`, 1.739 m vs 1.715 m and a flat 0.656 m/s through
    # the leg -- no collision at all. With `bullet`, the leg stops it.
    #
    # This overturns an earlier decision recorded here. `bullet` + `dantzig`
    # measured ~5 % slower than `dart` over 443 RTF samples, and on that basis
    # `dart` was selected. That comparison was real but it measured the wrong
    # quantity: a faster simulator that does not collide is not a cheaper
    # trade-off, it is the wrong answer. The sim is render-bound anyway
    # (PHYSICS.md), so the 5 % is not where the time goes.
    #
    # Alternatives are ode|bullet|fcl|dart and dantzig|pgs. If you change this,
    # re-run the collision A/B, not just an RTF sample.
    "collision_detector": "bullet",
    "solver_type": "dantzig",
}

# Roughly Dhaka. Only matters to ArduSub's simulated compass.
MAGNETIC_FIELD = "3.9e-05 1.1e-06 2.6e-05"


def _fmt3(values) -> str:
    return " ".join(f"{v:.6g}" for v in values)


def _indent(text: str, level: int) -> str:
    pad = "  " * level
    return "\n".join(pad + line if line.strip() else line for line in text.split("\n"))


def _include(uri: str, name: str, pose: str) -> str:
    return (
        "<include>\n"
        f"  <uri>model://{uri}</uri>\n"
        f"  <name>{name}</name>\n"
        f"  <pose>{pose}</pose>\n"
        "</include>"
    )


def resolve_pool(course: dict, spec: dict) -> dict:
    """Resolve the course's `pool` field into a concrete pool config."""
    # A named preset is a COMPETITION -- every competition with a spec file
    # brings its own pool. This used to accept the single literal "sauvc".
    pool = course.get("pool", course.get("competition", "sauvc"))
    if isinstance(pool, str):
        known = pl.competitions()
        if pool not in known:
            raise ValueError(
                f"unknown pool preset {pool!r}; "
                f"use one of {', '.join(known)} or an inline mapping")
        return dict(pl.load_spec(competition=pool)["pool"])

    cfg = dict(spec["pool"])
    cfg.update(pool)

    # The qualification gate and the orange flare are generated to span the
    # arena spec's depth. If a course quietly picks a different one they will be
    # the wrong length, and the failure looks like a physics bug rather than a
    # config mistake.
    if abs(cfg["depth"] - spec["pool"]["depth"]) > 1e-9:
        print(
            f"  warning: pool depth {cfg['depth']} m differs from the arena spec "
            f"({spec['pool']['depth']} m). Depth-spanning props "
            "were generated for the spec depth and will not reach. Update "
            "spec/<competition>.yaml and re-run --all instead.",
            file=sys.stderr,
        )
    return cfg


def build_props(course: dict, spec: dict, pool_cfg: dict):
    """Return (props_xml, dynamic_model_names) for the course's prop list."""
    floor_z = -pool_cfg["depth"]
    blocks = []
    dynamic = []
    used_names = set()

    for entry in course.get("props") or []:
        model_name = entry["model"]
        if model_name not in pl.PROPS:
            raise KeyError(
                f"unknown prop '{model_name}'. Known props: "
                f"{', '.join(sorted(pl.PROPS))}"
            )
        meta = pl.PROPS[model_name]

        name = entry.get("name", model_name)
        if name in used_names:
            raise ValueError(f"duplicate prop name '{name}' in course")
        used_names.add(name)

        x, y = entry.get("xy", [0.0, 0.0])
        yaw = entry.get("yaw", 0.0)

        if "z" in entry:
            z = entry["z"]
        else:
            # Floor-anchored props sit on the floor AT THEIR OWN x, not at a
            # single pool depth. SAUVC's floor is a shallow V (1.6 m centre,
            # 1.2 m ends), so a drum in the target zone placed at a flat -1.6 m
            # would hang 0.34 m in the water with nothing under it. Flat pools
            # return the same number everywhere, so nothing else changes.
            pitch = 0.0
            if meta["anchor"] == pl.ANCHOR_FLOOR:
                z = -pl.floor_depth_at(pool_cfg, x)
                # Tilt with the floor. Anything resting on a slope is tilted by
                # it; for the flat decals that is the difference between lying
                # on the floor and having one edge buried in it.
                pitch = pl.floor_pitch_at(pool_cfg, x)
            else:
                z = 0.0
            z += entry.get("z_offset", 0.0)

        # DECOMPOSE the world tilt into the prop's OWN roll and pitch.
        #
        # SDF composes rpy as Rz(yaw)·Ry(pitch)·Rx(roll), so roll and pitch are
        # about the prop's axes AFTER yaw. Putting the whole tilt in `pitch`
        # therefore tilts it about the wrong axis as soon as yaw is non-zero --
        # the target mat is yawed 90 degrees, so its "pitch" tips it along the
        # pool's y, where the floor is flat, and leaves the across-grade
        # direction untilted. Silent, and exactly the failure the tilt was
        # added to fix.
        #
        # A small world tilt about +y decomposes into the yawed frame as
        # roll = tilt·sin(yaw), pitch = tilt·cos(yaw). Verified against
        # measured link poses in Gazebo, not just derived.
        roll = pitch * math.sin(yaw)
        prop_pitch = pitch * math.cos(yaw)
        blocks.append(_include(
            model_name, name,
            f"{x:.6g} {y:.6g} {z:.6g} "
            f"{roll:.6g} {prop_pitch:.6g} {yaw:.6g}"))
        if meta["dynamic"]:
            dynamic.append(name)

        # Bump flares carry a golf ball that the AUV has to knock off. Placing it
        # here rather than inside the flare model keeps it a separate dynamic
        # body, which is what makes knocking it off possible at all.
        ball_on = meta.get("ball_on")
        if ball_on and entry.get("ball", True):
            ball_name = f"{name}_ball"
            lift = ball_on(spec)

            # FOLLOW THE FLARE'S TILT. On SAUVC's sloped floor a flare is
            # pitched to stand normal to the floor, so the top of an 0.8 m pole
            # is displaced HORIZONTALLY by height*sin(pitch) and sits slightly
            # lower than height above the base. Adding the lift straight up put
            # the ball 21 mm above the cup and about 26 mm to one side of it --
            # so it spawned in open water beside the flare and fell, every run.
            #
            # This is why the "does z change" check passed while every ball was
            # on the floor: a ball at rest on the bottom is perfectly stable.
            # Verify a ball by its HEIGHT against the cup, never by stability.
            ball_x = x + lift * math.sin(prop_pitch) * math.cos(yaw)
            ball_y = y + lift * math.sin(prop_pitch) * math.sin(yaw) \
                + lift * math.sin(roll)
            ball_z = z + lift * math.cos(prop_pitch) * math.cos(roll)
            blocks.append(
                _include(
                    "sauvc_golf_ball", ball_name,
                    f"{ball_x:.6g} {ball_y:.6g} {ball_z:.6g} 0 0 0"
                )
            )
            dynamic.append(ball_name)

    return "\n\n".join(blocks), dynamic


def build_vehicle(course: dict, pool_cfg: dict):
    """Return (vehicle_xml, vehicle_name)."""
    vehicle = course.get("vehicle")
    if not vehicle:
        return "", None

    model_name = vehicle.get("model", "duburi_heavy")
    name = vehicle.get("name", model_name)

    pose = vehicle.get("pose")
    if pose is None:
        # Default to floating just under the surface at the pool's -x end.
        pose = [-pool_cfg["length"] / 2.0 + 1.5, 0.0, -0.3]
    rpy = vehicle.get("rpy", [0.0, 0.0, vehicle.get("yaw", 0.0)])

    pose_str = f"{_fmt3(pose)} {_fmt3(rpy)}"
    return _include(model_name, name, pose_str), name


def generate(course_path: str, spec: dict = None, outdir: str = WORLDS_DIR) -> str:
    with open(course_path) as f:
        course = yaml.safe_load(f)

    # The COURSE chooses its competition, so each world is generated against its
    # own spec. Passing one spec for every course is what would silently build a
    # RoboSub world against SAUVC's 1.6 m pool. An explicit `spec` still wins so
    # `--spec` keeps working.
    competition = course.get("competition", course.get("pool", "sauvc"))
    if not isinstance(competition, str):
        competition = "sauvc"
    if spec is None:
        spec = pl.load_spec(competition=competition)

    stem = os.path.splitext(os.path.basename(course_path))[0]
    world_name = course.get("name", stem)

    pool_cfg = resolve_pool(course, spec)

    scene = dict(SCENE_DEFAULTS)
    course_scene = dict(course.get("scene") or {})
    preset_name = course_scene.pop("lighting", None)
    if preset_name:
        if preset_name not in LIGHTING_PRESETS:
            raise ValueError(
                f"unknown lighting preset '{preset_name}'. "
                f"Known: {', '.join(sorted(LIGHTING_PRESETS))}"
            )
        scene.update(LIGHTING_PRESETS[preset_name])
    scene.update(course_scene)

    physics = dict(PHYSICS_DEFAULTS)
    physics.update(course.get("physics") or {})

    water_surface = scene.get("water_surface", "plane")
    if water_surface not in WATER_SURFACES:
        raise ValueError(
            f"unknown water_surface {water_surface!r}; "
            f"known: {', '.join(WATER_SURFACES)}")

    snow_rate = float(scene.get("snow", 900.0))
    if snow_rate > 0:
        # THIS EMITTER IS FOR THE OPERATOR'S EYES ONLY. It does NOT reach any
        # camera sensor, so it does NOT appear in image_raw, image_fx, a
        # recorded dataset, or anything the detector sees.
        #
        # MEASURED: 0.4 m particles at 4000/s, the emitter confirmed alive with
        # a subscriber on its own topic, and the frame off
        # /duburi/sim/front_camera/image_fx came back pixel-for-pixel as clean
        # as with the emitter switched off. Per-pixel stddev over 14 frames was
        # 1.4700 with snow and 1.4678 without -- indistinguishable. It is the
        # same GUI-scene / sensor-scene split that makes <scene><fog> useless.
        #
        # The particulate the VISION PIPELINE sees is composited in
        # `underwater_fx.ParticleField` instead. Raising the rate here will
        # make the GUI prettier and change no dataset whatsoever.
        #
        # DENSITY is what reads on camera, not particle count. The old emitter
        # spanned the whole 25 x 16 x 1.6 m pool at 60/s, which is roughly one
        # particle per 10 cubic metres -- specks at infinity, not particulate.
        # This one covers the central 12 x 12 m at 15x the rate. It is static:
        # gz-sim particle emitters live in a model, and the world file cannot
        # inject a link into the vehicle, so following the hull would need a
        # node teleporting a model every frame. The pool is small enough that
        # a centred box covers everywhere the vehicle actually operates.
        #
        # <color_start>/<color_end> are NOT set, because gz-sim logs
        #   "ParticleEmitter SetColorRange is currently disabled"
        # and ignores them -- that is why particles used to render as hard
        # white dots. The soft falloff is in marine_snow.png's alpha channel
        # and the tint is <diffuse>.
        marine_snow = f"""    <model name="marine_snow">
      <static>true</static>
      <pose>0 0 {-pool_cfg['depth'] / 2.0:.6g} 0 0 0</pose>
      <link name="link">
        <particle_emitter name="snow" type="box">
          <emitting>true</emitting>
          <size>12 12 {pool_cfg['depth']:.6g}</size>
          <particle_size>0.006 0.006 0.006</particle_size>
          <lifetime>30</lifetime>
          <min_velocity>0.004</min_velocity>
          <max_velocity>0.025</max_velocity>
          <scale_rate>0</scale_rate>
          <rate>{snow_rate:.6g}</rate>
          <topic>marine_snow</topic>
          <material>
            <diffuse>0.80 0.84 0.80</diffuse>
            <specular>0.05 0.05 0.05</specular>
            <pbr>
              <metal>
                <albedo_map>model://{pl.texture_model(competition)}/marine_snow.png</albedo_map>
              </metal>
            </pbr>
          </material>
        </particle_emitter>
      </link>
    </model>"""
    else:
        marine_snow = ""

    props_xml, dynamic = build_props(course, spec, pool_cfg)
    if water_surface == "gerstner":
        # Sits at z=0 like the plane it replaces. No collision (the model is
        # visual-only), so `surface()` still works -- the vehicle is not pushing
        # against a lid.
        props_xml += (
            f'\n\n<include>\n  <uri>{GERSTNER_URI}</uri>\n'
            f'  <name>water_surface</name>\n'
            f'  <pose>0 0 0 0 0 0</pose>\n</include>'
        )
    vehicle_xml, vehicle_name = build_vehicle(course, pool_cfg)

    enabled = ([vehicle_name] if vehicle_name else []) + dynamic
    buoyancy_enables = "\n".join(f"      <enable>{n}</enable>" for n in enabled)

    with open(TEMPLATE) as f:
        template = f.read()

    world = template.format(
        course_file=os.path.basename(course_path),
        description=course.get("description", "").strip() or "No description.",
        world_name=world_name,
        water_density=f"{course.get('water_density', 1000.0):.6g}",
        buoyancy_enables=buoyancy_enables,
        collision_detector=physics["collision_detector"],
        solver_type=physics["solver_type"],
        max_step_size=physics["max_step_size"],
        real_time_factor=physics["real_time_factor"],
        real_time_update_rate=physics["real_time_update_rate"],
        magnetic_field=MAGNETIC_FIELD,
        ambient=_fmt3(scene["ambient"]),
        background=_fmt3(scene["background"]),
        shadows="true" if scene["shadows"] else "false",
        fog_colour=_fmt3(scene["fog_colour"]),
        fog_start=scene["fog_start"],
        fog_end=scene["fog_end"],
        fog_density=scene["fog_density"],
        floor_z=f"{-pool_cfg['depth']:.6g}",
        marine_snow=marine_snow,
        pool=_indent(pl.pool(spec, pool_cfg, water_surface), 2),
        props=_indent(props_xml, 2),
        vehicle=_indent(vehicle_xml, 2),
    )

    os.makedirs(outdir, exist_ok=True)
    out_path = os.path.join(outdir, f"{stem}.world")
    with open(out_path, "w") as f:
        f.write(world)

    # The turbidity sidecar. See LIGHTING_PRESETS: the world's <fog> is inert in
    # gz-sim 8, so the preset's real effect is carried here and loaded by
    # bridge.launch.py as underwater_fx's parameters. Written for every course,
    # including ones that name no preset, so the file is always there to load.
    fx = dict(FX_DEFAULTS)
    fx.update(scene.get("fx") or {})
    fx_path = os.path.join(outdir, f"{stem}.fx.yaml")
    with open(fx_path, "w") as f:
        f.write(f"# GENERATED from {os.path.basename(course_path)} "
                f"(lighting: {preset_name or 'default'}). Do not edit.\n")
        f.write("/**:\n  ros__parameters:\n")
        for k in sorted(fx):
            f.write(f"    {k}: {fx[k]}\n")
    return out_path


def write_lighting_presets(outdir: str) -> str:
    """Emit every preset's fx block as one file beside the worlds.

    So a consumer that wants "the clear preset" without loading a clear course
    -- the dataset recorder capturing a murky/clear pair from ONE sim launch --
    has a source of truth instead of a second copy of the numbers.
    """
    path = os.path.join(outdir, "lighting_presets.yaml")
    with open(path, "w") as f:
        f.write("# GENERATED by gen_world.py from LIGHTING_PRESETS. Do not edit.\n")
        f.write("# underwater_fx parameters per lighting preset.\n")
        for name in sorted(LIGHTING_PRESETS):
            fx = dict(FX_DEFAULTS)
            fx.update(LIGHTING_PRESETS[name].get("fx") or {})
            f.write(f"{name}:\n")
            for key in sorted(fx):
                f.write(f"  {key}: {fx[key]}\n")
    return path


def regenerate_assets() -> None:
    """Re-run the texture and prop generators before building worlds."""
    for script in ("gen_pool_texture.py", "gen_props.py"):
        path = os.path.join(HERE, script)
        print(f"--- {script}")
        subprocess.run([sys.executable, path], check=True, stdout=subprocess.DEVNULL)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("courses", nargs="*", help="Course YAML files.")
    parser.add_argument(
        "--all",
        action="store_true",
        help="Regenerate textures, props and every course in courses/.",
    )
    parser.add_argument("--list", action="store_true", help="List available courses.")
    parser.add_argument(
        "--spec", default=None,
        help="Arena spec YAML. Default: each course picks its own from "
             "its `competition:` key.")
    parser.add_argument("--outdir", default=WORLDS_DIR, help="Output directory.")
    args = parser.parse_args()

    available = sorted(glob.glob(os.path.join(COURSES_DIR, "*.yaml")))

    if args.list:
        for path in available:
            print(os.path.splitext(os.path.basename(path))[0])
        return

    if args.all:
        regenerate_assets()
        targets = available
    elif args.courses:
        targets = args.courses
    else:
        parser.error("give one or more course files, or --all")

    spec = pl.load_spec(args.spec) if args.spec else None
    for path in targets:
        print(f"--- {os.path.basename(path)}")
        print(f"  wrote {generate(path, spec, args.outdir)}")
    print(f"  wrote {write_lighting_presets(args.outdir)}")


if __name__ == "__main__":
    main()
