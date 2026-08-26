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
}

# Named lighting / fog presets courses can pick with scene.lighting: clear|competition|murky
LIGHTING_PRESETS = {
    "clear": {
        "ambient": [0.40, 0.50, 0.55],
        "fog_start": 3.0,
        "fog_end": 40.0,
        "fog_density": 0.01,
    },
    "competition": {
        "ambient": [0.28, 0.40, 0.48],
        "fog_start": 1.2,
        "fog_end": 22.0,
        "fog_density": 0.028,
    },
    "murky": {
        "ambient": [0.18, 0.30, 0.36],
        "fog_start": 0.6,
        "fog_end": 12.0,
        "fog_density": 0.05,
        "fog_colour": [0.10, 0.26, 0.30],
    },
}

PHYSICS_DEFAULTS = {
    "max_step_size": 0.001,
    "real_time_factor": 1.0,
    "real_time_update_rate": 1000,
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
    pool = course.get("pool", "sauvc")
    if isinstance(pool, str):
        if pool != "sauvc":
            raise ValueError(
                f"unknown pool preset '{pool}'; use 'sauvc' or an inline mapping"
            )
        return dict(spec["pool"])

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
            "(sauvc_qual_gate, sauvc_orange_flare) were generated for the spec "
            "depth and will not reach. Update spec/arena.yaml and re-run "
            "--all instead.",
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
            z = floor_z if meta["anchor"] == pl.ANCHOR_FLOOR else 0.0
            z += entry.get("z_offset", 0.0)

        blocks.append(_include(model_name, name, f"{x:.6g} {y:.6g} {z:.6g} 0 0 {yaw:.6g}"))
        if meta["dynamic"]:
            dynamic.append(name)

        # Bump flares carry a golf ball that the AUV has to knock off. Placing it
        # here rather than inside the flare model keeps it a separate dynamic
        # body, which is what makes knocking it off possible at all.
        ball_on = meta.get("ball_on")
        if ball_on and entry.get("ball", True):
            ball_name = f"{name}_ball"
            ball_z = z + ball_on(spec)
            blocks.append(
                _include(
                    "sauvc_golf_ball", ball_name, f"{x:.6g} {y:.6g} {ball_z:.6g} 0 0 0"
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


def generate(course_path: str, spec: dict, outdir: str = WORLDS_DIR) -> str:
    with open(course_path) as f:
        course = yaml.safe_load(f)

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

    props_xml, dynamic = build_props(course, spec, pool_cfg)
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
        pool=_indent(pl.pool(spec, pool_cfg), 2),
        props=_indent(props_xml, 2),
        vehicle=_indent(vehicle_xml, 2),
    )

    os.makedirs(outdir, exist_ok=True)
    out_path = os.path.join(outdir, f"{stem}.world")
    with open(out_path, "w") as f:
        f.write(world)
    return out_path


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
    parser.add_argument("--spec", default=pl.DEFAULT_SPEC, help="Arena spec YAML.")
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

    spec = pl.load_spec(args.spec)
    for path in targets:
        print(f"--- {os.path.basename(path)}")
        print(f"  wrote {generate(path, spec, args.outdir)}")


if __name__ == "__main__":
    main()
