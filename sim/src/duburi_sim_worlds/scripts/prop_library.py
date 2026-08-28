#!/usr/bin/env python3

"""SDF emitters for the SAUVC 2026 competition props and the pool itself.

This is the single source of truth for prop geometry. `gen_props.py` renders
these into standalone `model://` models, `gen_world.py` builds pool geometry
inline into each world, and `duburi_sim_scenarios` reuses the same emitters to
spawn props into a running simulator. Nothing else should describe a prop.

All dimensions come from spec/arena.yaml, which quotes the rulebook.

Anchoring convention. Every prop is authored with its origin at the point it is
mounted from, and declares which that is:

    ANCHOR_FLOOR    origin at the pool floor, geometry extends upward
    ANCHOR_SURFACE  origin at the water surface, geometry extends downward

The world generator resolves this into a z coordinate, so a course only has to
say where a prop sits in the pool plane.
"""

import math
import os

import yaml

ANCHOR_FLOOR = "floor"
ANCHOR_SURFACE = "surface"

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
DEFAULT_SPEC = os.path.join(PKG, "spec", "arena.yaml")

TEXTURE_MODEL = "sauvc_textures"


def load_spec(path: str = DEFAULT_SPEC) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


# --------------------------------------------------------------------------
# small XML helpers
# --------------------------------------------------------------------------


def _indent(text: str, level: int) -> str:
    pad = "  " * level
    return "\n".join(pad + line if line.strip() else line for line in text.split("\n"))


def _rgb(colour) -> str:
    return f"{colour[0]:.4g} {colour[1]:.4g} {colour[2]:.4g}"


def material(colour, emissive_gain: float = 0.25, specular: float = 0.2) -> str:
    """A solid-colour material.

    The emissive term is not decoration. Underwater scenes are fogged and lit
    from a single weak directional source, so without a little self-illumination
    props read as near-black to the cameras and the detector never sees them.
    """
    e = [c * emissive_gain for c in colour]
    return (
        "<material>\n"
        f"  <ambient>{_rgb(colour)} 1</ambient>\n"
        f"  <diffuse>{_rgb(colour)} 1</diffuse>\n"
        f"  <specular>{specular:.3g} {specular:.3g} {specular:.3g} 1</specular>\n"
        f"  <emissive>{_rgb(e)} 1</emissive>\n"
        "</material>"
    )


def pvc_material(colour, emissive_gain: float = 0.28) -> str:
    """Slightly specular PVC look for competition flares."""
    return material(colour, emissive_gain=emissive_gain, specular=0.55)


def textured_material(
    texture: str,
    tint: float = 0.55,
    specular: float = 0.08,
    roughness: float = 0.85,
    emissive: float = 0.0,
    roughness_map: str = "",
) -> str:
    """A PBR material driven by a texture in the sauvc_textures model.

    No emissive map on large surfaces. An emissive map in Gazebo adds the image
    at full strength on top of the lit result, which on a pool floor doubles the
    brightness and clips colour. Small props can take a flat emissive lift.
    """
    uri = f"model://{TEXTURE_MODEL}/{texture}"
    e = f"{emissive:.3g} {emissive:.3g} {emissive:.3g} 1"
    return (
        "<material>\n"
        f"  <ambient>{tint:.3g} {tint:.3g} {tint:.3g} 1</ambient>\n"
        f"  <diffuse>{tint:.3g} {tint:.3g} {tint:.3g} 1</diffuse>\n"
        f"  <specular>{specular:.3g} {specular:.3g} {specular:.3g} 1</specular>\n"
        f"  <emissive>{e}</emissive>\n"
        "  <pbr>\n"
        "    <metal>\n"
        f"      <albedo_map>{uri}</albedo_map>\n"
        "      <metalness>0.0</metalness>\n"
        + (
            f"      <roughness_map>model://{TEXTURE_MODEL}/{roughness_map}</roughness_map>\n"
            if roughness_map
            else f"      <roughness>{roughness:.3g}</roughness>\n"
        )
        +
        "    </metal>\n"
        "  </pbr>\n"
        "</material>"
    )


def stripe_material(texture: str) -> str:
    """Gate stripe albedo with enough self-light to read through fog.

    The emissive lift is NOT decoration -- see material() above. Without it the
    post reads near-black at fog range and the detector never sees the gate.
    """
    return textured_material(
        texture, tint=0.75, specular=0.25, emissive=0.12,
        roughness_map="rough_pvc.png",
    )


def fabric_material(texture: str, emissive: float = 0.14) -> str:
    """Inflated-fabric flares: matte, no specular hotspot."""
    return textured_material(
        texture, tint=0.80, specular=0.06, emissive=emissive,
        roughness_map="rough_fabric.png",
    )


def plastic_material(texture: str, emissive: float = 0.10) -> str:
    """Moulded plastic drum walls."""
    return textured_material(
        texture, tint=0.72, specular=0.14, emissive=emissive,
        roughness_map="rough_plastic.png",
    )


def box_inertia(mass: float, sx: float, sy: float, sz: float):
    return (
        mass / 12.0 * (sy * sy + sz * sz),
        mass / 12.0 * (sx * sx + sz * sz),
        mass / 12.0 * (sx * sx + sy * sy),
    )


def cylinder_inertia(mass: float, radius: float, length: float):
    """Inertia of a z-axis cylinder."""
    ixx = mass / 12.0 * (3.0 * radius * radius + length * length)
    return (ixx, ixx, 0.5 * mass * radius * radius)


def sphere_inertia(mass: float, radius: float):
    i = 0.4 * mass * radius * radius
    return (i, i, i)


def inertial(mass: float, inertia, pose: str = "0 0 0 0 0 0") -> str:
    ixx, iyy, izz = inertia
    return (
        "<inertial>\n"
        f"  <pose>{pose}</pose>\n"
        f"  <mass>{mass:.6g}</mass>\n"
        "  <inertia>\n"
        f"    <ixx>{ixx:.6g}</ixx><ixy>0</ixy><ixz>0</ixz>\n"
        f"    <iyy>{iyy:.6g}</iyy><iyz>0</iyz>\n"
        f"    <izz>{izz:.6g}</izz>\n"
        "  </inertia>\n"
        "</inertial>"
    )


def _geometry_box(sx, sy, sz) -> str:
    return f"<geometry><box><size>{sx:.6g} {sy:.6g} {sz:.6g}</size></box></geometry>"


def _geometry_cylinder(radius, length) -> str:
    return (
        "<geometry><cylinder>"
        f"<radius>{radius:.6g}</radius><length>{length:.6g}</length>"
        "</cylinder></geometry>"
    )


def _geometry_sphere(radius) -> str:
    return f"<geometry><sphere><radius>{radius:.6g}</radius></sphere></geometry>"


def visual(name, geometry, mat, pose="0 0 0 0 0 0", cast_shadows=True,
           transparency: float = 0.0) -> str:
    shadows = "" if cast_shadows else "  <cast_shadows>false</cast_shadows>\n"
    trans = f"  <transparency>{transparency:.3g}</transparency>\n" if transparency else ""
    return (
        f'<visual name="{name}">\n'
        f"  <pose>{pose}</pose>\n"
        f"{shadows}{trans}"
        f"  {geometry}\n"
        f"{_indent(mat, 1)}\n"
        "</visual>"
    )


def collision(name, geometry, pose="0 0 0 0 0 0") -> str:
    return (
        f'<collision name="{name}">\n'
        f"  <pose>{pose}</pose>\n"
        f"  {geometry}\n"
        "</collision>"
    )


def link(name, body: str, pose="0 0 0 0 0 0") -> str:
    return f'<link name="{name}">\n  <pose>{pose}</pose>\n{_indent(body, 1)}\n</link>'


def model(name, body: str, static: bool = True) -> str:
    return (
        f'<model name="{name}">\n'
        f"  <static>{'true' if static else 'false'}</static>\n"
        f"{_indent(body, 1)}\n"
        "</model>"
    )


def _solid(name, geometry, mat, mass, inertia, pose="0 0 0 0 0 0", collide=True):
    """A link with matching visual and collision geometry."""
    parts = [inertial(mass, inertia), visual(f"{name}_visual", geometry, mat)]
    if collide:
        parts.append(collision(f"{name}_collision", geometry))
    return link(name, "\n".join(parts), pose)


def _cylinder_link(
    name, radius, length, colour, mass, pose, collide=True, mat=None
):
    return _solid(
        name,
        _geometry_cylinder(radius, length),
        mat if mat is not None else material(colour),
        mass,
        cylinder_inertia(mass, radius, length),
        pose,
        collide,
    )


def _box_link(name, sx, sy, sz, colour, mass, pose, collide=True, mat=None):
    return _solid(
        name,
        _geometry_box(sx, sy, sz),
        mat if mat is not None else material(colour),
        mass,
        box_inertia(mass, sx, sy, sz),
        pose,
        collide,
    )


# --------------------------------------------------------------------------
# props
# --------------------------------------------------------------------------

WHITE = [0.92, 0.92, 0.92]


def qualification_gate(spec: dict) -> str:
    """150 cm wide, spanning pool floor to surface, orange markings both sides.

    Surface-anchored: the rulebook has it hanging from the water surface, so the
    top bar sits at z = 0 and the posts run down to the floor.
    """
    cfg = spec["props"]["qualification_gate"]
    depth = spec["pool"]["depth"]
    r = spec["pole_radius"]
    half = cfg["width"] / 2.0
    mark = min(cfg["marking_length"], depth)
    colour = cfg["marking_colour"]
    stripe = stripe_material("gate_stripe_orange.png")

    parts = [
        # Top bar at the surface, spanning y. Rolled 90 deg to lay the cylinder
        # along y instead of z.
        _cylinder_link(
            "top_bar", r, cfg["width"], WHITE, 1.0, f"0 0 0 {math.pi / 2:.6f} 0 0"
        )
    ]

    for side, sign in (("port", 1.0), ("starboard", -1.0)):
        y = sign * half
        # Orange stripe section, measured down from the surface.
        parts.append(
            _cylinder_link(
                f"post_{side}_marking",
                r,
                mark,
                colour,
                0.5,
                f"0 {y:.6g} {-mark / 2.0:.6g} 0 0 0",
                mat=stripe,
            )
        )
        # Plain section carrying on to the floor.
        rest = depth - mark
        if rest > 1e-6:
            parts.append(
                _cylinder_link(
                    f"post_{side}_lower",
                    r,
                    rest,
                    WHITE,
                    0.5,
                    f"0 {y:.6g} {-(mark + rest / 2.0):.6g} 0 0 0",
                )
            )

    return model("sauvc_qual_gate", "\n".join(parts))


def final_gate(spec: dict) -> str:
    """150 cm wide by 100 cm tall, red to port and green to starboard.

    Floor-anchored. Port is +y, which is the left-hand side for a vehicle
    travelling along +x. Stripe textures carry the rulebook red/green bands.
    """
    cfg = spec["props"]["final_gate"]
    r = spec["pole_radius"]
    half = cfg["width"] / 2.0
    height = cfg["height"]

    parts = [
        _cylinder_link(
            "top_bar",
            r,
            cfg["width"],
            WHITE,
            1.0,
            f"0 0 {height:.6g} {math.pi / 2:.6f} 0 0",
        )
    ]

    for side, sign, colour, tex in (
        ("port", 1.0, cfg["port_colour"], "gate_stripe_red.png"),
        ("starboard", -1.0, cfg["starboard_colour"], "gate_stripe_green.png"),
    ):
        y = sign * half
        parts.append(
            _cylinder_link(
                f"post_{side}",
                r,
                height,
                colour,
                1.0,
                f"0 {y:.6g} {height / 2.0:.6g} 0 0 0",
                mat=stripe_material(tex),
            )
        )

    # The gate stands in a white PVC rectangular base frame. It is in every
    # competition photo, it is most of what the BOTTOM camera sees when the
    # vehicle is over the gate, and we did not model it at all.
    bf = cfg.get("base_frame")
    if bf:
        br = bf["pipe_radius"]
        hl, hw = bf["length"] / 2.0, bf["width"] / 2.0
        for tag, pose, length in (
            ("front", f"{hl:.6g} 0 {br:.6g} {math.pi / 2:.6f} 0 0", bf["width"]),
            ("back", f"{-hl:.6g} 0 {br:.6g} {math.pi / 2:.6f} 0 0", bf["width"]),
            ("left", f"0 {hw:.6g} {br:.6g} 0 {math.pi / 2:.6f} 0", bf["length"]),
            ("right", f"0 {-hw:.6g} {br:.6g} 0 {math.pi / 2:.6f} 0", bf["length"]),
        ):
            parts.append(
                _cylinder_link(f"base_{tag}", br, length, WHITE, 1.0, pose)
            )

    return model("sauvc_final_gate", "\n".join(parts))


def orange_flare(spec: dict) -> str:
    """~15 cm diameter, spanning pool floor to surface. Floor-anchored."""
    cfg = spec["props"]["orange_flare"]
    depth = spec["pool"]["depth"]
    radius = cfg["diameter"] / 2.0
    return model(
        "sauvc_orange_flare",
        _cylinder_link(
            "flare",
            radius,
            depth,
            cfg["colour"],
            2.0,
            f"0 0 {depth / 2.0:.6g} 0 0 0",
            mat=fabric_material("flare_orange.png"),
        ),
    )


def bump_flare(spec: dict, colour_name: str) -> str:
    """80 cm tall, ~1.6 cm diameter pole. Floor-anchored.

    A golf ball is balanced on top by the world generator rather than being part
    of this model, because the whole point of the task is to knock it off.
    """
    cfg = spec["props"]["bump_flare"]
    colour = cfg["colours"][colour_name]
    radius = cfg["diameter"] / 2.0
    height = cfg["height"]
    pvc = pvc_material(colour)

    parts = [
        _cylinder_link(
            "pole",
            radius,
            height,
            colour,
            0.1,
            f"0 0 {height / 2.0:.6g} 0 0 0",
            mat=pvc,
        ),
        # A small base disc, so a pole this thin does not look like it is
        # floating and has something to stand on.
        _cylinder_link(
            "base", 0.06, 0.01, colour, 0.2, "0 0 0.005 0 0 0", mat=pvc
        ),
    ]
    return model(f"sauvc_flare_{colour_name}", "\n".join(parts))


def drum(spec: dict, colour_name: str, model_name: str = None, pinger: bool = False) -> str:
    """60 cm diameter, 30 cm deep, open-topped. Floor-anchored.

    The wall is a ring of box segments rather than a solid cylinder. That matters:
    a solid cylinder collision would make it impossible for the AUV to drop a ball
    into the drum, which is the entire Target Acquisition task.
    """
    cfg = spec["props"]["drum"]
    colour = cfg["colours"][colour_name]
    exterior = cfg["exterior_colour"]
    radius = cfg["diameter"] / 2.0
    height = cfg["depth"]
    thickness = cfg["wall_thickness"]
    n = int(cfg["segments"])
    rim = textured_material(
        "drum_rim.png", tint=0.7, specular=0.35, roughness=0.4, emissive=0.08
    )

    parts = [
        # Base disc, in the drum's colour. This is the dominant cue for the
        # downward camera looking into the drum.
        _cylinder_link(
            "base", radius, thickness, colour, 3.0,
            f"0 0 {thickness / 2.0:.6g} 0 0 0",
            # Stronger emissive lift than the default 0.25. Measured from the
            # bottom camera hovering over a drum -- i.e. the exact instant of the
            # ball drop -- the base read RGB [115,122,129], a colour spread of
            # 13 against the floor's 64. The vehicle shadows the drum it is
            # about to drop into, and a shadowed blue disc under blue-grey
            # ambient is grey. The colour of this disc is the whole cue for
            # Target Acquisition, so it has to survive its own shadow.
            mat=material(colour, emissive_gain=0.55),
        ),
        # Banded rim at the lip — reads as a painted metal edge on camera.
        _cylinder_link(
            "rim",
            radius + 0.005,
            0.025,
            exterior,
            0.3,
            f"0 0 {height - 0.012:.6g} 0 0 0",
            collide=False,
            mat=rim,
        ),
    ]

    # Chord width of one segment of the ring.
    seg_w = 2.0 * radius * math.sin(math.pi / n) * 1.05
    for i in range(n):
        angle = 2.0 * math.pi * i / n
        r_mid = radius - thickness / 2.0
        x = r_mid * math.cos(angle)
        y = r_mid * math.sin(angle)
        pose = f"{x:.6g} {y:.6g} {height / 2.0:.6g} 0 0 {angle:.6f}"
        parts.append(
            _box_link(
                f"wall_{i}", thickness, seg_w, height, exterior, 0.2, pose,
                # Moulded-plastic wall rather than flat dark grey. The drum is
                # 60 cm across and fills the bottom camera during Target
                # Acquisition, so this is the single most valuable surface to
                # texture in the whole arena.
                mat=plastic_material(f"drum_wall_{colour_name}.png"),
            )
        )
        # A thin coloured liner just inside the black wall, so the interior reads
        # as the drum's colour from any angle rather than only from directly above.
        liner_pose = (
            f"{(radius - thickness * 1.6) * math.cos(angle):.6g} "
            f"{(radius - thickness * 1.6) * math.sin(angle):.6g} "
            f"{height / 2.0:.6g} 0 0 {angle:.6f}"
        )
        parts.append(
            link(
                f"liner_{i}",
                "\n".join(
                    [
                        inertial(0.01, box_inertia(0.01, 0.004, seg_w, height)),
                        visual(
                            f"liner_{i}_visual",
                            _geometry_box(0.004, seg_w, height),
                            material(colour),
                        ),
                    ]
                ),
                liner_pose,
            )
        )

    if pinger:
        # Visual cue only (no acoustic physics yet): a yellow marker band so the
        # pinger drum is distinguishable from the other red drums on camera.
        marker = [0.95, 0.85, 0.08]
        parts.append(
            _cylinder_link(
                "pinger_band",
                radius + 0.012,
                0.04,
                marker,
                0.1,
                f"0 0 {height * 0.55:.6g} 0 0 0",
                collide=False,
                mat=material(marker, emissive_gain=0.45, specular=0.4),
            )
        )

    return model(model_name or f"sauvc_drum_{colour_name}", "\n".join(parts))


def target_mat(spec: dict) -> str:
    """The green mat the drums stand on. Floor-anchored, flat, no collision.

    Rulebook Figure 16 shows the four drums in a line on a green mat. It is the
    largest single thing the bottom camera sees while hunting the target zone,
    so it is a real detection cue and not decoration -- a drum search that finds
    the mat first has a much easier job.

    Kept as its own prop rather than being welded to the drums because the
    rulebook says it "may not be present" for 2026: a course can place the drums
    without it and the vehicle then has to find them against bare tile.
    """
    cfg = spec["props"]["target_mat"]
    # Thin, and lifted a hair off the floor so it does not z-fight the tiles.
    #
    # TRIVIAL ISOTROPIC INERTIA, not the computed plate tensor. A 6.0 x 2.2 x
    # 0.006 m box gives Ixx+Iyy = 17.017 against Izz = 17.0 -- a valid tensor
    # must satisfy the triangle inequality and this one sits on the boundary, so
    # rounding tips it over and Gazebo REFUSES THE WHOLE WORLD with "A link
    # named mat has invalid inertia". The mat is static and non-colliding, so
    # its inertia is never integrated; a token value is both honest and safe.
    # Any very thin, very elongated prop added later will hit the same wall.
    return model(
        "sauvc_target_mat",
        _solid(
            "mat",
            _geometry_box(cfg["length"], cfg["width"], 0.006),
            material(cfg["colour"], emissive_gain=0.30),
            0.001,
            (1e-6, 1e-6, 1e-6),
            "0 0 0.004 0 0 0",
            False,
        ),
    )


def starting_zone(spec: dict) -> str:
    """140 x 140 cm marking on the water surface. Surface-anchored.

    Flat and floating just above z = 0, not a raised frame. The vehicle starts
    inside this square, so anything with height here sits directly across the
    front camera's view of the course, and anything that casts a shadow paints a
    square outline on the floor right where the bottom camera is looking.
    """
    cfg = spec["props"]["starting_zone"]
    size = cfg["size"]
    b = cfg["border"]
    colour = cfg["colour"]
    half = size / 2.0
    # Thin in z, and lifted clear of the surface so a camera at z = 0 is not
    # sitting inside the geometry.
    thickness = 0.01
    z = 0.02

    parts = []
    for name, sx, sy, x, y in (
        ("north", b, size, half, 0.0),
        ("south", b, size, -half, 0.0),
        ("east", size, b, 0.0, -half),
        ("west", size, b, 0.0, half),
    ):
        parts.append(
            link(
                f"edge_{name}",
                "\n".join(
                    [
                        inertial(0.1, box_inertia(0.1, sx, sy, thickness)),
                        visual(
                            f"edge_{name}_visual",
                            _geometry_box(sx, sy, thickness),
                            material(colour, emissive_gain=0.4),
                            cast_shadows=False,
                        ),
                    ]
                ),
                f"{x:.6g} {y:.6g} {z:.6g} 0 0 0",
            )
        )
    return model("sauvc_starting_zone", "\n".join(parts))


def ball(spec: dict) -> str:
    """The 4 cm ball the AUV drops into a drum. Dynamic, and denser than water."""
    cfg = spec["props"]["ball"]
    radius = cfg["diameter"] / 2.0
    mass = cfg["mass"]
    return model(
        "sauvc_ball",
        _solid(
            "ball",
            _geometry_sphere(radius),
            # From the spec, not a literal. It was the one prop colour that
            # could not be changed without editing code.
            material(cfg.get("colour", [0.95, 0.35, 0.05])),
            mass,
            sphere_inertia(mass, radius),
        ),
        static=False,
    )


def golf_ball(spec: dict) -> str:
    """The golf ball balanced on a bump flare. Dynamic, so it can be knocked off."""
    cfg = spec["props"]["golf_ball"]
    radius = cfg["diameter"] / 2.0
    mass = cfg["mass"]
    return model(
        "sauvc_golf_ball",
        _solid(
            "ball",
            _geometry_sphere(radius),
            material(cfg["colour"]),
            mass,
            sphere_inertia(mass, radius),
        ),
        static=False,
    )


# --------------------------------------------------------------------------
# prop registry
# --------------------------------------------------------------------------
#
# `anchor`  where the model origin sits, see the module docstring
# `dynamic` whether the model needs buoyancy enabled and can be knocked around
# `ball_on` height above the model origin at which to auto-place a golf ball

PROPS = {
    "sauvc_qual_gate": {
        "build": qualification_gate,
        "anchor": ANCHOR_SURFACE,
        "dynamic": False,
    },
    "sauvc_final_gate": {
        "build": final_gate,
        "anchor": ANCHOR_FLOOR,
        "dynamic": False,
    },
    "sauvc_target_mat": {
        "build": target_mat,
        "anchor": ANCHOR_FLOOR,
        "dynamic": False,
    },
    "sauvc_orange_flare": {
        "build": orange_flare,
        "anchor": ANCHOR_FLOOR,
        "dynamic": False,
    },
    "sauvc_flare_red": {
        "build": lambda s: bump_flare(s, "red"),
        "anchor": ANCHOR_FLOOR,
        "dynamic": False,
        "ball_on": lambda s: s["props"]["bump_flare"]["height"]
        + s["props"]["golf_ball"]["diameter"] / 2.0,
    },
    "sauvc_flare_yellow": {
        "build": lambda s: bump_flare(s, "yellow"),
        "anchor": ANCHOR_FLOOR,
        "dynamic": False,
        "ball_on": lambda s: s["props"]["bump_flare"]["height"]
        + s["props"]["golf_ball"]["diameter"] / 2.0,
    },
    "sauvc_flare_blue": {
        "build": lambda s: bump_flare(s, "blue"),
        "anchor": ANCHOR_FLOOR,
        "dynamic": False,
        "ball_on": lambda s: s["props"]["bump_flare"]["height"]
        + s["props"]["golf_ball"]["diameter"] / 2.0,
    },
    "sauvc_drum_red": {
        "build": lambda s: drum(s, "red"),
        "anchor": ANCHOR_FLOOR,
        "dynamic": False,
    },
    "sauvc_drum_blue": {
        "build": lambda s: drum(s, "blue"),
        "anchor": ANCHOR_FLOOR,
        "dynamic": False,
    },
    "sauvc_drum_red_pinger": {
        # Yellow marker band distinguishes the pinger drum visually. Acoustic
        # physics is out of scope for this world package.
        "build": lambda s: drum(
            s, "red", model_name="sauvc_drum_red_pinger", pinger=True
        ),
        "anchor": ANCHOR_FLOOR,
        "dynamic": False,
    },
    "sauvc_starting_zone": {
        "build": starting_zone,
        "anchor": ANCHOR_SURFACE,
        "dynamic": False,
    },
    "sauvc_ball": {"build": ball, "anchor": ANCHOR_FLOOR, "dynamic": True},
    "sauvc_golf_ball": {"build": golf_ball, "anchor": ANCHOR_FLOOR, "dynamic": True},
}


def build(name: str, spec: dict) -> str:
    """Return the SDF `<model>` block for a registered prop."""
    if name not in PROPS:
        raise KeyError(
            f"unknown prop '{name}'. Known props: {', '.join(sorted(PROPS))}"
        )
    return PROPS[name]["build"](spec)


def standalone_sdf(name: str, spec: dict) -> str:
    """Wrap a prop in a complete SDF document, for spawning or for model.sdf."""
    return (
        '<?xml version="1.0"?>\n'
        '<sdf version="1.9">\n'
        f"{_indent(build(name, spec), 1)}\n"
        "</sdf>\n"
    )


# --------------------------------------------------------------------------
# the pool
# --------------------------------------------------------------------------


def pool(spec: dict, pool_cfg: dict = None) -> str:
    """The pool shell: a textured floor and four walls, water surface at z = 0.

    Visuals are textured planes; collisions are primitive boxes. Keeping the two
    separate is deliberate, since mesh collisions for a pool this size are far
    more expensive than the boxes that describe it exactly.
    """
    cfg = pool_cfg or spec["pool"]
    length = cfg["length"]
    width = cfg["width"]
    depth = cfg["depth"]
    t = cfg.get("wall_thickness", 0.2)

    hl, hw = length / 2.0, width / 2.0
    floor_z = -depth

    parts = []

    floor_body = [
        inertial(1000.0, (1000.0, 1000.0, 1000.0)),
        visual(
            "floor_visual",
            _geometry_box(length, width, 0.01),
            textured_material("pool_floor.png"),
            f"0 0 {0.005:.6g} 0 0 0",
        ),
        collision("floor_collision", _geometry_box(length, width, 0.1), "0 0 -0.05 0 0 0"),
    ]
    parts.append(link("floor", "\n".join(floor_body), f"0 0 {floor_z:.6g} 0 0 0"))

    # Walls. Each is a thin visual slab facing inward plus a thicker collision
    # box outside it, so the vehicle stops at the tiled surface it can see.
    wall_defs = (
        ("x_pos", hl, 0.0, "pool_wall_long.png", width, (0.01, width, depth), (t, width + 2 * t, depth)),
        ("x_neg", -hl, 0.0, "pool_wall_long.png", width, (0.01, width, depth), (t, width + 2 * t, depth)),
        ("y_pos", 0.0, hw, "pool_wall_short.png", length, (length, 0.01, depth), (length, t, depth)),
        ("y_neg", 0.0, -hw, "pool_wall_short.png", length, (length, 0.01, depth), (length, t, depth)),
    )

    for name, x, y, texture, _span, vis_size, col_size in wall_defs:
        # Nudge the collision box outward so it sits behind the visual skin.
        cx = x + (t / 2.0 + 0.005) * (1 if x > 0 else -1 if x < 0 else 0)
        cy = y + (t / 2.0 + 0.005) * (1 if y > 0 else -1 if y < 0 else 0)
        body = [
            inertial(1000.0, (1000.0, 1000.0, 1000.0)),
            visual(
                f"wall_{name}_visual",
                _geometry_box(*vis_size),
                textured_material(texture),
                f"{x:.6g} {y:.6g} {-depth / 2.0:.6g} 0 0 0",
            ),
            collision(
                f"wall_{name}_collision",
                _geometry_box(*col_size),
                f"{cx:.6g} {cy:.6g} {-depth / 2.0:.6g} 0 0 0",
            ),
        ]
        parts.append(link(f"wall_{name}", "\n".join(body)))

    # WATER SURFACE. Gazebo has no water: the pool is a floor, four walls and a
    # fog volume, so everything above z=0 was raw <sky> and the boundary read as
    # a hard edge between "pool" and "outdoors" -- which is not what a single
    # competition photo looks like. A translucent rippled plane at z=0 is what
    # turns the sky into something seen THROUGH water.
    #
    # NO COLLISION, on purpose: the vehicle surfaces through this plane, and a
    # collision here would make `surface()` push against a lid.
    #
    # cast_shadows off as well -- a 25x16 m shadow caster directly above the
    # whole arena darkens every prop and is the one thing that would undo the
    # emissive lift the props rely on to stay visible through fog.
    parts.append(
        link(
            "water_surface",
            "\n".join([
                inertial(1.0, (1.0, 1.0, 1.0)),
                visual(
                    "water_surface_visual",
                    _geometry_box(length, width, 0.01),
                    textured_material(
                        "water_surface.png", tint=0.95, specular=0.35,
                        roughness=0.15, emissive=0.22),
                    "0 0 0 0 0 0",
                    cast_shadows=False,
                    transparency=0.62,
                ),
            ]),
        )
    )

    return model("sauvc_pool", "\n".join(parts))
