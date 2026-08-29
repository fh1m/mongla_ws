#!/usr/bin/env python3

"""SDF emitters for the SAUVC 2026 competition props and the pool itself.

This is the single source of truth for prop geometry. `gen_props.py` renders
these into standalone `model://` models, `gen_world.py` builds pool geometry
inline into each world, and `duburi_sim_scenarios` reuses the same emitters to
spawn props into a running simulator. Nothing else should describe a prop.

All dimensions come from spec/<competition>.yaml, which quotes the rulebook.

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
SPEC_DIR = os.path.join(PKG, "spec")
DEFAULT_COMPETITION = "sauvc"
DEFAULT_SPEC = os.path.join(SPEC_DIR, f"{DEFAULT_COMPETITION}.yaml")

def texture_model(competition: str = DEFAULT_COMPETITION) -> str:
    """Texture model name for a competition.

    Per-competition because gen_pool_texture sizes the floor and wall PNGs from
    that competition's pool. Sharing one texture model between a 25x16 m pool
    and a 20x12 m one would stretch the tile pitch, silently -- the same class
    of failure as the aspect-correct textures we already fixed once.
    """
    return f"{competition}_textures"


TEXTURE_MODEL = texture_model()


def spec_path(competition: str = DEFAULT_COMPETITION) -> str:
    """Path to a competition's arena spec.

    One file per competition rather than one file with a competition key: the
    specs are mostly rulebook quotations, and nesting them would have meant
    re-indenting every line of a file whose value is precisely those comments.
    """
    path = os.path.join(SPEC_DIR, f"{competition}.yaml")
    if not os.path.isfile(path):
        raise ValueError(
            f"unknown competition {competition!r}; "
            f"known: {', '.join(competitions())}")
    return path


def competitions() -> list:
    """Every competition with a spec file -- discovered, not listed."""
    return sorted(f[:-5] for f in os.listdir(SPEC_DIR) if f.endswith(".yaml"))


def load_spec(path: str = None, competition: str = DEFAULT_COMPETITION) -> dict:
    """Load a competition's spec.

    An explicit `path` still wins, so `--spec` and existing callers are
    unaffected; otherwise the competition selects the file.
    """
    with open(path or spec_path(competition)) as f:
        spec = yaml.safe_load(f)
    spec.setdefault("competition", competition)
    return spec


def prop_competition(name: str) -> str:
    """Which competition a prop belongs to.

    Derived from the model-name prefix rather than stored on every registry
    entry. The prefix convention already exists (`sauvc_final_gate`), so
    deriving it means a new prop cannot be registered with the tag missing or
    disagreeing with its own name -- the failure mode that let target_mat sit in
    one table and not the other.
    """
    entry = PROPS.get(name)
    if entry and "competition" in entry:
        return entry["competition"]
    head = name.split("_", 1)[0]
    if head in competitions():
        return head
    raise ValueError(
        f"cannot tell which competition {name!r} belongs to: name it "
        f"'<competition>_...' or set 'competition' in its PROPS entry")


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
    competition: str = DEFAULT_COMPETITION,
) -> str:
    """A PBR material driven by a texture in the <competition>_textures model.

    No emissive map on large surfaces. An emissive map in Gazebo adds the image
    at full strength on top of the lit result, which on a pool floor doubles the
    brightness and clips colour. Small props can take a flat emissive lift.
    """
    tex_model = texture_model(competition)
    uri = f"model://{tex_model}/{texture}"
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
            f"      <roughness_map>model://{tex_model}/{roughness_map}</roughness_map>\n"
            if roughness_map
            else f"      <roughness>{roughness:.3g}</roughness>\n"
        )
        +
        "    </metal>\n"
        "  </pbr>\n"
        "</material>"
    )


def stripe_material(texture: str, competition: str = DEFAULT_COMPETITION) -> str:
    """Gate stripe albedo with enough self-light to read through fog.

    The emissive lift is NOT decoration -- see material() above. Without it the
    post reads near-black at fog range and the detector never sees the gate.
    """
    return textured_material(
        texture, tint=0.75, specular=0.25, emissive=0.12,
        roughness_map="rough_pvc.png", competition=competition,
    )


def fabric_material(texture: str, emissive: float = 0.14, competition: str = DEFAULT_COMPETITION) -> str:
    """Inflated-fabric flares: matte, no specular hotspot."""
    return textured_material(
        texture, tint=0.80, specular=0.06, emissive=emissive,
        roughness_map="rough_fabric.png", competition=competition,
    )


def plastic_material(texture: str, emissive: float = 0.10, competition: str = DEFAULT_COMPETITION) -> str:
    """Moulded plastic drum walls."""
    return textured_material(
        texture, tint=0.72, specular=0.14, emissive=emissive,
        roughness_map="rough_plastic.png", competition=competition,
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


# ---------------------------------------------------------------------------
# Detection classes
# ---------------------------------------------------------------------------
#
# APPEND ONLY. NEVER INSERT, NEVER REORDER, NEVER DELETE.
#
# The position in this list IS the YOLO class index AND the Gazebo semantic
# label. Both are baked into every dataset ever recorded, so moving an entry
# silently relabels history: a run captured last month would decode with this
# month's meaning and nothing would error. Retire a prop by leaving its slot in
# place, not by removing the line.
#
# This replaces gt_labels.CLASSES / MODEL_TO_CLASS / PROP_HALF_EXTENTS. The
# half-extents are gone entirely -- Gazebo's boundingbox_camera measures the
# real projected box on the GPU, occlusion and truncation included, so there is
# nothing left to hand-maintain and nothing left to disagree with itself.
#
# Label 0 is reserved: gz-sim treats an unlabelled entity as 0, so using it for
# a real class would make "background" and that class indistinguishable.
DETECTION_CLASSES = [
    "_background",              # 0 -- reserved, never assigned to a prop
    "sauvc_qual_gate",
    "sauvc_final_gate",
    "sauvc_orange_flare",
    "sauvc_flare_red",
    "sauvc_flare_yellow",
    "sauvc_flare_blue",
    "sauvc_drum_red",
    "sauvc_drum_blue",
    "sauvc_drum_red_pinger",
    "sauvc_starting_zone",
    "sauvc_target_mat",
    "robosub_gate",
    "robosub_slalom",
    "robosub_bins",
    "robosub_torpedo_survey_repair",
    "robosub_torpedo_search_rescue",
    "robosub_octagon",
    "robosub_resupply_table",
    "robosub_path_marker",
    "robosub_pinger",
    "robosub_item_bolt",
    "robosub_item_plug",
    "robosub_item_pill",
    "robosub_item_bandage",
    # Appended 2026-08-29 -- the balls were never classes under the old
    # gt_labels list. New entries go HERE, at the end, always.
    "sauvc_ball",
    "sauvc_golf_ball",
]

_LABEL_OF = {n: i for i, n in enumerate(DETECTION_CLASSES)}

if len(set(DETECTION_CLASSES)) != len(DETECTION_CLASSES):
    raise RuntimeError("duplicate entry in DETECTION_CLASSES; labels must be unique")


def detection_label(model_name: str) -> int:
    """Semantic label for a prop, or 0 if it is not a detection target.

    0 rather than an exception on purpose: the pool shell, the vehicle and the
    water surface all flow through model() and none of them is a class.
    """
    return _LABEL_OF.get(model_name, 0)


def model(name, body: str, static: bool = True, label: int = None) -> str:
    """One <model>, carrying its semantic label when it is a detection target.

    The label plugin is what makes Gazebo's boundingbox_camera emit a box for
    this model at all. WITHOUT IT THE SENSOR PUBLISHES NOTHING AND SAYS NOTHING
    -- an unlabelled prop is simply absent from every frame's annotations, with
    no warning anywhere, which is the same silent-empty-labels failure this
    project has already been bitten by once.
    """
    lab = detection_label(name) if label is None else label
    plugin = "" if not lab else (
        '  <plugin filename="gz-sim-label-system" '
        'name="gz::sim::systems::Label">\n'
        f"    <label>{lab}</label>\n"
        "  </plugin>\n"
    )
    return (
        f'<model name="{name}">\n'
        f"  <static>{'true' if static else 'false'}</static>\n"
        f"{plugin}"
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
            f"0 0 {FLOOR_DECAL_Z} 0 0 0",
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
    z = FLOOR_DECAL_Z

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

# --------------------------------------------------------------------------
# RoboSub 2026 props
#
# Dimensions come from spec/robosub.yaml, which quotes the Team Handbook with
# page numbers. Model names carry the `robosub_` prefix, which is what keeps
# them in the same flat models/ directory as the SAUVC set and what
# prop_competition() reads to build each against the right pool.
#
# Thin plates get TRIVIAL ISOTROPIC inertia, like sauvc_target_mat. A physically
# correct tensor for a 1.2 x 0.15 x 0.006 m path marker violates the triangle
# inequality (Ixx + Iyy >= Izz) once floating-point rounds it, and Gazebo then
# refuses to load the ENTIRE WORLD -- not just the prop. Static visual props do
# not need a real tensor.
# --------------------------------------------------------------------------

_THIN = (1e-6, 1e-6, 1e-6)


def _role_sign(spec, name, image, pose, size=None):
    """A 12 in x 12 in printed vinyl role sign.

    The image is the ACTUAL emoji RoboNation prints, rendered by
    gen_pool_texture.make_role_image. It must go on a THIN SQUARE plate: an SDF
    box applies one material to all six faces and stretches the albedo across
    each of them exactly once, so a non-square plate distorts the glyph and a
    thick one shows it on the edges too.

    ORIENTATION CONVENTION -- read this before writing a pose. The plate's
    THICKNESS runs along its local x, so the printed faces are +x and -x and
    the sign already faces along x with pose rpy 0 0 0. Adding yaw 1.5708 turns
    it EDGE-ON, which is what every gate sign was doing: visible as a thin
    white line rather than an image.

    Every one of these signs has a rulebook-defined direction it must face,
    because the vehicle has to READ it:

      gate     faces the approaching AUV, i.e. along the run axis (x), so
               rpy 0 0 0. The handbook front view shows both signs hanging
               flat off the top bar toward the vehicle.
      bins     read from ABOVE by the downward camera, so pitched back off
               vertical rather than square-on.
      torpedo  faces the AUV like the gate: the board is what you aim at.
      octagon  the images "hang inward", so each faces the octagon centre --
               yaw = ang + pi, not ang, which points them outward.
    """
    cfg = spec["sign"]
    side = size or cfg["size"]
    return _box_link(
        name, cfg["thickness"], side, side, WHITE, 0.05, pose, collide=False,
        # Low emissive on purpose. The lift exists so props stay visible
        # through fog, but on a PRINTED face it flattens the artwork -- and the
        # artwork is the thing a detector has to classify. Enough to lift it
        # off the background, not enough to wash the colour out.
        mat=textured_material(f"role_{image}.png", tint=1.0, specular=0.06,
                              roughness=0.55, emissive=0.10,
                              competition="robosub"),
    )


def robosub_gate(spec):
    """Task 1/6 gate -- a PASS-THROUGH PVC frame, 3.048 m x 1.524 m.

    Built wrong the first time as two solid panels filling the opening, which
    is not a gate at all: the vehicle drives THROUGH this. From the handbook
    CAD it is a horizontal top bar carried on two legs, each leg banded into
    two 609.6 mm colour segments, plus a 609.6 x 50.8 mm red divider hanging
    from the centre of the bar and two 305 mm role signs.

    THE ASYMMETRY IS THE SCORED FEATURE. Front view: the right leg is RED over
    BLACK and the left leg is BLACK over RED; the back face reverses both. The
    side the vehicle picks IS its role for the rest of the run, so a gate
    mission has to read which half it is looking at -- not merely find a gap.
    """
    cfg = spec["props"]["gate"]
    w, h = cfg["width"], cfg["height"]
    r = spec["pvc_one_in"]
    red, black = cfg["colours"]["red"], cfg["colours"]["black"]
    band = cfg["band_length"]
    half = w / 2.0
    parts = []

    # Width runs along y; the vehicle passes along x. Legs hang from the bar.
    parts.append(_cylinder_link(
        "top_bar", r, w, WHITE, 5.0, f"0 0 {h:.6g} 1.5708 0 0",
        mat=pvc_material(WHITE)))

    for side, y, upper, lower in (
        ("port", -half, black, red),      # left  as seen from the front
        ("stbd", half, red, black),       # right as seen from the front
    ):
        # A plain stub between the bar and the first colour band, as the CAD
        # shows, then the two bands.
        parts.append(_cylinder_link(
            f"post_{side}", r, h, WHITE, 4.0,
            f"0 {y:.6g} {h / 2.0:.6g} 0 0 0", mat=pvc_material(WHITE)))
        for tag, colour, cz in (
            ("upper", upper, h - 0.1524 - band / 2.0),
            ("lower", lower, h - 0.1524 - band * 1.5),
        ):
            # Slightly proud of the post so the band reads as painted pipe.
            parts.append(_cylinder_link(
                f"band_{side}_{tag}", r * 1.25, band, colour, 0.4,
                f"0 {y:.6g} {cz:.6g} 0 0 0", collide=False,
                mat=material(colour, emissive_gain=0.22)))

    # The red divider: hangs from the centre of the bar, 609.6 mm down.
    parts.append(_box_link(
        "divider", cfg["divider_width"], cfg["divider_width"],
        cfg["divider_drop"], red, 0.5,
        f"0 0 {h - cfg['divider_drop'] / 2.0:.6g} 0 0 0", collide=False,
        mat=material(red, emissive_gain=0.22)))

    # Role signs hang from the bar, 152.4 mm below it, one per role.
    for role, y in (("survey_repair", -w * 0.22), ("search_rescue", w * 0.22)):
        image = spec["roles"][role]["gate_images"][0]
        parts.append(_role_sign(
            spec, f"sign_{role}", image,
            # rpy 0 0 0 -- the plate already faces along x, at the AUV.
            f"0 {y:.6g} {h - 0.1524 - spec['sign']['size'] / 2.0:.6g} 0 0 0"))

    return model("robosub_gate", "\n".join(parts))


def robosub_slalom(spec):
    """Task 2: one set of three moored pipes, WHITE / RED / WHITE.

    Three SETS make the task; this is one, so a course places three of them and
    can stagger the heights the handbook calls for ('moored at different
    heights') per instance rather than baking one arrangement in.
    """
    cfg = spec["props"]["slalom"]
    r = spec["pvc_one_in"]
    h, gap = cfg["height"], cfg["spacing"]
    parts = []
    for name, y, colour in (
        ("pipe_left", gap, cfg["colours"]["white"]),
        ("pipe_centre", 0.0, cfg["colours"]["red"]),
        ("pipe_right", -gap, cfg["colours"]["white"]),
    ):
        parts.append(_cylinder_link(
            name, r, h, colour, 1.5, f"0 {y:.6g} {h / 2.0:.6g} 0 0 0",
            mat=pvc_material(colour)))
    return model("robosub_slalom", "\n".join(parts))


def robosub_bins(spec):
    """Task 3 -- ONE prop: a PVC pipeline with four crates hanging off its sides.

    Built wrong the first time as four loose crates sitting on the floor. The
    handbook is explicit: "3D pipeline made from PVC supported off the bottom
    of the pool", and "four bins hang off the SIDES of the pipeline" -- two
    Survey & Repair, two Search & Rescue. Making it one model also matches how
    it is placed: the whole assembly goes down as a unit.

    The crates are CleverMade 25 L: 0.335 x 0.335 x 0.28 m, square in plan.
    """
    cfg = spec["props"]["bin"]
    lx, ly, lz, t = cfg["length"], cfg["width"], cfg["height"], cfg["wall"]
    colour = cfg["colour"]
    ph, span = cfg["pipeline_height"], cfg["pipeline_span"]
    r = spec["pvc_three_quarter_in"]
    parts = []

    # The pipeline: a spine with two cross members, standing on four feet.
    parts.append(_cylinder_link(
        "spine", r, span, WHITE, 2.0, f"0 0 {ph:.6g} 0 1.5708 0",
        mat=pvc_material(WHITE)))
    for i, x in ((0, -span / 3.0), (1, span / 3.0)):
        parts.append(_cylinder_link(
            f"cross_{i}", r, span * 0.72, WHITE, 1.5,
            f"{x:.6g} 0 {ph:.6g} 1.5708 0 0", mat=pvc_material(WHITE)))
        for j, y in ((0, -span * 0.36), (1, span * 0.36)):
            parts.append(_cylinder_link(
                f"leg_{i}{j}", r, ph, WHITE, 1.0,
                f"{x:.6g} {y:.6g} {ph / 2.0:.6g} 0 0 0",
                mat=pvc_material(WHITE)))

    # Four crates hanging off the cross members, two per role.
    layout = (
        ("sr_a", "survey_repair", -span / 3.0, -span * 0.36),
        ("sr_b", "survey_repair", span / 3.0, span * 0.36),
        ("rescue_a", "search_rescue", -span / 3.0, span * 0.36),
        ("rescue_b", "search_rescue", span / 3.0, -span * 0.36),
    )
    for tag, role, cx, cy in layout:
        base = ph - lz / 2.0
        parts.append(_box_link(
            f"crate_{tag}_floor", lx, ly, t, colour, 1.0,
            f"{cx:.6g} {cy:.6g} {base:.6g} 0 0 0",
            mat=plastic_material("rough_plastic.png", competition="robosub")))
        for name, sx, sy, dx, dy in (
            ("xp", t, ly, (lx - t) / 2.0, 0.0),
            ("xn", t, ly, -(lx - t) / 2.0, 0.0),
            ("yp", lx, t, 0.0, (ly - t) / 2.0),
            ("yn", lx, t, 0.0, -(ly - t) / 2.0),
        ):
            parts.append(_box_link(
                f"crate_{tag}_{name}", sx, sy, lz, colour, 0.5,
                f"{cx + dx:.6g} {cy + dy:.6g} {base + lz / 2.0:.6g} 0 0 0"))

        # "INSIDE the bins will be images representing each role" -- handbook
        # p. 47, verbatim. The image lies FLAT ON THE BIN FLOOR, inside the
        # crate, facing up at the downward camera. That is the whole shape of
        # the task: you read the image looking down into the bin, then drop a
        # marker into that same bin.
        #
        # Two wrong versions preceded this. First it sat on TOP of the crate,
        # which is a lid over the opening you have to drop through. Then it
        # stood upright on a post beside the crate, which is readable but is
        # not what the handbook says and puts the image somewhere a marker
        # never goes.
        #
        # rpy 0 1.5708 0 lays the plate flat: _role_sign builds it with its
        # thickness along local x, so a 90-degree pitch turns the printed faces
        # to point up and down.
        image = spec["roles"][role]["task_image"]
        parts.append(_role_sign(
            spec, f"panel_{tag}", image,
            f"{cx:.6g} {cy:.6g} {base + t + 0.004:.6g} 0 1.5708 0",
            size=(lx - 2.0 * t) * 0.92))

    return model("robosub_bins", "\n".join(parts))


def robosub_torpedo_board(spec, role="survey_repair", model_name=None):
    """Task 4 -- a 0.6 m printed board on two PVC legs, with FOUR openings.

    Built wrong the first time as an H-shaped frame with two square gaps and
    the wrong overall size. The real board is a full 2 ft square standing on
    legs, printed with all four role images, and its openings are CIRCLES.

    SDF has no primitive with a hole, so the geometry here is the board's
    printed FACE (carrying the artwork, openings included) plus a ring of short
    boxes approximating each circular rim. For a torpedo that must physically
    pass through, use `robosub_torpedo_mesh` -- the vendored mesh has real
    holes. This variant is the regenerable one and is what a detector sees.
    """
    cfg = spec["props"]["torpedo_board"]
    size, th = cfg["size"], cfg["thickness"]
    r_pvc = spec["pvc_one_in"]
    legs = cfg.get("leg_height", 0.55)
    cz = legs + size / 2.0
    parts = []

    for i, y in ((0, -size / 2.0), (1, size / 2.0)):
        parts.append(_cylinder_link(
            f"leg_{i}", r_pvc, legs + size, WHITE, 2.0,
            f"0 {y:.6g} {(legs + size) / 2.0:.6g} 0 0 0",
            mat=pvc_material(WHITE)))

    # The printed face. One thin plate carrying the whole artwork, so the
    # openings, the red rims and the four role images are all in register.
    parts.append(_box_link(
        "board", th, size, size, WHITE, 3.0, f"0 0 {cz:.6g} 0 0 0",
        mat=textured_material(f"torpedo_panel_{role}.png", tint=1.0,
                              specular=0.06, roughness=0.6, emissive=0.10,
                              competition="robosub")))

    return model(model_name or f"robosub_torpedo_{role}", "\n".join(parts))


def robosub_octagon(spec):
    """Task 5: the 2.7 m octagon the vehicle surfaces inside.

    SURFACE-anchored: it floats, so its origin is the water surface at z = 0 and
    its geometry hangs from there. Getting this wrong puts it on the floor.
    """
    cfg = spec["props"]["octagon"]
    r = cfg["diameter"] / 2.0
    pr = cfg["pipe_radius"]
    colour = cfg["colour"]
    parts = []
    # Eight sides of a regular octagon, each a chord of the circumscribed circle.
    side = 2.0 * r * math.sin(math.pi / 8.0)
    apothem = r * math.cos(math.pi / 8.0)
    for i in range(8):
        ang = i * math.pi / 4.0
        parts.append(_cylinder_link(
            f"side_{i}", pr, side, colour, 1.0,
            f"{apothem * math.cos(ang):.6g} {apothem * math.sin(ang):.6g} 0 "
            f"0 1.5708 {ang + math.pi / 2.0:.6g}",
            mat=pvc_material(colour)))
    # Role images hang inward from the frame.
    for i, (role, ang) in enumerate((
        ("survey_repair", 0.0), ("search_rescue", math.pi),
    )):
        image = spec["roles"][role]["gate_images"][1]
        parts.append(_role_sign(
            spec, f"role_sign_{i}", image,
            # yaw = ang + pi so the printed face looks INTO the octagon --
            # "hanging role images ... facing inward". Plain `ang` aims the
            # image at the pool wall, where nothing can read it.
            f"{(apothem - 0.1) * math.cos(ang):.6g} "
            f"{(apothem - 0.1) * math.sin(ang):.6g} -0.35 0 0 "
            f"{ang + math.pi:.6g}"))
    return model("robosub_octagon", "\n".join(parts))


def robosub_resupply_table(spec):
    """Task 5 resupply table: 2 ft x 2 ft of 1/2 in PVC, on braced legs.

    The collectible items stand on this, so the top has to be a real surface
    with a rim -- a bare plate lets an item slide off the edge the moment the
    vehicle disturbs the water, and then the task is unrunnable.
    """
    cfg = spec["props"]["octagon"]
    size, h = cfg["table_size"], cfg["table_height"]
    r = spec["pvc_half_in"]
    colour = cfg["colour"]
    half = size / 2.0 - r
    parts = [_box_link(
        "top", size, size, 0.02, colour, 2.0, f"0 0 {h:.6g} 0 0 0",
        mat=pvc_material(colour))]

    # A rim around the top so items cannot slide off.
    for tag, sx, sy, dx, dy in (
        ("xp", r * 2, size, half, 0.0), ("xn", r * 2, size, -half, 0.0),
        ("yp", size, r * 2, 0.0, half), ("yn", size, r * 2, 0.0, -half),
    ):
        parts.append(_box_link(
            f"rim_{tag}", sx, sy, 0.03, colour, 0.2,
            f"{dx:.6g} {dy:.6g} {h + 0.02:.6g} 0 0 0",
            mat=pvc_material(colour)))

    for i, (dx, dy) in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1))):
        parts.append(_cylinder_link(
            f"leg_{i}", r, h, colour, 0.5,
            f"{dx * half:.6g} {dy * half:.6g} {h / 2.0:.6g} 0 0 0",
            mat=pvc_material(colour)))
    # Foot rails, as the CAD shows -- and they stop the table tipping.
    for tag, y in (("yp", half), ("yn", -half)):
        parts.append(_cylinder_link(
            f"foot_{tag}", r, size, colour, 0.4,
            f"0 {y:.6g} 0.03 0 1.5708 0", mat=pvc_material(colour)))
    return model("robosub_resupply_table", "\n".join(parts))


def robosub_path_marker(spec):
    """Orange path marker: two 18 in segments on PVC T-stands, off the bottom.

    Built wrong the first time as a decal lying flat on the pool floor. The CAD
    shows it raised on stands, which matters for the downward camera: an
    elevated marker has a shadow and a parallax offset from the floor tiling,
    and a decal has neither.

    "Each path marker is placed directly after the current task and points to
    the next task" -- so the ORIENTATION carries information, and there are
    exactly two: gate to slalom, and slalom to bins.
    """
    cfg = spec["props"]["path_marker"]
    seg, n = cfg["segment_length"], int(cfg["segments"])
    w, th, hgt = cfg["width"], cfg["thickness"], cfg["stand_height"]
    r = spec["pvc_half_in"]
    colour = cfg["colour"]
    parts = []
    total = seg * n
    for i in range(n):
        cx = -total / 2.0 + seg * (i + 0.5)
        parts.append(_box_link(
            f"segment_{i}", seg, w, th, colour, 0.6,
            f"{cx:.6g} 0 {hgt:.6g} 0 0 0",
            mat=material(colour, emissive_gain=0.32)))
        parts.append(_cylinder_link(
            f"stand_{i}", r, hgt, WHITE, 0.4,
            f"{cx:.6g} 0 {hgt / 2.0:.6g} 0 0 0", mat=pvc_material(WHITE)))
        parts.append(_cylinder_link(
            f"foot_{i}", r, w * 2.2, WHITE, 0.4,
            f"{cx:.6g} 0 0.02 1.5708 0 0", mat=pvc_material(WHITE)))
    return model("robosub_path_marker", "\n".join(parts))


def robosub_pinger(spec):
    """A Benthos ALP-365 stand-in.

    Body only. The acoustics live in the hydrophone model, which reads this
    prop's POSE from the course -- nothing about the sound is in the geometry,
    and pretending otherwise by modelling a transducer would be theatre.
    """
    cfg = spec["props"]["pinger"]
    return model("robosub_pinger", _cylinder_link(
        "body", cfg["radius"], cfg["height"], cfg["colour"], 0.5,
        f"0 0 {cfg['height'] / 2.0:.6g} 0 0 0"))


def robosub_collectible(spec, kind="bolt", model_name=None):
    """Task 5 pick-up items: jars (bolt, plug) and boxes (pill, bandage).

    MASS IS COMPUTED FROM VOLUME, not picked. These were 0.15 kg flat, which is
    LESS than the water they displace (192 cm3 jar, 212 cm3 box), so every one
    of them was positively buoyant: they rose off the resupply table and
    oscillated against the surface. That is not a physics-engine artefact, it
    is the model being wrong -- an object a vehicle is meant to pick up and
    carry has to sit still on a table first.

    Slightly negative (density 1150 kg/m3) so they rest on the table and stay
    where a manipulator puts them, while still being light enough to lift.
    """
    cfg = spec["props"]["collectible"]
    colour = cfg["colours"][kind]
    density = float(cfg.get("density", 1150.0))
    if kind in ("bolt", "plug"):
        r, h = cfg["jar_diameter"] / 2.0, cfg["jar_height"]
        mass = math.pi * r * r * h * density
        body = _cylinder_link("body", r, h, colour, mass,
                              f"0 0 {h / 2.0:.6g} 0 0 0")
    else:
        sz, h = cfg["box_size"], cfg["box_height"]
        mass = sz * sz * h * density
        body = _box_link("body", sz, sz, h, colour, mass,
                         f"0 0 {h / 2.0:.6g} 0 0 0")
    return model(model_name or f"robosub_item_{kind}", body, static=False)



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
    # ---- RoboSub 2026 -------------------------------------------------
    "robosub_gate": {
        "build": robosub_gate,
        "anchor": ANCHOR_FLOOR,
        "dynamic": False,
    },
    "robosub_slalom": {
        "build": robosub_slalom,
        "anchor": ANCHOR_FLOOR,
        "dynamic": False,
    },
    "robosub_bins": {
        "build": robosub_bins,
        "anchor": ANCHOR_FLOOR,
        "dynamic": False,
    },
    "robosub_torpedo_survey_repair": {
        "build": lambda s: robosub_torpedo_board(s, "survey_repair"),
        "anchor": ANCHOR_FLOOR,
        "dynamic": False,
    },
    "robosub_torpedo_search_rescue": {
        "build": lambda s: robosub_torpedo_board(s, "search_rescue"),
        "anchor": ANCHOR_FLOOR,
        "dynamic": False,
    },
    "robosub_octagon": {
        "build": robosub_octagon,
        "anchor": ANCHOR_SURFACE,
        "dynamic": False,
    },
    "robosub_resupply_table": {
        "build": robosub_resupply_table,
        "anchor": ANCHOR_FLOOR,
        "dynamic": False,
    },
    "robosub_path_marker": {
        "build": robosub_path_marker,
        "anchor": ANCHOR_FLOOR,
        "dynamic": False,
    },
    "robosub_pinger": {
        "build": robosub_pinger,
        "anchor": ANCHOR_FLOOR,
        "dynamic": False,
    },
    "robosub_item_bolt": {
        "build": lambda s: robosub_collectible(s, "bolt"),
        "anchor": ANCHOR_FLOOR,
        "dynamic": True,
    },
    "robosub_item_plug": {
        "build": lambda s: robosub_collectible(s, "plug"),
        "anchor": ANCHOR_FLOOR,
        "dynamic": True,
    },
    "robosub_item_pill": {
        "build": lambda s: robosub_collectible(s, "pill"),
        "anchor": ANCHOR_FLOOR,
        "dynamic": True,
    },
    "robosub_item_bandage": {
        "build": lambda s: robosub_collectible(s, "bandage"),
        "anchor": ANCHOR_FLOOR,
        "dynamic": True,
    },

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


# Height a flat floor decal must sit at so it is not swallowed by the pool.
#
# The floor VISUAL is a 0.01 m slab whose centre is offset +0.005 from the link,
# so its top face is 0.010 above the model origin -- and props are placed at the
# floor plane, not at the floor's top face. The green target mat was authored at
# 0.004 with a 0.006 thickness, which put it entirely INSIDE the floor slab: it
# rendered nothing at all, and it looked like the prop had never been added.
#
# 0.02 is what sauvc_starting_zone already used and is visibly correct. Anything
# flat that lies on the pool bottom must use this, not a hand-picked epsilon.
FLOOR_DECAL_Z = 0.02



def floor_depth_at(cfg: dict, x: float) -> float:
    """Depth of the pool floor below the surface at longitudinal position x.

    SAUVC's pool is not flat. The rulebook's side view gives 1.2 m at the two
    ends and 1.6 m at the centre -- a shallow V, 3.2 % grade. That matters for
    more than looks: a prop dropped at a fixed -1.6 m sits 0.34 m in the air at
    the target zone, the DVL's bottom-track altitude changes as the vehicle
    transits, and a depth hold that clears the floor mid-pool can ground at the
    ends.

    A pool with no `floor_edge_depth` is flat and this returns its one depth,
    so RoboSub and every existing course are unaffected.
    """
    deep = cfg["depth"]
    edge = cfg.get("floor_edge_depth")
    if not edge:
        return deep
    half = cfg["length"] / 2.0
    t = min(abs(float(x)) / half, 1.0) if half else 0.0
    return deep - (deep - edge) * t



def floor_pitch_at(cfg: dict, x: float) -> float:
    """Pitch of the floor at x, so a prop resting on it lies FLAT ON it.

    Anything sitting on a sloped floor is tilted by that slope -- trivially
    true, and it matters most for the flat decals. The 6 x 2.2 m target mat
    spans 2.2 m across the grade, so an unpitched mat has one edge buried in
    the floor and the other floating above it: the same swallowed-decal failure
    already fixed once for a different reason.

    Sign follows the surface normal: on the -x half the floor rises toward the
    wall, on the +x half it rises the other way. Zero for a flat pool.
    """
    edge = cfg.get("floor_edge_depth")
    if not edge:
        return 0.0
    half = cfg["length"] / 2.0
    if not half:
        return 0.0
    grade = (cfg["depth"] - edge) / half
    return math.atan(grade) * (1.0 if x < 0 else -1.0)


def pool(spec: dict, pool_cfg: dict = None, water_surface: str = "plane") -> str:
    """The pool shell: a textured floor and four walls, water surface at z = 0.

    Visuals are textured planes; collisions are primitive boxes. Keeping the two
    separate is deliberate, since mesh collisions for a pool this size are far
    more expensive than the boxes that describe it exactly.
    """
    cfg = pool_cfg or spec["pool"]
    comp = spec.get("competition", DEFAULT_COMPETITION)
    length = cfg["length"]
    width = cfg["width"]
    depth = cfg["depth"]
    t = cfg.get("wall_thickness", 0.2)

    hl, hw = length / 2.0, width / 2.0
    floor_z = -depth

    parts = []

    # THE FLOOR. Flat unless the pool declares `floor_edge_depth`, in which
    # case it is the shallow V the SAUVC side view draws: 1.6 m at the centre
    # rising to 1.2 m at both ends. Built as two tilted slabs meeting at x = 0
    # rather than a mesh, so the collision stays a primitive -- a mesh floor
    # for a 25 m pool costs far more to collide against than two boxes that
    # describe it exactly.
    edge_depth = cfg.get("floor_edge_depth")
    if not edge_depth:
        floor_body = [
            inertial(1000.0, (1000.0, 1000.0, 1000.0)),
            visual(
                "floor_visual",
                _geometry_box(length, width, 0.01),
                textured_material("pool_floor.png", competition=comp),
                f"0 0 {0.005:.6g} 0 0 0",
            ),
            collision("floor_collision", _geometry_box(length, width, 0.1),
                      "0 0 -0.05 0 0 0"),
        ]
        parts.append(link("floor", "\n".join(floor_body),
                          f"0 0 {floor_z:.6g} 0 0 0"))
    else:
        half = length / 2.0
        rise = cfg["depth"] - edge_depth          # 0.4 m over half the pool
        pitch = math.atan2(rise, half)            # tilt of each slab
        slab = math.hypot(half, rise)             # its true length
        for tag, sign in (("neg", -1.0), ("pos", 1.0)):
            # Centre of each slab: halfway along it, half the rise up.
            cx = sign * half / 2.0
            cz = floor_z + rise / 2.0
            body = [
                inertial(1000.0, (1000.0, 1000.0, 1000.0)),
                visual(
                    f"floor_{tag}_visual",
                    _geometry_box(slab, width, 0.01),
                    # Its OWN half of the floor image -- see gen_pool_texture.
                    textured_material(f"pool_floor_{tag}.png", competition=comp),
                    "0 0 0.005 0 0 0",
                ),
                collision(f"floor_{tag}_collision",
                          _geometry_box(slab, width, 0.1), "0 0 -0.05 0 0 0"),
            ]
            parts.append(link(
                f"floor_{tag}", "\n".join(body),
                f"{cx:.6g} 0 {cz:.6g} 0 {-sign * pitch:.6g} 0"))

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
                textured_material(texture, competition=comp),
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
    # SKIPPED for `water_surface: gerstner`, which uses Gazebo's own animated
    # Gerstner surface instead; two surfaces at z=0 would z-fight.
    if water_surface == "plane":
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
                        roughness=0.15, emissive=0.22, competition=comp),
                    "0 0 0 0 0 0",
                    cast_shadows=False,
                    transparency=0.62,
                ),
            ]),
        )
        )

    return model(f"{comp}_pool", "\n".join(parts))
