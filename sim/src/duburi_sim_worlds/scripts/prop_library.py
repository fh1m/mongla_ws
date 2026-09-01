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


def pvc_material(colour, emissive_gain: float = 0.28,
                 competition: str = DEFAULT_COMPETITION) -> str:
    """PVC tube: the extrusion seams in RELIEF, tinted to the pipe's colour.

    This used to be `material()` -- a flat colour with no maps at all -- and it
    is what the gate, the slalom, the bins pipework, the torpedo frame, the
    octagon, the resupply table and every path marker are made of. So the props
    a gate mission and a slalom mission actually look at were the smoothest
    surfaces in the scene, with whatever detail they had painted on. That is
    the worst case for feature matching: a descriptor keys on local gradients,
    and a painted gradient does not move when the light does.

    A separate `pvc_textured_material()` was added last round to fix this and
    then called by nothing, because it took no colour and every call site here
    passes one. Folding it in is what actually lands it: the shared grey PVC
    albedo is TINTED by the pipe's colour (textured_material accepts an RGB
    triple), so a red slalom pipe is red with seams rather than grey.

    `emissive_gain` keeps its meaning -- see `material()`: underwater scenes are
    fogged and singly lit, and without a self-illumination term props read as
    near-black to the cameras. It is applied PER CHANNEL, not as a grey: a flat
    grey emissive lifts all three channels equally, which is a desaturation
    term, and on a red pipe it is most of what made it pink.

    THE ALBEDO IS `albedo_pvc.png`, NOT `rough_pvc.png`. This passed the
    ROUGHNESS map as the albedo -- a mid-grey noise field, mean 0.618 -- so
    every pipe rendered at 62 % of its own colour before the grey emissive
    washed the rest out. Measured: [0.72, 0.11, 0.13] reached the screen near
    [0.65, 0.27, 0.28], which is the pale pink in the slalom screenshots.
    """
    e = [min(0.6, c * emissive_gain) for c in colour]
    return textured_material(
        "albedo_pvc.png", tint=colour, specular=0.55, emissive=e,
        roughness_map="rough_pvc.png", normal_map="norm_pvc.png",
        competition=competition,
    )


def textured_material(
    texture: str,
    tint: float = 0.55,
    specular: float = 0.08,
    roughness: float = 0.85,
    emissive: float = 0.0,
    roughness_map: str = "",
    normal_map: str = "",
    double_sided: bool = False,
    competition: str = DEFAULT_COMPETITION,
) -> str:
    """A PBR material driven by a texture in the <competition>_textures model.

    No emissive map on large surfaces. An emissive map in Gazebo adds the image
    at full strength on top of the lit result, which on a pool floor doubles the
    brightness and clips colour. Small props can take a flat emissive lift.

    `normal_map` is new and matters more than it sounds. Until now NOTHING in
    this tree used one, so every prop was a perfectly smooth surface with its
    detail painted on -- which is the single clearest tell of a CG render, and
    the worst case for feature matching, because descriptors key on local
    gradients and a painted gradient does not move with the light. SDF 1.9's
    <pbr><metal> has accepted normal_map all along.

    `double_sided` is required by any thin geometry you can get behind -- a
    latticed crate wall seen from inside the crate is a back face, and without
    this it is simply not drawn.
    """
    tex_model = texture_model(competition)
    uri = f"model://{tex_model}/{texture}"
    # Emissive may be a scalar (grey) or an RGB triple. The triple is not a
    # nicety: emissive is ADDED to the lit result, so a grey lift on a coloured
    # prop raises the two channels the colour does not use and desaturates it.
    ec = ((emissive, emissive, emissive)
          if isinstance(emissive, (int, float)) else tuple(emissive))
    e = f"{ec[0]:.3g} {ec[1]:.3g} {ec[2]:.3g} 1"
    # `tint` may be a scalar (grey) or an RGB triple. The triple exists because
    # a shared texture has to be able to carry a prop's own colour: the crate
    # lattice reuses the moulded-plastic albedo but a CleverMade crate is
    # near-black, and a scalar tint can only make that texture darker grey. The
    # first render after the crates went textured showed exactly that -- pale
    # grey lattice where the spec had said [0.18,0.18,0.20] all along.
    tc = (tint, tint, tint) if isinstance(tint, (int, float)) else tuple(tint)
    ts = f"{tc[0]:.3g} {tc[1]:.3g} {tc[2]:.3g}"
    return (
        "<material>\n"
        f"  <ambient>{ts} 1</ambient>\n"
        f"  <diffuse>{ts} 1</diffuse>\n"
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
        + (f"      <normal_map>model://{tex_model}/{normal_map}</normal_map>\n"
           if normal_map else "")
        + ("      <double_sided>true</double_sided>\n" if double_sided else "")
        +
        "    </metal>\n"
        "  </pbr>\n"
        "</material>"
    )


def stripe_material(texture: str = "albedo_pvc.png",
                    competition: str = DEFAULT_COMPETITION) -> str:
    """Gate stripe albedo with enough self-light to read through fog.

    The emissive lift is NOT decoration -- see material() above. Without it the
    post reads near-black at fog range and the detector never sees the gate.
    """
    return textured_material(
        texture, tint=0.75, specular=0.25, emissive=0.12,
        roughness_map="rough_pvc.png", normal_map="norm_pvc.png",
        competition=competition,
    )


def fabric_material(texture: str = "albedo_fabric.png",
                    emissive: float = 0.14,
                    competition: str = DEFAULT_COMPETITION, tint=0.80) -> str:
    """Inflated fabric and printed vinyl: matte, no specular hotspot.

    `tint` takes an RGB triple for a coloured surface -- the green target mat
    and the starting-zone marking share this weave texture and must keep their
    own colour, which a scalar tint can only darken toward grey.
    """
    return textured_material(
        texture, tint=tint, specular=0.06, emissive=emissive,
        roughness_map="rough_fabric.png", normal_map="norm_fabric.png",
        competition=competition,
    )


def plastic_material(texture: str = "albedo_plastic.png",
                     emissive: float = 0.10,
                     competition: str = DEFAULT_COMPETITION,
                     double_sided: bool = False, tint=0.72) -> str:
    """Moulded plastic drum walls and crate lattice."""
    return textured_material(
        texture, tint=tint, specular=0.14, emissive=emissive,
        roughness_map="rough_plastic.png", normal_map="norm_plastic.png",
        double_sided=double_sided, competition=competition,
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
           transparency: float = 0.0, label: str = None) -> str:
    """One <visual>, optionally carrying its OWN semantic label.

    IT DOES NOT DO WHAT IT LOOKS LIKE IT DOES. Measured 2026-08-31, and left
    here so the next attempt starts from the measurement instead of repeating
    it:

      * `gz-sim-label-system` really does attach at visual scope -- it carries a
        VisualTag component and the SDF is accepted without complaint.
      * A label on a visual DOES change which class the model is annotated as:
        with the gate's model-scope label suppressed and only the role boards
        labelled, `repair` appeared in 268 of 268 bounding-box frames.
      * But it does NOT produce a box PER VISUAL. Re-run with the frame visuals
        labelled `robosub_gate` and the two boards labelled `repair`/`rescue`,
        and 267 of 267 frames carried ONLY `robosub_gate`. One box per model is
        what the sensor emits; a visual label merely competes to name it.

    So sub-feature classes -- the `rescue`/`repair`, `fire`/`blood`, `hole` that
    `gate_rescue_repair.pt`, `bin_fire_blood.pt` and the torpedo mission are
    actually trained on -- CANNOT be produced this way, and a dataset captured
    in simulation still cannot train the models the missions run. Getting there
    needs each sub-feature to be its OWN MODEL, which costs the joints that make
    the boards swing, so it is a real trade and not a small edit.

    Nothing in the tree passes `label` today. The parameter stays because the
    finding above is worth more than the four lines it costs, and because the
    next person to try this will reach for exactly this argument.
    """
    shadows = "" if cast_shadows else "  <cast_shadows>false</cast_shadows>\n"
    trans = f"  <transparency>{transparency:.3g}</transparency>\n" if transparency else ""
    lab = ""
    if label:
        lid = detection_label(label)
        if not lid:
            raise SystemExit(
                f"visual {name!r}: {label!r} is not in DETECTION_CLASSES. Append "
                "it -- never insert, the index is the YOLO class id.")
        lab = ('  <plugin filename="gz-sim-label-system" '
               'name="gz::sim::systems::Label">\n'
               f"    <label>{lid}</label>\n"
               "  </plugin>\n")
    return (
        f'<visual name="{name}">\n'
        f"  <pose>{pose}</pose>\n"
        f"{shadows}{trans}{lab}"
        f"  {geometry}\n"
        f"{_indent(mat, 1)}\n"
        "</visual>"
    )


def collision(name, geometry, pose="0 0 0 0 0 0", surface: str = "") -> str:
    surf = f"{_indent(surface, 1)}\n" if surface else ""
    return (
        f'<collision name="{name}">\n'
        f"  <pose>{pose}</pose>\n"
        f"  {geometry}\n"
        f"{surf}"
        "</collision>"
    )


def link(name, body: str, pose="0 0 0 0 0 0") -> str:
    return f'<link name="{name}">\n  <pose>{pose}</pose>\n{_indent(body, 1)}\n</link>'


def hydrodynamics(link_name, xy_quad, z_quad, rot_quad) -> str:
    """Fossen drag on one link of a DYNAMIC prop.

    WITHOUT THIS A KNOCKED PROP RINGS FOREVER. Only the vehicle carried
    hydrodynamics, so a prop moved like a body in air -- and a self-righting
    flare with no drag is a pendulum with nothing to remove its energy.
    Measured before adding it: knocked to 87.6 degrees, then still swinging
    through 25 degrees 24 s later.

    Quadratic terms only. The linear ones matter near zero velocity where
    there is nothing left to damp, and leaving them out keeps this to the three
    numbers that can actually be derived from the prop's own dimensions:

        translational   1/2 * rho * Cd * A          (A = projected area)
        rotational      1/2 * rho * Cd * d * L^4/4  (a rod sweeping about one end)

    Added mass is deliberately absent, as on the vehicle: it belongs in the
    link's <fluid_added_mass>, and setting it in both places double-counts.
    """
    return (f'<plugin filename="gz-sim-hydrodynamics-system" '
            f'name="gz::sim::systems::Hydrodynamics">\n'
            f'  <link_name>{link_name}</link_name>\n'
            f'  <water_density>1000.0</water_density>\n'
            f'  <xUabsU>{-abs(xy_quad):.6g}</xUabsU>\n'
            f'  <yVabsV>{-abs(xy_quad):.6g}</yVabsV>\n'
            f'  <zWabsW>{-abs(z_quad):.6g}</zWabsW>\n'
            f'  <kPabsP>{-abs(rot_quad):.6g}</kPabsP>\n'
            f'  <mQabsQ>{-abs(rot_quad):.6g}</mQabsQ>\n'
            f'  <nRabsR>{-abs(rot_quad):.6g}</nRabsR>\n'
            f'</plugin>')


def rod_drag(diameter, length, cd=1.0, rho=1000.0):
    """(transverse, axial, rotational) quadratic drag for a rod. See `hydrodynamics`."""
    return (0.5 * rho * cd * diameter * length,
            0.5 * rho * cd * math.pi * (diameter / 2.0) ** 2,
            0.5 * rho * cd * diameter * length ** 4 / 4.0)


def plate_drag(side, thickness, cd=1.2, rho=1000.0):
    """(face, edge, rotational) quadratic drag for a flat plate. See `hydrodynamics`.

    A hanging board is nothing like a rod: broadside it sweeps its whole area,
    edge-on almost none. `hydrodynamics` takes one figure for BOTH horizontal
    axes, so the face value is applied to the in-plane horizontal axis too --
    an over-damping that costs nothing here, because a board hinged about that
    axis barely moves along it.
    """
    return (0.5 * rho * cd * side * side,
            0.5 * rho * cd * side * thickness,
            0.5 * rho * cd * side * side ** 3 / 4.0)


def joint(name, parent, child, jtype="fixed", axis=None, limit=None,
          damping=0.0, pose=None, axis2=None, stiffness=0.0, reference=0.0) -> str:
    """Join two links of a DYNAMIC model.

    A static model needs none of these -- <static>true</static> welds every
    link to the world implicitly, which is why this file had zero joints until
    props became pushable. The moment a model goes dynamic that stops being
    true: an unjointed multi-link model is N SEPARATE FREE BODIES that fly
    apart on the first physics step, with nothing logged.

    `parent` may be the literal `world`, which PINS the child in place inside an
    otherwise dynamic model -- the gate uses it so the moored frame cannot be
    shoved while the boards hanging off it still swing.

    axis    : (x, y, z) unit vector, required for a revolute joint.
    axis2   : second axis, for a `universal` joint -- a prop that can be shoved
              from any bearing needs two, or it yields along one heading and is
              rigid along the other.
    stiffness / reference :
              a torsional spring pulling the joint back toward `reference`.
              This is how a FASTENED board behaves: a 305 mm vinyl print
              zip-tied to a bar springs back because of its fastening, not
              because of its weight. Trying to do it with weight alone was
              measured and does not work -- 0.08 kg of restoring force over a
              0.15 m lever is 0.12 N.m, and the board crawled home over 20 s.
              Raising damping to compensate made it slower still, which is the
              tell that the missing term was the spring, not the damper.
    limit   : (lower, upper) radians. WITHOUT IT a struck board keeps rotating
              and winds through the frame it hangs from; the real thing is
              zip-tied and swings through a limited arc.
    damping : joint-space damping. Drag on the link (see `hydrodynamics`) does
              most of the work, but a hinge with neither rings at its own
              frequency for a long time.
    pose    : anchor, IN THE CHILD LINK'S FRAME. Default (None) hinges about the
              child's own centre, which for a hanging board means it pivots
              about its middle like a propeller. A board zip-tied along its top
              edge wants `pose` at that edge.
    """
    body = (f'  <parent>{parent}</parent>\n'
            f'  <child>{child}</child>\n')
    if pose is not None:
        body += f'  <pose>{pose}</pose>\n'
    for tag, vec in (("axis", axis), ("axis2", axis2)):
        if vec is None:
            continue
        lim = ""
        if limit is not None:
            lim = (f"    <limit>\n"
                   f"      <lower>{limit[0]:.6g}</lower>\n"
                   f"      <upper>{limit[1]:.6g}</upper>\n"
                   f"    </limit>\n")
        body += (f'  <{tag}>\n'
                 f'    <xyz>{vec[0]:.6g} {vec[1]:.6g} {vec[2]:.6g}</xyz>\n'
                 f'{lim}'
                 f'    <dynamics>\n'
                 f'      <damping>{damping:.6g}</damping>\n'
                 f'      <spring_stiffness>{stiffness:.6g}</spring_stiffness>\n'
                 f'      <spring_reference>{reference:.6g}</spring_reference>\n'
                 f'    </dynamics>\n'
                 f'  </{tag}>\n')
    return (f'<joint name="{name}" type="{jtype}">\n'
            f'{body}'
            f'</joint>')


def weld_all(root: str, children) -> list:
    """Fixed joints welding every `children` link to `root`. See `joint`."""
    return [joint(f"{root}_{c}_fix", root, c) for c in children]


def friction(mu=0.8) -> str:
    """An explicit contact friction surface.

    Nothing in this tree set friction before, so every contact ran on whatever
    the engine defaulted to -- fine while every prop was static and could not
    slide, load-bearing the moment one can. BOTH engine blocks are written
    because the world runs DART with bullet's collision detector, and which one
    reads the value is not worth guessing at.
    """
    return ('<surface>\n'
            '  <friction>\n'
            f'    <ode><mu>{mu}</mu><mu2>{mu}</mu2></ode>\n'
            f'    <bullet><friction>{mu}</friction>'
            f'<friction2>{mu}</friction2></bullet>\n'
            '  </friction>\n'
            '</surface>')


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


def _solid(name, geometry, mat, mass, inertia, pose="0 0 0 0 0 0", collide=True,
           visible=True, surface="", label=None):
    """A link with matching visual and collision geometry.

    `visible=False` gives a COLLISION-ONLY link. That exists because two
    co-planar surfaces both carrying a visual is not a cosmetic problem: the
    torpedo board's tiled collision plate sat in exactly the same plane, at
    exactly the same thickness, as its printed face, and the two z-fought into
    a streaky grey mess that hid the artwork the task is detected from. One
    physical surface should be drawn once.

    It is also free performance. RTF here is driven by visuals, not collisions
    (PHYSICS.md: cutting collision shapes 101 -> 37 changed it not at all), and
    every visual is paid four times over -- two cameras plus two bounding-box
    cameras.
    """
    parts = [inertial(mass, inertia)]
    if visible:
        parts.append(visual(f"{name}_visual", geometry, mat, label=label))
    if collide:
        parts.append(collision(f"{name}_collision", geometry, surface=surface))
    return link(name, "\n".join(parts), pose)


def _cylinder_link(
    name, radius, length, colour, mass, pose, collide=True, mat=None,
    visible=True, surface="", label=None,
):
    return _solid(
        name,
        _geometry_cylinder(radius, length),
        mat if mat is not None else material(colour),
        mass,
        cylinder_inertia(mass, radius, length),
        pose,
        collide,
        visible,
        surface,
        label,
    )


def _box_link(name, sx, sy, sz, colour, mass, pose, collide=True, mat=None,
              visible=True, surface="", label=None):
    return _solid(
        name,
        _geometry_box(sx, sy, sz),
        mat if mat is not None else material(colour),
        mass,
        box_inertia(mass, sx, sy, sz),
        pose,
        collide,
        visible,
        surface,
        label,
    )


# --------------------------------------------------------------------------
# props
# --------------------------------------------------------------------------

WHITE = [0.92, 0.92, 0.92]
# The RoboSub task frames are grey tube in the CAD, not white PVC. Kept separate
# from WHITE so the SAUVC props, which really are white PVC, do not move with it.
GREY_PVC = [0.62, 0.63, 0.65]


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


# --------------------------------------------------------------------------
# Flare mass budget -- why a knocked flare stands back up
# --------------------------------------------------------------------------
#
# The flare has to do two things that pull in opposite directions: STAY ON THE
# FLOOR when the pool current pushes it, and RIGHT ITSELF when the vehicle
# knocks it over. The tempting answer -- "make it buoyant and moor it" -- is
# wrong: a net-buoyant free body just accelerates upward until it hits the
# surface, because buoyancy supplies no restoring force in TRANSLATION.
#
# What works is the opposite pairing:
#
#   net weight NEGATIVE      -> it presses on the floor and stays put
#   centre of BUOYANCY well
#   ABOVE centre of MASS     -> it rights itself from any tilt
#
# Those are compatible: dense low ballast, near-buoyant volume above it. It is
# how a weighted marker buoy works, and how the real flare works.
#
# The numbers, all from this file's own geometry (water 1000 kg/m^3):
#
#   link        volume m^3    displaces kg     mass kg
#   pole        1.6085e-4     0.1609           0.020    (sealed hollow pipe)
#   cup_floor   9.47e-6       0.0095           0.008
#   8x rim      8.55e-6       0.0086           0.012 total
#   base        1.131e-4      0.1131           0.600    <- ballast
#   TOTAL       2.919e-4      0.2919           0.640
#
#   net weight in water   0.348 kg  = 3.41 N DOWN     -> cannot float away
#   centre of mass        z = 0.0425 m
#   centre of buoyancy    z = 0.2722 m               -> 0.230 m ABOVE the CoM
#   righting couple       0.513 * sin(theta) N*m, positive at every angle
#
# It also rights itself from FLAT, which a pivot argument would not show: the
# centre of mass sits 0.0425 m up standing and 0.060 m up lying on its side, so
# gravity alone has no barrier to getting up, and the buoyancy couple is pure
# gain on top of that.
#
# Two consequences worth stating because they are not obvious:
#
# - The golf ball needs only ~7 degrees of tilt to leave the cup. Its net
#   seating force is 0.0051 kg -- it very nearly floats -- so effective gravity
#   in the cup is 1.09 m/s^2 and the righting motion itself throws it out. The
#   knock task therefore works easily, AND any residual flare wobble ejects
#   balls on its own, which is what ball_check.py catches.
# - DART's own collision detector silently returns false for cylinder-box, and
#   the base disc on the pool floor IS a cylinder on a box. If anyone ever
#   reverts collision_detector to `dart` for the measured 5%, every flare falls
#   through the floor. The note also lives beside that setting in gen_world.py.
_FLARE_M_POLE = 0.020
_FLARE_M_CUP = 0.008
_FLARE_M_RIM = 0.0015
_FLARE_M_BASE = 0.600
_FLARE_BASE_R = 0.06
_FLARE_BASE_H = 0.01


def bump_flare(spec: dict, colour_name: str) -> str:
    """80 cm tall, ~1.6 cm diameter pole. Floor-anchored.

    A golf ball is balanced on top by the world generator rather than being part
    of this model, because the whole point of the task is to knock it off.
    """
    cfg = spec["props"]["bump_flare"]
    colour = cfg["colours"][colour_name]
    radius = cfg["diameter"] / 2.0
    height = cfg["height"]
    ball_r = spec["props"]["golf_ball"]["diameter"] / 2.0
    pvc = pvc_material(colour)

    parts = [
        _cylinder_link(
            "pole",
            radius,
            height,
            colour,
            _FLARE_M_POLE,
            f"0 0 {height / 2.0:.6g} 0 0 0",
            mat=pvc,
        ),
        # A CUP at the tip with a RIM WALL, so the ball is held rather than
        # balanced.
        #
        # Two failed attempts precede this. Balancing the ball on the flat top
        # of a 1.6 cm pole is a metastable equilibrium -- once water current was
        # added every ball rolled off within seconds. Adding a "cup" that was
        # actually a flat disc NARROWER than the ball changed nothing: a disc
        # has no wall, so there was still nothing to roll against.
        #
        # This is a ring of thin wall segments around a floor, inside diameter
        # a little under the ball's, so the ball nests into the opening and is
        # held by contact with the rim. A cylinder would close the top and the
        # AUV could not bump the ball out, which is the whole task.
        #
        # NOT a solid cup for the same reason the drum wall is a ring: the ball
        # has to be removable from above.
        _cylinder_link(
            "cup_floor",
            ball_r * 1.05,
            0.006,
            colour,
            _FLARE_M_CUP,
            f"0 0 {height + 0.003:.6g} 0 0 0",
            mat=pvc,
        ),
        # A small base disc, so a pole this thin does not look like it is
        # floating and has something to stand on.
        # THE BALLAST. Dense and low: it is what makes the flare stand up
        # again instead of lying where it was knocked. See _FLARE_M_BASE.
        _cylinder_link(
            "base", _FLARE_BASE_R, _FLARE_BASE_H, colour, _FLARE_M_BASE,
            f"0 0 {_FLARE_BASE_H / 2.0:.6g} 0 0 0", mat=pvc,
            surface=friction(0.8),
        ),
    ]
    # The rim: eight thin wall segments around the cup floor. Inside radius is
    # just under the ball's, so the ball rests ON the rim and is trapped
    # laterally while still liftable straight up.
    rim_r = ball_r * 0.92
    for i in range(8):
        a = i * math.pi / 4.0
        parts.append(_box_link(
            f"cup_rim_{i}", 0.004, rim_r * 0.85, 0.016, colour, _FLARE_M_RIM,
            f"{rim_r * math.cos(a):.6g} {rim_r * math.sin(a):.6g} "
            f"{height + 0.012:.6g} 0 0 {a:.6g}",
            mat=pvc))

    # DYNAMIC, so it must be WELDED. A static model is welded to the world
    # implicitly; drop that and these eleven links are eleven free bodies.
    parts.extend(weld_all(
        "base", ["pole", "cup_floor"] + [f"cup_rim_{i}" for i in range(8)]))
    parts.append(hydrodynamics("pole", *rod_drag(cfg["diameter"], height)))

    return model(f"sauvc_flare_{colour_name}", "\n".join(parts), static=False)


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
                # VISUAL ONLY. The ring needs 20 segments to LOOK like a drum,
                # but 20 collision boxes per drum is 80 shapes for four drums,
                # and the solver checks every one every step -- measured as the
                # dominant cost in a 101-shape course running at RTF 0.5. The
                # wall's physical job (keep a dropped ball in) is done by the
                # coarse collision ring added after this loop.
                f"wall_{i}", thickness, seg_w, height, exterior, 0.2, pose,
                collide=False,
                # Moulded-plastic wall rather than flat dark grey. The drum is
                # 60 cm across and fills the bottom camera during Target
                # Acquisition, so this is the single most valuable surface to
                # texture in the whole arena.
                mat=plastic_material(f"drum_wall_{colour_name}.png"),
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

    # INTERIOR LINER -- ONE cylinder, not a second 20-segment ring.
    #
    # It exists so the inside of the drum reads as the drum's colour from an
    # angle, not only from straight above. That is a purely visual job, and a
    # ring of 20 boxes does it no better than one cylinder while costing 20
    # draw calls per drum across every render pass. Measured: the four drums
    # were 169 of this course's 191 visuals, and stripping the props took the
    # sim from RTF 0.5 to 1.0 -- rendering, not collision, is what props cost.
    parts.append(
        link(
            "liner",
            "\n".join([
                inertial(0.05, cylinder_inertia(0.05, radius, height)),
                visual(
                    "liner_visual",
                    _geometry_cylinder(radius - thickness * 1.4, height * 0.98),
                    material(colour, emissive_gain=0.30),
                ),
            ]),
            f"0 0 {height / 2.0:.6g} 0 0 0",
        )
    )

    # COARSE COLLISION RING. The 20 visual segments above carry no collision;
    # this is what actually stops a dropped ball rolling out sideways. Four
    # boxes instead of twenty, arranged as a square around the drum's inside
    # diameter -- a ball that clears the rim still cannot escape, and the
    # solver checks 4 shapes per drum rather than 20.
    #
    # Deliberately NOT a solid cylinder: that would close the top and make the
    # drop impossible, which is the reason the wall was a ring in the first
    # place.
    inner = radius - thickness
    for tag, sx, sy, dx, dy in (
        ("xp", thickness * 2, inner * 2, inner, 0.0),
        ("xn", thickness * 2, inner * 2, -inner, 0.0),
        ("yp", inner * 2, thickness * 2, 0.0, inner),
        ("yn", inner * 2, thickness * 2, 0.0, -inner),
    ):
        parts.append(link(
            f"collide_{tag}",
            "\n".join([
                inertial(0.1, (0.01, 0.01, 0.01)),
                collision(f"collide_{tag}_collision",
                          _geometry_box(sx, sy, height)),
            ]),
            f"{dx:.6g} {dy:.6g} {height / 2.0:.6g} 0 0 0",
        ))

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
            # Vinyl, not paint. It was the one large SAUVC surface still on a
            # flat-colour material, and it is the biggest single thing the
            # bottom camera sees while hunting the target zone -- so it is also
            # the biggest smooth-CG tell in the frame.
            fabric_material("albedo_fabric.png", emissive=0.14,
                            tint=cfg["colour"]),
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
                            # Painted/taped surface marking, so it takes
                            # the same vinyl treatment as the target mat.
                            fabric_material("albedo_fabric.png",
                                            emissive=0.22,
                                            tint=colour),
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


def _role_sign(spec, name, image, pose, size=None, collide=False, mass=0.05,
               label=None):
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
        name, cfg["thickness"], side, side, WHITE, mass, pose, collide=collide,
        label=label,
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

    SURFACE-anchored, and that is the whole point of this prop. Handbook p. 32,
    verbatim:

        "It is buoyant, floating just below the surface and moored to the
         bottom. ... The AUV can pass through the gate at any depth from the
         floor to just below the gate."
        "The AUV chooses a marine animal by passing under a specific side."

    So the frame hangs DOWN from just under the water surface and the clear
    water is BELOW it -- the opposite of a gate standing on the floor, which is
    how this was modelled until now. It changes the one number a gate mission
    must get right: approach depth. Floor-anchored, the top bar sat 0.58 m deep
    and the legs blocked the floor; hung from the surface in RoboSub's 2.1 m
    pool the bar is 0.1 m deep, the legs reach 1.62 m, and the vehicle passes
    between them anywhere below that.

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
    # How far under the surface the buoyant frame floats. "Just below" is not a
    # number the handbook gives, so it is a spec key rather than a literal.
    sub = float(cfg.get("submergence", 0.1))
    bar_z = -sub
    parts = []

    # Width runs along y; the vehicle passes along x. Legs hang from the bar.
    parts.append(_cylinder_link(
        "top_bar", r, w, WHITE, 5.0, f"0 0 {bar_z:.6g} 1.5708 0 0",
        mat=pvc_material(WHITE, competition="robosub")))

    for side, y, upper, lower in (
        ("port", -half, black, red),      # left  as seen from the front
        ("stbd", half, red, black),       # right as seen from the front
    ):
        # A plain stub between the bar and the first colour band, as the CAD
        # shows, then the two bands. Everything hangs BELOW bar_z.
        parts.append(_cylinder_link(
            f"post_{side}", r, h, WHITE, 4.0,
            f"0 {y:.6g} {bar_z - h / 2.0:.6g} 0 0 0", mat=pvc_material(WHITE, competition="robosub")))
        for tag, colour, cz in (
            ("upper", upper, bar_z - 0.1524 - band / 2.0),
            ("lower", lower, bar_z - 0.1524 - band * 1.5),
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
        f"0 0 {bar_z - cfg['divider_drop'] / 2.0:.6g} 0 0 0",
        # SOLID -- a 610 mm plate hanging down the middle of the gate mouth is
        # what makes the two sides two separate choices. Driven through, the
        # gate is one wide opening and the side-selection task is not a task.
        mat=material(red, emissive_gain=0.22)))

    # Role signs hang from the bar, 152.4 mm below it, one per role.
    sign_side = spec["sign"]["size"]
    sign_names = []
    for role, y in (("survey_repair", -w * 0.22), ("search_rescue", w * 0.22)):
        image = spec["roles"][role]["gate_images"][0]
        sign_names.append(f"sign_{role}")
        # A PLAIN LINK, and per-sign labels are NOT possible. This comment
        # used to claim the opposite -- "a nested <model> is its own merge
        # group, so it can carry its own class" -- and it SURVIVED THE REVERT
        # of the experiment that disproved it, so it sat here asserting a
        # capability the code beneath it does not have.
        #
        # Measured twice, ~270 frames each, both reverted whole: gz merges
        # everything under a top-level model instance into ONE box and picks
        # one label for it (Ogre2BoundingBoxCamera::MergeMultiLinksModels2D).
        # Visual-scope labels gave 267/267 frames of `robosub_gate`; nesting
        # changed nothing. That is why `repair`/`rescue` are deliberately absent
        # from DETECTION_CLASSES -- nothing emits them.
        #
        # The only remaining path is each sub-feature as its own TOP-LEVEL model
        # placed by the course, which costs the hinge joints that let these
        # boards swing. Settled negative; stop re-testing it each round.
        parts.append(_role_sign(
            spec, f"sign_{role}", image,
            # rpy 0 0 0 -- the plate already faces along x, at the AUV.
            f"0 {y:.6g} {bar_z - 0.1524 - spec['sign']['size'] / 2.0:.6g} 0 0 0",
            # SOLID. These are 305 mm corrugated-plastic boards hanging in the
            # gate mouth; a hull driving through one is the sim lying about
            # where the clear water is, which is the one thing a gate mission
            # has to get right.
            collide=True,
            # Corrugated plastic, and it is BUOYANT -- 305 x 305 x 4 mm
            # displaces 0.372 kg against about 0.15 kg of board. What holds it
            # flat in the pool is the fastening, so that is where the restoring
            # force is modelled (the hinge's spring, below) rather than faked by
            # making the board heavy. An earlier attempt did exactly that and
            # the measurement said no: weight alone brought it back over 20 s.
            mass=0.15,
            ))

    # "moored to the bottom" -- two lines from the foot of each leg to the
    # floor. Non-colliding: they are rope, and a vehicle that clips one should
    # not be stopped by it. They exist because they are VISIBLE, a near-vertical
    # line either side of the opening that a detector will see and that the real
    # course really has.
    leg_foot = bar_z - h
    floor_z = -float(spec["pool"]["depth"])
    tether = max(0.05, leg_foot - floor_z)
    for side, y in (("port", -half), ("stbd", half)):
        parts.append(_cylinder_link(
            f"mooring_{side}", 0.006, tether, (0.15, 0.15, 0.14), 0.1,
            f"0 {y:.6g} {leg_foot - tether / 2.0:.6g} 0 0 0", collide=False,
            mat=material((0.15, 0.15, 0.14))))

    # --- what moves, and what does not -------------------------------------
    #
    # The gate WAS static, which is why a hull that hit a role marker stopped
    # dead against it and nothing budged: Round 6 gave the signs collision, but
    # <static>true</static> welds every link to the world, so no amount of
    # collision can make a static link move.
    #
    # It is now dynamic with the FRAME PINNED. A fixed joint to `world` holds
    # the moored 3 m structure exactly where the course put it -- it is far
    # heavier than the hull, and a gate that drifted would move the geometry
    # the scorer measures the run against. Everything hanging off the bar
    # swings.
    parts.extend(weld_all("top_bar", [
        "post_port", "post_stbd",
        "band_port_upper", "band_port_lower",
        "band_stbd_upper", "band_stbd_lower",
        "mooring_port", "mooring_stbd",
    ]))
    parts.append(joint("gate_mooring", "world", "top_bar"))

    # The boards hinge about the bar's own axis (y), so they swing FORE AND AFT
    # -- the direction a hull transiting along x actually pushes them. The
    # anchor sits at each board's top edge, not its centre, or it pivots about
    # its middle like a propeller.
    face, edge, rot = plate_drag(sign_side, spec["sign"]["thickness"])
    for _name in sign_names:
        parts.append(joint(f"{_name}_hinge", "top_bar", _name, "revolute",
                           axis=(0.0, 1.0, 0.0), limit=(-1.0, 1.0),
                           # The FASTENING is the restoring force, not gravity.
                           # Measured with gravity alone (damping 0.02): -56.9
                           # deg, still -5.3 deg at t=21.5 s. Raising damping to
                           # 1.5 made it WORSE -- -41.3 deg at t=15.4 s, a
                           # near-linear crawl -- which is what an overdamped
                           # hinge with almost no restoring torque looks like.
                           # k = 3.0 N.m/rad against I ~ 0.014 kg.m^2 puts the
                           # natural period near 0.4 s, and damping 0.4 is close
                           # to critical, so it deflects and comes straight back.
                           stiffness=3.0, reference=0.0, damping=0.4,
                           pose=f"0 0 {sign_side / 2.0:.6g} 0 0 0"))
        parts.append(hydrodynamics(_name, face, edge, rot))

    # The divider is a 610 mm plate hanging down the gate mouth; same hinge.
    _dw, _dd = cfg["divider_width"], cfg["divider_drop"]
    d_face, d_edge, d_rot = plate_drag(_dd, _dw)
    parts.append(joint("divider_hinge", "top_bar", "divider", "revolute",
                       axis=(0.0, 1.0, 0.0), limit=(-1.0, 1.0),
                       stiffness=6.0, reference=0.0, damping=0.8,
                       pose=f"0 0 {_dd / 2.0:.6g} 0 0 0"))
    parts.append(hydrodynamics("divider", d_face, d_edge, d_rot))

    return model("robosub_gate", "\n".join(parts), static=False)


def robosub_slalom(spec):
    """Task 2: one set of three moored pipes, WHITE / RED / WHITE.

    Three SETS make the task; this is one, so a course places three of them and
    can stagger the heights the handbook calls for ('moored at different
    heights') per instance rather than baking one arrangement in.
    """
    cfg = spec["props"]["slalom"]
    r = spec["pvc_one_in"]
    h, gap = cfg["height"], cfg["spacing"]
    # Ballasted the same way as the flare, for the same reason and by the same
    # arithmetic -- see the flare mass budget above. One mechanism, verified
    # once. A 1 in PVC pipe of this length displaces far more than it weighs,
    # so without the anchor disc a pushable pipe would simply surface.
    pipe_m = 0.35                      # hollow capped 1 in PVC
    anchor_r, anchor_h, anchor_m = 0.05, 0.012, 0.90
    parts = []
    for name, y, colour in (
        ("pipe_left", gap, cfg["colours"]["white"]),
        ("pipe_centre", 0.0, cfg["colours"]["red"]),
        ("pipe_right", -gap, cfg["colours"]["white"]),
    ):
        parts.append(_cylinder_link(
            name, r, h, colour, pipe_m,
            f"0 {y:.6g} {anchor_h + h / 2.0:.6g} 0 0 0",
            mat=pvc_material(colour, competition="robosub")))
        parts.append(_cylinder_link(
            f"{name}_anchor", anchor_r, anchor_h, colour, anchor_m,
            f"0 {y:.6g} {anchor_h / 2.0:.6g} 0 0 0",
            mat=pvc_material(colour, competition="robosub"), surface=friction(0.8)))

    # THREE INDEPENDENT PIPES, EACH MOORED TO THE FLOOR ON ITS OWN HINGE.
    #
    # This used to be `weld_all(names[0], names[1:])` -- one rigid body, on the
    # reasoning that a hull clipping one pipe should shove the set "rather than
    # scattering three loose poles". That is not what the handbook describes
    # ("moored at different heights to the floor, and floating vertically",
    # i.e. each pipe individually) and it is not what the pool does: clipping
    # the left pipe left the centre and right ones swinging too, which taught a
    # slalom mission that the whole gap moves when you touch its edge.
    #
    # Simply un-welding them is NOT enough, and the first attempt proved it:
    # measured, an unmoored pipe took the hit, travelled 2.29 m in 3.7 s and
    # then diverged the solver outright. A free body is not what a moored pipe
    # is.
    #
    # So each pipe is a self-righting inverted pendulum, which is what the
    # handbook actually describes. The anchor disc is WELDED TO THE WORLD (it
    # is the mooring block) and the pipe hangs off it on a UNIVERSAL joint at
    # its base -- two axes, because a hull can brush a pipe from any bearing
    # and a single hinge would yield along one heading and stand rigid along
    # the other. The pipe displaces 0.789 kg against 0.35 kg of mass, so 0.44 kg
    # of buoyancy above the hinge stands it upright and rights it after a knock.
    # It can be pushed over and it cannot be pushed AWAY.
    for pipe in ("pipe_left", "pipe_centre", "pipe_right"):
        parts.append(joint(f"{pipe}_mooring", "world", f"{pipe}_anchor"))
        parts.append(joint(
            f"{pipe}_hinge", f"{pipe}_anchor", pipe, "universal",
            axis=(1.0, 0.0, 0.0), axis2=(0.0, 1.0, 0.0),
            limit=(-1.2, 1.2), damping=0.05,
            pose=f"0 0 {-h / 2.0:.6g} 0 0 0"))

    for name in ("pipe_left", "pipe_centre", "pipe_right"):
        parts.append(hydrodynamics(name, *rod_drag(2.0 * r, h)))
    return model("robosub_slalom", "\n".join(parts), static=False)


def robosub_bins(spec):
    """Task 3 -- a BRANCHING 3D pipeline with four crates cantilevered off it.

    Third attempt, and the previous two were both wrong in instructive ways.
    First: four loose crates on the floor. Then: a flat rectangular frame with
    the crates sitting inside it and pipe ends poking out past them, which is
    what the top view showed and is not what the CAD shows at all.

    From the handbook figure the structure is a SPINE with T-branches, each
    branch reaching out and UP to carry one crate on the end of a short riser.
    The four crates sit at DIFFERENT HEIGHTS and different reaches -- that is
    the "3D" in "3D pipeline made from PVC", and it is the whole difficulty of
    the task: the AUV cannot hold one altitude and drop into all four, it has
    to re-acquire depth per bin.

    Each crate carries its role image on its floor, facing up, because that is
    how the downward camera reads it on the way in.

    Crates are CleverMade 25 L: 0.335 x 0.335 x 0.28 m, square in plan.
    """
    cfg = spec["props"]["bin"]
    lx, ly, lz, t = cfg["length"], cfg["width"], cfg["height"], cfg["wall"]
    colour = cfg["colour"]
    ph, span = cfg["pipeline_height"], cfg["pipeline_span"]
    r = spec["pvc_three_quarter_in"]
    parts = []

    def pipe(name, length, pose):
        # SOLID. This was scenery on the argument that nothing in the task
        # pushes the frame -- true of the task, false of the vehicle, which
        # was descending straight through the pipework to reach the crates.
        # An approach that only works because the sim lets the hull occupy the
        # structure is an approach that fails in the pool.
        parts.append(_cylinder_link(
            name, r, length, WHITE, 1.0, pose,
            mat=pvc_material(WHITE, competition="robosub")))

    # The spine, along x, on two feet.
    pipe("spine", span, f"0 0 {ph:.6g} 0 1.5708 0")
    for i, x in ((0, -span / 2.2), (1, span / 2.2)):
        pipe(f"foot_riser_{i}", ph, f"{x:.6g} 0 {ph / 2.0:.6g} 0 0 0")
        pipe(f"foot_{i}", 0.5, f"{x:.6g} 0 0.03 1.5708 0 0")

    # Four branches off the spine. Each reaches out in y and rises to its own
    # height, so no two crates share an altitude.
    #                 tag          role            x        y     lift
    layout = (
        ("sr_a",      "survey_repair", -span * 0.30,  0.52,  0.30),
        ("rescue_a",  "search_rescue", -span * 0.10, -0.52,  0.10),
        ("sr_b",      "survey_repair",  span * 0.14,  0.52, -0.05),
        ("rescue_b",  "search_rescue",  span * 0.34, -0.52,  0.18),
    )
    for tag, role, bx, by, lift in layout:
        z = ph + lift
        # Horizontal arm out to the crate, then a short riser up to its base.
        pipe(f"arm_{tag}", abs(by),
             f"{bx:.6g} {by / 2.0:.6g} {ph:.6g} 1.5708 0 0")
        pipe(f"riser_{tag}", abs(lift) + 0.02,
             f"{bx:.6g} {by:.6g} {ph + lift / 2.0:.6g} 0 0 0")

        base = z
        # THE ROLE IMAGE IS THE CRATE FLOOR. Handbook, verbatim: "Inside the
        # bins will be images representing each role." Read literally, and this
        # prop's own docstring has said so since it was written -- the upright
        # placards were the drift.
        #
        # It also removes a whole class of defect rather than tuning it. The
        # placard hung off the pipework on a post that ran up through the
        # middle of the sign face (`base .. base + side*1.5` through a sign
        # centred at `base + side*0.75`), and the two +x crates' signs landed at
        # x = -0.02, in among the other crates' arms. No sign, no post, nothing
        # across the opening a marker is dropped through.
        #
        # An SDF box stretches its albedo across each face exactly once, and a
        # CleverMade crate is square in plan (0.335 x 0.335), so the glyph maps
        # undistorted. Emissive stays low: this face is what the detector
        # classifies and a lift washes the colour out.
        #
        # CONSEQUENCE, stated because it is a real capability change: the
        # FORWARD camera can no longer read a bin's role on approach, only the
        # DOWNWARD one. That is already how the bin mission works
        # (`align('fire', camera='downward', ...)`), so no mission changes --
        # but it should not be discovered later.
        image = spec["roles"][role]["task_image"]
        parts.append(_box_link(
            f"crate_{tag}_floor", lx, ly, t, WHITE, 1.0,
            f"{bx:.6g} {by:.6g} {base:.6g} 0 0 0",
            mat=textured_material(f"role_{image}.png", tint=1.0, specular=0.06,
                                  roughness=0.55, emissive=0.10,
                                  competition="robosub")))
        # LATTICED WALLS. The rulebook names a CleverMade collapsible crate and
        # its walls are moulded lattice -- you can see into it and through it.
        # These were solid slabs, and worse, they were the ONLY links in the
        # prop with no `mat=` at all, so they fell through to a flat
        # solid-colour material while the crate floor got a textured PBR one.
        # A flat black slab in fog is exactly the "solid grey box" the render
        # showed.
        #
        # The visual is a shared mesh (132 triangles, instanced 16 times); the
        # collision stays a plain box, because a marker only has to land inside
        # the crate, not thread a bar. Double-sided, or a thin wall vanishes
        # when seen from within.
        wall_mat = plastic_material("albedo_plastic.png", competition="robosub",
                                    double_sided=True, tint=colour,
                                    emissive=0.04)
        for name, sx, sy, dx, dy, yaw in (
            ("xp", t, ly, (lx - t) / 2.0, 0.0, 0.0),
            ("xn", t, ly, -(lx - t) / 2.0, 0.0, 0.0),
            ("yp", lx, t, 0.0, (ly - t) / 2.0, 1.5708),
            ("yn", lx, t, 0.0, -(ly - t) / 2.0, 1.5708),
        ):
            pose = (f"{bx + dx:.6g} {by + dy:.6g} "
                    f"{base + lz / 2.0:.6g} 0 0 {yaw:.6g}")
            parts.append(link(f"crate_{tag}_{name}", "\n".join([
                inertial(0.5, box_inertia(0.5, sx, sy, lz)),
                visual(f"crate_{tag}_{name}_visual",
                       '<geometry><mesh><uri>'
                       'model://robosub_meshes/meshes/crate_wall.obj'
                       '</uri></mesh></geometry>', wall_mat),
                collision(f"crate_{tag}_{name}_collision",
                          _geometry_box(sx, sy, lz)),
            ]), pose))

        # THE LIGHT, and the magnetic detector beside it.
        #
        # 2026 adds "integrated lights and magnetic detectors" to the pipework,
        # scored 500 per light. The lens is a small emissive disc on the riser
        # where a downward camera can see it; the scorer extinguishes it on a
        # close pass. Emissive rather than an actual <light>: a point light per
        # crate is four more shadow casters on a render-bound sim, and what a
        # detector reads is the bright lens, not the pool it lights.
        # It sat on the deleted placard's standoff, so it moves onto the crate
        # rim: an up-facing lens on the outboard wall top, which is where a
        # downward camera coming in on the bin actually sees it. Off the
        # pipework by ~0.17 m rather than on it -- the handbook says
        # "integrated lights" without saying where, and beside the bin it
        # lights is the reading a scorer can check.
        parts.append(_cylinder_link(
            f"light_{tag}", r * 1.9, 0.014, (1.0, 0.92, 0.55), 0.05,
            f"{bx - lx / 2.0 + r * 2.0:.6g} {by:.6g} "
            f"{base + lz:.6g} 0 0 0", collide=False,
            mat=material((1.0, 0.92, 0.55), emissive_gain=0.85)))
    return model("robosub_bins", "\n".join(parts))


def torpedo_layout(spec: dict):
    """The Task 4 board's printed elements, PACKED, in plate coordinates.

    THE ONE PLACE THE BOARD'S GEOMETRY IS DECIDED. Returns
    ``[{"kind", "y", "z", "half_width", "radius"}, ...]`` -- `kind` is
    ``"image"``, ``"large"`` or ``"small"``; `radius` is the OPENING radius
    (the hole the mesh cuts and the collision leaves out), None for an image;
    `half_width` is the element's drawn half-extent, which for a ring is the
    RED ANNULUS's outer radius and is therefore larger than `radius`.

    WHY THIS EXISTS. The column centres used to be typed into the spec by hand:
    four columns 0.235 apart, while a large ring's outer radius (0.157 UV) plus
    an image's half-width (0.105) needs 0.262 between their centres. An overlap
    was guaranteed by arithmetic the moment those numbers were written, and the
    large ring ran 0.006 past the board's edge -- which is what "emojis in the
    holes" and "holes hitting the walls of the board" were. Nothing checked it,
    and THAT was the bug; the render was only the symptom.

    So the spec now declares the ORDER of a row's slots and nothing else. The
    positions are packed here from the same radii the mesh cuts and the texture
    paints, spread with equal gaps across the width left inside `inset`. Change
    a radius and the packing moves with it; make the elements too wide and
    `test_torpedo_board.py` fails on a non-positive gap rather than shipping a
    board that overlaps.

    SLOT ORDER IS LEFT-TO-RIGHT AS PRINTED, which is descending y: the board's
    face is +x, and the UV mapping is u = 0.5 - y/size, so +y is image-left.

    y is across the board, z is up, both relative to the plate centre.
    """
    cfg = spec["props"]["torpedo_board"]
    half = cfg["size"] / 2.0
    band = cfg["ring_band"]
    radius = {"large": cfg["large_opening"] / 2.0,
              "small": cfg["small_opening"] / 2.0}
    half_width = dict({k: r * band for k, r in radius.items()},
                      image=cfg["image_size_m"] / 2.0)
    usable = 2.0 * (half - cfg["inset"])

    out = []
    for row in cfg["rows"]:
        z = row["z"] * half
        widths = [2.0 * half_width[k] for k in row["slots"]]
        # Outer edges flush with the usable width, equal gaps between. n-1
        # gaps, not n+1: spreading to the edges is what buys the separation,
        # and a non-positive gap here is the overlap the test looks for.
        gap = (usable - sum(widths)) / max(1, len(widths) - 1)
        y = usable / 2.0
        for kind, w in zip(row["slots"], widths):
            out.append({"kind": kind, "y": y - w / 2.0, "z": z,
                        "half_width": w / 2.0, "radius": radius.get(kind)})
            y -= w + gap
    return out


def torpedo_gaps(spec: dict):
    """Per-row edge-to-edge gap between adjacent elements, in metres.

    Split out of `torpedo_layout` so the overlap test asserts on the number
    itself rather than re-deriving it from placed centres.
    """
    cfg = spec["props"]["torpedo_board"]
    usable = 2.0 * (cfg["size"] / 2.0 - cfg["inset"])
    band = cfg["ring_band"]
    hw = {"large": cfg["large_opening"] / 2.0 * band,
          "small": cfg["small_opening"] / 2.0 * band,
          "image": cfg["image_size_m"] / 2.0}
    return [(usable - sum(2.0 * hw[k] for k in row["slots"]))
            / max(1, len(row["slots"]) - 1)
            for row in cfg["rows"]]


def torpedo_openings(spec: dict):
    """The board's openings, in PLATE coordinates: [(y, z, radius), ...].

    The collision plate is tiled around this list, the visual mesh is cut from
    it and the printed artwork draws it, because they were previously written
    out separately and drifted: the texture painted FOUR circles while the
    collision cut TWO, somewhere else entirely. The holes you could see were
    not the holes you could shoot through -- so a mission aimed at the artwork
    struck solid board and the sim scored it a miss, for a reason no operator
    could see.

    2026 LAYOUT: FOUR openings, two large and two small, alternating with the
    four role images across two rows -- the arrangement the TeamTime "Task 4 -
    Deploy (Torpedoes)" slide shows. The 2025 handbook says only "two different
    size openings", which is about the two SIZES and does not contradict the
    count.
    """
    return [(e["y"], e["z"], e["radius"]) for e in torpedo_layout(spec)
            if e["radius"] is not None]


def torpedo_images(spec: dict):
    """The board's printed role images: [(y, z, side), ...], plate coordinates."""
    return [(e["y"], e["z"], 2.0 * e["half_width"])
            for e in torpedo_layout(spec) if e["kind"] == "image"]


def _plate_uv(spec, y, z):
    """Plate (y, z) -> the PIL pixel fraction the texture generator paints at.

    MEASURED, not reasoned to. The board is a MESH now, so Ogre's box-face
    convention stopped applying and this line kept the old `0.5 - y` u from it,
    which painted every ring and image on the mirrored side of the board: at
    1.0 m the render showed eight circles, four cut and four painted, side by
    side. Two rounds of reasoning about which axis was flipped produced two
    different wrong answers, so it was settled with a diagnostic texture --
    four labelled quadrants, one render -- which gives:

        sampled_u = vt_u          (no horizontal flip)
        sampled_v = 1 - vt_v      (OBJ `vt` is bottom-origin, PIL row 0 is top)

    The mesh writes vt = (0.5 + y/size, 0.5 + z/size), so a point at (y, z) is
    sampled from PIL (0.5 + y/size, 0.5 - z/size) -- which is what this returns.
    The v term therefore looks like the mesh's INVERSE and is correct; only u
    was wrong. Change `gen_prop_meshes.add()` and this must move with it.
    """
    size = spec["props"]["torpedo_board"]["size"]
    return (0.5 + y / size, 0.5 - z / size)


def torpedo_openings_uv(spec: dict):
    """`torpedo_openings` mapped into texture UV: [(u, v, r_frac), ...]."""
    size = spec["props"]["torpedo_board"]["size"]
    return [(*_plate_uv(spec, y, z), r / size)
            for (y, z, r) in torpedo_openings(spec)]


def torpedo_images_uv(spec: dict):
    """`torpedo_images` mapped into texture UV: [(u, v, side_frac), ...]."""
    size = spec["props"]["torpedo_board"]["size"]
    return [(*_plate_uv(spec, y, z), side / size)
            for (y, z, side) in torpedo_images(spec)]


def _plate_with_holes(prefix, size, thickness, openings, cz, strips=26):
    """Collision for a flat plate that a projectile can pass THROUGH.

    SDF has no primitive with a hole, and that is not a cosmetic problem here:
    the rulebook scores a torpedo differently for passing through an opening
    (full points) than for striking the board (partial), so a solid plate makes
    the task's own distinction unmeasurable. A good shot and a near miss both
    just bounce.

    The plate is therefore tiled in horizontal strips, and each strip is cut
    into the spans that are NOT inside an opening -- one collision box per
    span, typically 60-70 for two holes. They are primitives, so this is cheap,
    and the strip resolution only has to be finer than the projectile: at 26
    strips a 0.6 m board has 23 mm rows against a 51 mm round.

    `openings` is [(y, z, radius), ...] in plate coordinates. The plate faces
    along x, so its extent runs in y (across) and z (up).
    """
    half = size / 2.0
    step = size / strips
    parts = []
    n = 0
    for i in range(strips):
        # Row centre, relative to the plate centre.
        rz = -half + (i + 0.5) * step
        # Where this row is blocked, as (from, to) spans in y.
        blocked = []
        for oy, oz, r in openings:
            dz = abs(rz - oz)
            if dz >= r:
                continue
            # Half-width of the circle at this height.
            hw = math.sqrt(r * r - dz * dz)
            blocked.append((oy - hw, oy + hw))
        blocked.sort()

        # Walk the row left to right, emitting the gaps between blocked spans.
        cursor = -half
        spans = []
        for lo, hi in blocked:
            if lo > cursor:
                spans.append((cursor, min(lo, half)))
            cursor = max(cursor, hi)
        if cursor < half:
            spans.append((cursor, half))

        for lo, hi in spans:
            width = hi - lo
            if width < 1e-4:
                continue
            parts.append(_box_link(
                f'{prefix}_c{n}', thickness, width, step,
                (0.8, 0.8, 0.8), 0.05,
                f'0 {(lo + hi) / 2.0:.6g} {cz + rz:.6g} 0 0 0',
                mat=material((0.8, 0.8, 0.8)),
                visible=False))
            n += 1
    return parts


def robosub_torpedo_board(spec, role="survey_repair", model_name=None):
    """Task 4 -- a 0.6 m printed board on two PVC legs, with real openings.

    Built wrong the first time as an H-shaped frame with two square gaps and
    the wrong overall size. The real board is a full 2 ft square standing on
    legs, printed with all four role images, and its openings are CIRCLES.

    THE OPENINGS ARE NOW PHYSICALLY OPEN. The printed face carries the artwork
    and does not collide; the collision is a strip-tiled plate with the two
    circles cut out (see `_plate_with_holes`). That is what makes the
    rulebook's own distinction measurable -- "A torpedo must pass through the
    opening for full points. Partial points are awarded if the torpedo touches
    the board without passing through" (p. 36).
    """
    cfg = spec["props"]["torpedo_board"]
    size, th = cfg["size"], cfg["thickness"]
    r_pvc = spec["pvc_one_in"]
    legs = cfg.get("leg_height", 0.55)
    cz = legs + size / 2.0
    parts = []

    # THE FRAME. The board is not a bare plate on two sticks: the CAD shows a
    # tube rectangle around the panel, standing on two legs. Ours
    # had no frame at all and its legs sat flush with the panel edge, doubling
    # as its side edges -- which is why the render read as a sheet of card
    # balanced on two poles rather than a built structure.
    #
    # Uprights run the full height and carry the rails; the panel is inset
    # inside them.
    frame_mat = pvc_material(GREY_PVC, competition="robosub")
    for i, y in ((0, -size / 2.0), (1, size / 2.0)):
        parts.append(_cylinder_link(
            f"leg_{i}", r_pvc, legs + size, GREY_PVC, 2.0,
            f"0 {y:.6g} {(legs + size) / 2.0:.6g} 0 0 0",
            mat=frame_mat))

    # Top and bottom rails, closing the rectangle.
    for tag, z in (("top", legs + size), ("bottom", legs)):
        parts.append(_cylinder_link(
            f"rail_{tag}", r_pvc, size, GREY_PVC, 1.2,
            f"0 0 {z:.6g} 1.5708 0 0", mat=frame_mat))

    # NO REAR KICKSTAND. The board stands on its TWO legs, which is what the
    # task slide and the pool photographs show. Two raking braces and their foot
    # pads were added to stop it "reading as unsupported", and they read as four
    # legs instead -- two upright, two slanted -- which is not the prop. Last
    # round fixed the braces' pitch (`pi - atan2`, they had raked the wrong way);
    # this round deletes the geometry that fix was correcting, and `brace_rake`
    # goes with it rather than staying as a spec key nothing reads.

    # The printed face. One thin plate carrying the whole artwork, so the
    # openings, their red rims and the two role images stay in register --
    # they all come from torpedo_openings() now, not from two hand-written
    # lists that had already drifted apart.
    #
    # NO COLLISION, and NO VISUAL on the strips below: the strip tiling is the
    # physical board, this plate is the visible one. Both drawing gave two
    # co-planar surfaces at the same thickness, which z-fought into the streaky
    # grey mess that hid the artwork entirely.
    #
    # AND IT IS A MESH, NOT A BOX, because a box cannot have a hole in it. The
    # openings used to be PAINTED on a solid face: dark disks on an RGB texture
    # with no alpha. They never parallaxed, never showed water or a prop behind
    # them, never responded to fog. A detector trained on that learns a painted
    # bullseye rather than a hole, which is exactly the transfer failure this
    # simulator exists to catch. The mesh is generated by gen_prop_meshes.py
    # from torpedo_openings() -- the same list the collision strips below are
    # tiled from, so the hole you can see stays the hole you can shoot through.
    parts.append(link("board", "\n".join([
        inertial(3.0, box_inertia(3.0, th, size, size)),
        visual("board_visual",
               # The <geometry> wrapper is not optional: without it SDF logs
               # "XML Element[mesh] ... not defined in SDF", copies it through
               # as an unknown child, and the renderer then fails the visual
               # outright -- the board simply is not drawn, legs floating on
               # their own, with the reason 60 lines up the log.
               f'<geometry><mesh>'
               f'<uri>model://robosub_meshes/meshes/torpedo_plate.obj</uri>'
               f'</mesh></geometry>',
               # emissive STAYS 0.10. Raising it to 0.22 was tried and
               # REJECTED on measurement: the panel is a vertical face in a
               # scene lit from above, so a 0.93 white texture reaches the
               # camera at 0.51 and the lift did brighten it (130 -> 158, +22%)
               # -- but emissive is added as a GREY, so the artwork's redness
               # fell 44 -> 32 (-27%) in the same frame. That is precisely the
               # desaturation this round removed from pvc_material, and adding
               # it back here to make white look whiter would be incoherent.
               # Grazing-lit is what a vertical board in a pool IS; the board
               # is already the brightest object in frame.
               textured_material(f"torpedo_panel_{role}.png", tint=1.0,
                                 specular=0.06, roughness=0.6, emissive=0.10,
                                 competition="robosub")),
    ]), f"0 0 {cz:.6g} 0 0 0"))

    # Openings, in plate coordinates. Placed to match the printed artwork:
    # the larger opening upper-left, the smaller lower-right.
    openings = torpedo_openings(spec)
    parts.extend(_plate_with_holes("plate", size, th, openings, cz))

    # NO RIM GEOMETRY. A first attempt put a thin cylinder behind each opening
    # to give the edge some depth; rendered, they were solid discs that PLUGGED
    # the holes the mesh had just cut -- four painted dots again, by a different
    # route. The printed red annulus in the texture is the rim, and the mesh's
    # own wall gives the edge its thickness.

    # "The 'far' distance is denoted by the horizontal bars at the bottom of
    # the board" (p. 36). Two bars, because the spec carries two standoffs.
    # NO STANDOFF MARKER. There used to be two red bars hanging in open water
    # under the board, on nothing. The scored standoff is a HORIZONTAL firing
    # distance -- a bar's height cannot encode it, so they marked nothing and
    # in a render just looked like a mistake.
    #
    # They are not replaced. A floor marker at the scored range would be a
    # training aid, and this prop feeds the vision datasets: painting a stripe
    # on the floor that will not be at the competition teaches the detector
    # something false. The range belongs on the operator's readout, which is
    # where the scoring dashboard now puts it -- not in the water.

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
            # Scored by surfacing INSIDE it, not by contact.
            f"side_{i}", pr, side, colour, 1.0,
            f"{apothem * math.cos(ang):.6g} {apothem * math.sin(ang):.6g} 0 "
            f"0 1.5708 {ang + math.pi / 2.0:.6g}",
            collide=False, mat=pvc_material(colour, competition="robosub")))
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
        mat=pvc_material(colour, competition="robosub"))]

    # A rim around the top so items cannot slide off.
    for tag, sx, sy, dx, dy in (
        ("xp", r * 2, size, half, 0.0), ("xn", r * 2, size, -half, 0.0),
        ("yp", size, r * 2, 0.0, half), ("yn", size, r * 2, 0.0, -half),
    ):
        parts.append(_box_link(
            f"rim_{tag}", sx, sy, 0.03, colour, 0.2,
            f"{dx:.6g} {dy:.6g} {h + 0.02:.6g} 0 0 0",
            mat=pvc_material(colour, competition="robosub")))

    for i, (dx, dy) in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1))):
        parts.append(_cylinder_link(
            f"leg_{i}", r, h, colour, 0.5,
            f"{dx * half:.6g} {dy * half:.6g} {h / 2.0:.6g} 0 0 0",
            collide=False, mat=pvc_material(colour, competition="robosub")))
    # Foot rails, as the CAD shows -- and they stop the table tipping.
    for tag, y in (("yp", half), ("yn", -half)):
        parts.append(_cylinder_link(
            f"foot_{tag}", r, size, colour, 0.4,
            f"0 {y:.6g} 0.03 0 1.5708 0", collide=False,
            mat=pvc_material(colour, competition="robosub")))
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
            f"{cx:.6g} 0 {hgt / 2.0:.6g} 0 0 0", mat=pvc_material(WHITE, competition="robosub")))
        parts.append(_cylinder_link(
            f"foot_{i}", r, w * 2.2, WHITE, 0.4,
            f"{cx:.6g} 0 0.02 1.5708 0 0", mat=pvc_material(WHITE, competition="robosub")))
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
        f"0 0 {cfg['height'] / 2.0:.6g} 0 0 0",
        mat=plastic_material("albedo_plastic.png", competition="robosub",
                             tint=cfg["colour"])))


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
    item_mat = plastic_material("albedo_plastic.png", competition="robosub",
                                tint=colour)
    if kind in ("bolt", "plug"):
        r, h = cfg["jar_diameter"] / 2.0, cfg["jar_height"]
        mass = math.pi * r * r * h * density
        body = _cylinder_link("body", r, h, colour, mass,
                              f"0 0 {h / 2.0:.6g} 0 0 0", mat=item_mat)
    else:
        sz, h = cfg["box_size"], cfg["box_height"]
        mass = sz * sz * h * density
        body = _box_link("body", sz, sz, h, colour, mass,
                         f"0 0 {h / 2.0:.6g} 0 0 0", mat=item_mat)
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
        "dynamic": True,
        # Ball sits IN the cup: cup top (height + 10 mm) minus the depth the
        # ball settles into it. Sitting it a full radius above the pole put it
        # balanced on the rim instead of nested.
        # Ball centre sits a FULL RADIUS above the cup floor's top face, which
        # is where a sphere resting on a surface actually is.
        #
        # Two previous versions used a fraction of the diameter (0.34, then
        # 0.30) on the theory that the ball "nests into" the rim. It does not:
        # the rim radius is 0.92 of the ball's, so the ball rests on the rim's
        # inner edge at essentially its own radius above the floor. Those
        # fractions put the sphere's centre 8.5 mm BELOW where it belongs,
        # i.e. interpenetrating the cup floor -- and the solver resolves an
        # overlap by ejecting the lighter body. The ball was being launched
        # off the flare on the first physics step, not falling off it.
        #
        # This is why the earlier "held for 75 s" check passed while the balls
        # were on the floor: it measured whether z CHANGED, not whether z was
        # RIGHT. A ball already at rest on the pool bottom is perfectly stable.
        "ball_on": lambda s: s["props"]["bump_flare"]["height"]
        + 0.006
        + s["props"]["golf_ball"]["diameter"] / 2.0,
    },
    "sauvc_flare_yellow": {
        "build": lambda s: bump_flare(s, "yellow"),
        "anchor": ANCHOR_FLOOR,
        "dynamic": True,
        # Ball sits IN the cup: cup top (height + 10 mm) minus the depth the
        # ball settles into it. Sitting it a full radius above the pole put it
        # balanced on the rim instead of nested.
        # Ball centre sits a FULL RADIUS above the cup floor's top face, which
        # is where a sphere resting on a surface actually is.
        #
        # Two previous versions used a fraction of the diameter (0.34, then
        # 0.30) on the theory that the ball "nests into" the rim. It does not:
        # the rim radius is 0.92 of the ball's, so the ball rests on the rim's
        # inner edge at essentially its own radius above the floor. Those
        # fractions put the sphere's centre 8.5 mm BELOW where it belongs,
        # i.e. interpenetrating the cup floor -- and the solver resolves an
        # overlap by ejecting the lighter body. The ball was being launched
        # off the flare on the first physics step, not falling off it.
        #
        # This is why the earlier "held for 75 s" check passed while the balls
        # were on the floor: it measured whether z CHANGED, not whether z was
        # RIGHT. A ball already at rest on the pool bottom is perfectly stable.
        "ball_on": lambda s: s["props"]["bump_flare"]["height"]
        + 0.006
        + s["props"]["golf_ball"]["diameter"] / 2.0,
    },
    "sauvc_flare_blue": {
        "build": lambda s: bump_flare(s, "blue"),
        "anchor": ANCHOR_FLOOR,
        "dynamic": True,
        # Ball sits IN the cup: cup top (height + 10 mm) minus the depth the
        # ball settles into it. Sitting it a full radius above the pole put it
        # balanced on the rim instead of nested.
        # Ball centre sits a FULL RADIUS above the cup floor's top face, which
        # is where a sphere resting on a surface actually is.
        #
        # Two previous versions used a fraction of the diameter (0.34, then
        # 0.30) on the theory that the ball "nests into" the rim. It does not:
        # the rim radius is 0.92 of the ball's, so the ball rests on the rim's
        # inner edge at essentially its own radius above the floor. Those
        # fractions put the sphere's centre 8.5 mm BELOW where it belongs,
        # i.e. interpenetrating the cup floor -- and the solver resolves an
        # overlap by ejecting the lighter body. The ball was being launched
        # off the flare on the first physics step, not falling off it.
        #
        # This is why the earlier "held for 75 s" check passed while the balls
        # were on the floor: it measured whether z CHANGED, not whether z was
        # RIGHT. A ball already at rest on the pool bottom is perfectly stable.
        "ball_on": lambda s: s["props"]["bump_flare"]["height"]
        + 0.006
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
        "anchor": ANCHOR_SURFACE,
        # Dynamic, but the frame is pinned to the world by a fixed joint -- the
        # role boards and the divider are what swing. Both halves of this flag
        # must agree with <static> in the model or build_props refuses.
        "dynamic": True,
    },
    "robosub_slalom": {
        "build": robosub_slalom,
        "anchor": ANCHOR_FLOOR,
        "dynamic": True,
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
    # A SUBDIVIDED, DOUBLE-SIDED GRID -- and NOT animated. Round 23 got the
    # geometry prerequisite in and could not get ShaderParam to drive it; the
    # measurements are in .context/PHYSICS.md so the next attempt starts from
    # them rather than from scratch. In short: the GLSL COMPILES (a deliberately
    # broken one aborts the server, exit -6, and logs the compile failure -- so
    # the old "a failing shader renders nothing and logs nothing" line is wrong
    # for gz-sim 8), the plugin DOES replace the material (100 % of pixels
    # differ from this one), and every <param> value arrives as ZERO: changing
    # the fragment shader wholesale, and changing `tau` by three orders of
    # magnitude, both left the render byte-identical at mean 97.667.
    #
    # The grid is kept because it is verified equivalent to the box it replaced
    # (mean 111.452 vs 111.424 from a probe camera under it) and because it is
    # the thing that was missing every previous round: a <box> is 8 vertices and
    # a vertex-displacement shader has nothing to displace.
    #
    # `gerstner` remains the unbounded Fuel ocean and remains the wrong default
    # for a pool.
    #
    # NO COLLISION, on purpose: the vehicle surfaces through this sheet, and a
    # collision here would make `surface()` push against a lid.
    #
    # cast_shadows off as well -- a 25x16 m shadow caster directly above the
    # whole arena darkens every prop and is the one thing that would undo the
    # emissive lift the props rely on to stay visible through fog.
    if water_surface == "plane":
        parts.append(
        link(
            "water_surface",
            "\n".join([
                inertial(1.0, (1.0, 1.0, 1.0)),
                visual(
                    "water_surface_visual",
                    f'<geometry><mesh><uri>model://robosub_meshes/meshes/'
                    f'water_{comp}.obj</uri></mesh></geometry>',
                    # Blue-tinted rather than near-white: from underneath, a
                    # water surface is a dim mirror of the pool, not a window.
                    # This material is what renders if the shader fails to
                    # compile is caught and the plugin removed; while the
                    # plugin is live the shader owns the colour.
                    textured_material(
                        "water_surface.png", tint=(0.42, 0.60, 0.68),
                        specular=0.35, roughness=0.15, emissive=0.10,
                        competition=comp),
                    "0 0 0 0 0 0",
                    cast_shadows=False,
                    # KEPT even though the fragment shader writes its own alpha:
                    # <transparency> is what puts the visual in Ogre's
                    # transparent render queue in the first place. At 0.62 the
                    # sheet was effectively not there and the pool read as
                    # EMPTY, which is what "every course looked like it had no
                    # water" meant when the default was flipped to the
                    # unbounded ocean to hide it.
                    transparency=0.18,
                ),
            ]),
        )
        )

    return model(f"{comp}_pool", "\n".join(parts))
