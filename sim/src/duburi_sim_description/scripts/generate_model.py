#!/usr/bin/env python3

"""Generate a Gazebo model.sdf from a template and a YAML config.

Substitutes strings of the form "@foo" in a `model.sdf.in` template with values
computed from a YAML configuration file.

Derived from bluerov2_gz by Clyde McQueen (MIT). Extended here with camera
placement, odometry publishing and ArduPilot FDM endpoint configuration. See
ATTRIBUTION.md.

Required config fields:
    - model_name: Name of the model. Also determines the thruster command topics.
    - mass: Vehicle mass in kg.
    - control_method: 0 for thrust, 1 for angular velocity.
    - bounding_box: Used to derive the buoyancy collision box and, if drag is not
        given, the quadratic drag coefficients.
    - center_of_mass / center_of_volume: The offset between these two is what
        gives the vehicle its righting moment.
    - buoyancy_adjustment: Mass added to `mass` to get displaced mass. Positive
        makes the vehicle buoyant.
    - thrusters: Thruster positions, in the order they are declared in the
        template. Orientation lives in the template, not here.

Optional config fields:
    - inertia, drag.linear, drag.quadratic, default_current
    - added_mass: positive magnitudes, emitted as //inertial/fluid_added_mass.
      See ADDED_MASS_KEYS for why this is not the plugin's <xDotU> family.
    - fluid_density (default 1000.0). This MUST match the world's Buoyancy
      <default_density>, otherwise the displaced mass the generator computes is
      not the displaced mass the simulator applies.
    - cameras: front/bottom placement and shared image properties.
    - ardupilot: fdm_addr / fdm_port_in.
    - odom_publish_frequency

Typical usage:
    scripts/generate_model.py \
        models/duburi_heavy/model.sdf.in \
        models/duburi_heavy/model.sdf \
        models/duburi_heavy/configs.yaml
"""

import math
import re
from argparse import ArgumentParser

import yaml

# SDF 1.9 supports degrees="true"; provide some nice vars for earlier versions
d180 = math.pi
d90 = d180 / 2
d45 = d90 / 2
d30 = d90 / 3
d135 = d90 + d45


def thrust_to_ang_vel(
    thrust: float,
    propeller_diameter: float,
    thrust_coefficient: float,
    fluid_density: float,
) -> float:
    """Convert thrust to angular velocity.

    Defined by Fossen in "Guidance and Control of Ocean Vehicles" on p. 246.
    """
    assert thrust >= 0
    assert thrust_coefficient >= 0
    return math.sqrt(
        thrust / (fluid_density * thrust_coefficient * pow(propeller_diameter, 4))
    )


# Added mass is emitted into //inertial/fluid_added_mass rather than into the
# Hydrodynamics plugin's <xDotU> family, and the two are not interchangeable.
#
# The plugin applies added mass explicitly, as Ma * a where a is a finite
# difference of the previous step's velocity. That feedback loop diverges
# whenever an added-mass term exceeds the rigid-body mass on the same axis, and
# for the BlueROV2 Heavy heave adds 18.7 kg to a 13.5 kg vehicle. The failure is
# not subtle: the pose blows up within a few hundred steps and ODE aborts the
# process on an AABB assertion.
#
# SDFormat >= 1.10 with the dartsim engine folds fluid_added_mass into the
# link's spatial inertia instead, so it is solved implicitly and is
# unconditionally stable. DART also derives the added-mass Coriolis terms from
# that same inertia, so nothing is lost by leaving the plugin's tags at zero.
#
# Consequence for units and signs: these are magnitudes of a positive
# semi-definite inertia matrix, so they are positive. The plugin's tags are
# negative. Do not copy values between the two.
ADDED_MASS_KEYS = ("xDotU", "yDotV", "zDotW", "kDotP", "mDotQ", "nDotR")

# Defaults for the optional camera block. 640x480 is not a free choice: it is
# what the sim_front and sim_bottom profiles in duburi_vision/config/cameras.yaml
# declare, and duburi_ws is the consumer we must not force to change.
CAMERA_DEFAULTS = {
    "width": 640,
    "height": 480,
    "update_rate": 30.0,
    # 1.396 rad ~= 80 deg, close to the Blue Robotics low-light USB cameras.
    "hfov": 1.396,
    # Stands in for water turbidity rather than a real lens limit.
    "far_clip": 30.0,
    "noise_stddev": 0.007,
    "front": {"x": 0.2, "y": 0.0, "z": 0.0},
    "bottom": {"x": 0.0, "y": 0.0, "z": -0.1},
}

# Doppler Velocity Log. Modelled on the Nortek Nucleus1000 carried by the real
# hull: 4-beam janus array, ~8 Hz. The reference world ships 1 Hz, which is too
# slow to close a distance loop on -- the control layer samples position per
# tick, so a 1 Hz DVL would quantise every move into ~0.5 m steps.
DVL_DEFAULTS = {
    "x": 0.0,
    "y": 0.0,
    # Just below the hull, alongside the bottom camera.
    "z": -0.10,
    "update_rate": 8.0,
    # +/- 0.4 cm/s within 2 stddevs, the reference figure.
    "noise_stddev": 0.002,
    "resolution": 0.01,
    # The SAUVC pool floor is 1.6 m down, so max range is never the binding
    # constraint; minimum_range is, because the hull flies close to the bottom.
    "maximum_range": 50.0,
    "minimum_range": 0.1,
    # OFF, and this is the single most expensive flag in the model.
    #
    # It draws the four beams as Gazebo debug visuals, which run on the SAME
    # Ogre2 render thread the cameras use. Measured 2026-08-28, SAUVC final
    # course, headless, identical hardware:
    #
    #     visualize true  :  2.83 Hz,  jitter (stdev) 435 ms,  max gap 1126 ms
    #     visualize false : 12.75 Hz,  jitter (stdev)   6 ms,  max gap  104 ms
    #
    # 4.5x the frame rate and 76x less jitter. The jitter is the part that
    # matters: it is what the operator sees as laggy teleop video in the web
    # lab and as juddery recorded dataset video. Headless it is pure waste --
    # it cannot draw, so it only logs "Failed to render beam markers".
    #
    # Nothing is lost by default: duburi_sim_bridge/dvl_bridge.py publishes an
    # equivalent MarkerArray, so RViz shows the beams either way. Set
    # dvl.visualize_beams: true in configs.yaml and rebuild only if you
    # specifically want them in the Gazebo GUI, and expect the frame rate above.
    "visualize_beams": False,
}


class ModelParams:
    """Wrapper for the parameters needed to generate an SDF for use by Gazebo."""

    def __init__(
        self,
        model_name: str,
        mass: float,
        fluid_density: float,
        collision: tuple,
        center_of_mass: tuple,
        center_of_volume: tuple,
        inertia: tuple,
        linear_drag: tuple,
        quadratic_drag: tuple,
        added_mass: tuple,
        default_current: tuple,
        thrusters: list,
        use_angvel_cmd: int,
        cameras: dict,
        dvl: dict = None,
        range_cameras: bool = False,
        fdm_addr: str = "127.0.0.1",
        fdm_port_in: int = 9002,
        odom_publish_frequency: float = 50.0,
        propeller_size: str = "0.1 0.02 0.01",
        propeller_mass: float = 0.002,
        propeller_inertia: tuple = (0.001, 0.001, 0.001),
        propeller_diameter: float = 0.1,
        thrust_coefficient: float = 0.02,
        max_thrust: float = 50,
        servo_range: tuple = (1100, 1900),
        control_offset: float = -0.5,
    ) -> None:
        # Quoted for use as an XML attribute value; the plain form is for
        # element text such as the Thruster <namespace>.
        self.model_name = f'"{model_name}"'
        self.model_name_plain = model_name

        self.mass = mass
        self.fluid_density = fluid_density

        # The collision box is used by the BuoyancyPlugin
        self.collision_x = collision[0]
        self.collision_y = collision[1]
        self.collision_z = collision[2]

        self.center_of_mass_x = center_of_mass[0]
        self.center_of_mass_y = center_of_mass[1]
        self.center_of_mass_z = center_of_mass[2]

        self.center_of_volume_x = center_of_volume[0]
        self.center_of_volume_y = center_of_volume[1]
        self.center_of_volume_z = center_of_volume[2]

        self.ixx = inertia[0]
        self.iyy = inertia[1]
        self.izz = inertia[2]

        self.xU, self.yV, self.zW, self.kP, self.mQ, self.nR = linear_drag

        (
            self.xUabsU,
            self.yVabsV,
            self.zWabsW,
            self.kPabsP,
            self.mQabsQ,
            self.nRabsR,
        ) = quadratic_drag

        # Added mass goes into //inertial/fluid_added_mass, not into the
        # Hydrodynamics plugin. See the note on ADDED_MASS_KEYS below.
        (
            self.am_xx,
            self.am_yy,
            self.am_zz,
            self.am_pp,
            self.am_qq,
            self.am_rr,
        ) = added_mass

        self.default_current_x = default_current[0]
        self.default_current_y = default_current[1]
        self.default_current_z = default_current[2]

        self.use_angvel_cmd = bool(use_angvel_cmd)

        # Camera parameters
        self.camera_width = cameras["width"]
        self.camera_height = cameras["height"]
        self.camera_update_rate = cameras["update_rate"]
        self.camera_hfov = cameras["hfov"]
        self.camera_far_clip = cameras["far_clip"]
        self.camera_noise_stddev = cameras["noise_stddev"]
        self.front_camera_x = cameras["front"]["x"]
        self.front_camera_y = cameras["front"]["y"]
        self.front_camera_z = cameras["front"]["z"]
        self.bottom_camera_x = cameras["bottom"]["x"]
        self.bottom_camera_y = cameras["bottom"]["y"]
        self.bottom_camera_z = cameras["bottom"]["z"]

        dvl = dict(DVL_DEFAULTS) if dvl is None else dvl
        self.dvl_x = dvl["x"]
        self.dvl_y = dvl["y"]
        self.dvl_z = dvl["z"]
        self.dvl_update_rate = dvl["update_rate"]
        self.dvl_noise_stddev = dvl["noise_stddev"]
        self.dvl_resolution = dvl["resolution"]
        self.dvl_maximum_range = dvl["maximum_range"]
        self.dvl_minimum_range = dvl["minimum_range"]
        # Box sensors cost a render pass on the SAME Ogre2 thread the cameras
        # use -- the thread that took the sim from 12.75 Hz to 2.83 Hz when the
        # DVL drew beam visuals. Measured cost is small (see ROBOSUB doc), so
        # they stay on; this is the knob if that ever changes.
        # Propeller wake. gz-sim's Thruster defaults are alpha_1=1, alpha_2=0,
        # i.e. thrust independent of speed. 0.2 is the standard wake fraction
        # for an open propeller in undisturbed flow; alpha_2 negative is what
        # makes thrust fall as the advance ratio rises.
        # PROPELLER WAKE. Ct = alpha_1 + alpha_2 * J, evaluated each step.
        #
        # alpha_1 is the STATIC thrust coefficient -- the same 0.02 that used to
        # be <thrust_coefficient> -- so bollard-pull behaviour is unchanged and
        # only the speed falloff is new. alpha_1 = 1.0 (gz's default) would have
        # changed Ct by 50x and quietly rescaled every propeller speed.
        #
        # PROVENANCE, since the T200 numbers are measured and these are not:
        # Blue Robotics publish bollard-pull only -- thrust with the vehicle
        # held still -- so there is no T200 Ct-vs-J curve to fit. alpha_2 is
        # from open-propeller theory, where Ct falls roughly linearly with
        # advance ratio and vanishes near J ~ 1.5-2 for a low-pitch prop.
        # -0.012 puts the zero at J = 1.67. Marked as MODELLED, not measured,
        # in PHYSICS.md; a real Ct-J curve would replace it.
        #
        # wake_fraction 0.2 is the standard figure for an open propeller in
        # undisturbed flow, which is what a thruster on an open frame sees.
        self.wake_fraction = 0.2
        self.alpha_1 = 0.02
        self.alpha_2 = -0.012

        self.boxes_always_on = 1
        # Range images cost a render pass each, and unlike the bounding-box
        # cameras -- which measured free -- these are EXPENSIVE: enabling both
        # took the cameras from 12 Hz to 4 Hz on the SAUVC final course. That
        # is the difference between a usable dataset capture and a slideshow.
        #
        # So they are OFF by default. Per-pixel attenuation is the most
        # physically faithful part of the image pipeline and it is worth having
        # for a perception experiment, but not at a 3x frame-rate cost on every
        # ordinary run. `range_cameras: true` in configs.yaml turns them on;
        # underwater_fx falls back to uniform attenuation when no range image
        # arrives, so the sim is correct either way, just less faithful.
        self.range_always_on = int(bool(range_cameras))

        self.dvl_visualize_beams = str(
            dvl.get("visualize_beams", DVL_DEFAULTS["visualize_beams"])
        ).lower()

        # ArduPilotPlugin endpoint
        self.fdm_addr = fdm_addr
        self.fdm_port_in = fdm_port_in

        self.odom_publish_frequency = odom_publish_frequency

        # Propeller link parameters
        self.propeller_size = propeller_size
        self.propeller_mass = propeller_mass
        self.propeller_ixx = propeller_inertia[0]
        self.propeller_iyy = propeller_inertia[1]
        self.propeller_izz = propeller_inertia[2]

        # ThrusterPlugin parameters
        self.propeller_diameter = propeller_diameter
        self.thrust_coefficient = thrust_coefficient

        # ArduPilotPlugin control parameters
        self.servo_min = servo_range[0]
        self.servo_max = servo_range[1]
        self.control_offset = control_offset

        if use_angvel_cmd:
            ang_vel = thrust_to_ang_vel(
                max_thrust, propeller_diameter, thrust_coefficient, fluid_density
            )
            self.cw_control_multiplier = -ang_vel * 2
            self.ccw_control_multiplier = ang_vel * 2
        else:
            # PWM is normalised to [0, 1], then (normalised + offset) * multiplier
            # gives newtons. With offset -0.5 and multiplier 2 * max_thrust,
            # PWM 1100 -> -max_thrust, 1500 -> 0, 1900 -> +max_thrust.
            self.cw_control_multiplier = max_thrust * 2
            self.ccw_control_multiplier = max_thrust * 2

        for i, thruster in enumerate(thrusters):
            thruster_num = i + 1

            setattr(self, f"thruster{thruster_num}_x", thruster[0])
            setattr(self, f"thruster{thruster_num}_y", thruster[1])
            setattr(self, f"thruster{thruster_num}_z", thruster[2])

            topic = f"/model/{model_name}/joint/thruster{thruster_num}_joint/cmd_"
            topic += "vel" if use_angvel_cmd else "thrust"

            setattr(self, f"thruster{thruster_num}_topic", topic)
            # ArduPilotPlugin publishes the raw AFFINE thrust here; the
            # t200_curve node reshapes it against the real T200 curve and
            # republishes on the topic above, which the Thruster plugin reads.
            setattr(self, f"linear_thruster{thruster_num}_topic", topic + "_linear")


def _merge_cameras(config: dict) -> dict:
    """Overlay the config's optional `cameras` block onto CAMERA_DEFAULTS."""
    cameras = {k: v for k, v in CAMERA_DEFAULTS.items() if k not in ("front", "bottom")}
    cameras["front"] = dict(CAMERA_DEFAULTS["front"])
    cameras["bottom"] = dict(CAMERA_DEFAULTS["bottom"])

    override = config.get("cameras") or {}
    for key, value in override.items():
        if key in ("front", "bottom"):
            cameras[key].update(value)
        else:
            cameras[key] = value
    return cameras


def _merge_dvl(config: dict) -> dict:
    """Overlay the config's optional `dvl` block onto DVL_DEFAULTS."""
    dvl = dict(DVL_DEFAULTS)
    dvl.update(config.get("dvl") or {})
    return dvl


def get_model_params_from_config(config_path: str) -> ModelParams:
    """Build a ModelParams object from a YAML config file."""
    with open(config_path) as config_file:
        config = yaml.safe_load(config_file)

    mass = config["mass"]
    fluid_density = config.get("fluid_density", 1000.0)

    bounding_x = config["bounding_box"]["x"]
    bounding_y = config["bounding_box"]["y"]
    bounding_z = config["bounding_box"]["z"]

    displaced_mass = mass + config["buoyancy_adjustment"]

    # The collision box is deliberately not the hull. Its z is chosen so that
    # box volume * fluid_density == displaced_mass.
    collision = (
        bounding_x,
        bounding_y,
        displaced_mass / (bounding_x * bounding_y * fluid_density),
    )

    try:
        inertia = (
            config["inertia"]["ixx"],
            config["inertia"]["iyy"],
            config["inertia"]["izz"],
        )
    except KeyError:
        ixx = mass / 12 * (collision[1] ** 2 + collision[2] ** 2)
        iyy = mass / 12 * (collision[0] ** 2 + collision[2] ** 2)
        izz = mass / 12 * (collision[0] ** 2 + collision[1] ** 2)
        inertia = (ixx, iyy, izz)

    try:
        linear_drag = (
            config["drag"]["linear"]["xU"],
            config["drag"]["linear"]["yV"],
            config["drag"]["linear"]["zW"],
            config["drag"]["linear"]["kP"],
            config["drag"]["linear"]["mQ"],
            config["drag"]["linear"]["nR"],
        )
    except (KeyError, TypeError):
        linear_drag = (0, 0, 0, 0, 0, 0)

    try:
        quadratic_drag = (
            config["drag"]["quadratic"]["xUabsU"],
            config["drag"]["quadratic"]["yVabsV"],
            config["drag"]["quadratic"]["zWabsW"],
            config["drag"]["quadratic"]["kPabsP"],
            config["drag"]["quadratic"]["mQabsQ"],
            config["drag"]["quadratic"]["nRabsR"],
        )
    except (KeyError, TypeError):
        # Flat-plate approximation, -0.5 * area * Cd * rho, when nothing better
        # is available.
        quadratic_drag = (
            -0.5 * bounding_y * bounding_z * 0.8 * fluid_density,
            -0.5 * bounding_x * bounding_z * 0.95 * fluid_density,
            -0.5 * bounding_x * bounding_y * 0.95 * fluid_density,
            -0.5 * 0.008 * fluid_density,
            -0.5 * 0.008 * fluid_density,
            -0.5 * 0.008 * fluid_density,
        )

    try:
        added_mass = tuple(config["added_mass"][key] for key in ADDED_MASS_KEYS)
    except (KeyError, TypeError):
        added_mass = (0, 0, 0, 0, 0, 0)

    negative = [k for k, v in zip(ADDED_MASS_KEYS, added_mass) if v < 0]
    if negative:
        raise ValueError(
            f"added_mass entries must be positive magnitudes, got negative "
            f"{', '.join(negative)}. Unlike the Hydrodynamics plugin's own "
            f"xDotU-style tags, //inertial/fluid_added_mass is a real inertia "
            f"matrix and must be positive semi-definite."
        )

    try:
        current = (
            config["default_current"]["x"],
            config["default_current"]["y"],
            config["default_current"]["z"],
        )
    except (KeyError, TypeError):
        current = (0, 0, 0)

    ardupilot = config.get("ardupilot") or {}

    return ModelParams(
        config["model_name"],
        mass,
        fluid_density,
        collision,
        (
            config["center_of_mass"]["x"],
            config["center_of_mass"]["y"],
            config["center_of_mass"]["z"],
        ),
        (
            config["center_of_volume"]["x"],
            config["center_of_volume"]["y"],
            config["center_of_volume"]["z"],
        ),
        inertia,
        linear_drag,
        quadratic_drag,
        added_mass,
        current,
        [
            (thruster["x"], thruster["y"], thruster["z"])
            for thruster in config["thrusters"]
        ],
        config["control_method"],
        cameras=_merge_cameras(config),
        dvl=_merge_dvl(config),
        range_cameras=config.get("range_cameras", False),
        fdm_addr=ardupilot.get("fdm_addr", "127.0.0.1"),
        fdm_port_in=ardupilot.get("fdm_port_in", 9002),
        odom_publish_frequency=config.get("odom_publish_frequency", 50.0),
    )


def generate_model(input_path: str, output_path: str, config_path: str) -> None:
    """Render the template at input_path into output_path using config_path."""
    params = vars(get_model_params_from_config(config_path)) | globals()

    with open(input_path) as f:
        s = f.read()

    # Strip the range-camera sensors entirely unless they are asked for.
    # Disabling them with always_on=0 does not work -- Gazebo creates and
    # renders the sensor anyway -- and each one costs a full render pass on the
    # thread the colour cameras share.
    if not params.get("range_always_on"):
        s = re.sub(r"\n +<!-- RANGE image.*?</sensor>\n", "\n", s, flags=re.S)

    pattern = re.compile(r"@(\w+)")
    missing = sorted(
        {m.group(1) for m in pattern.finditer(s)} - set(params)
    )
    if missing:
        raise KeyError(
            f"template {input_path} references unknown tokens: {', '.join(missing)}"
        )

    def substitute(match):
        value = params[match.group(1)]
        # Round floats so the generated SDF stays readable and diffable.
        return str(round(value, 6) if isinstance(value, float) else value)

    with open(output_path, "w") as f:
        f.write(re.sub(pattern, substitute, s))


if __name__ == "__main__":
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("infile", help="Template SDF to substitute into.")
    parser.add_argument("outfile", help="Output SDF path.")
    parser.add_argument("config", help="YAML configuration to load.")

    args = parser.parse_args()

    generate_model(args.infile, args.outfile, args.config)
    print(f"wrote {args.outfile}")
