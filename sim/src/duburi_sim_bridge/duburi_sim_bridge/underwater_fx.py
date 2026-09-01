#!/usr/bin/env python3

"""Underwater image degradation for vision / OD training.

Subscribes to the drop-in raw camera topics (unchanged contract) and publishes
degraded copies on image_fx. Effects are driven by ROS parameters so the web
lab can change turbidity live without restarting Gazebo.

Algorithm (lightweight UUV-style attenuation):
  - depth-aware RGB channel attenuation (red falls off fastest)
  - turbidity-scaled backscatter haze
  - mild Gaussian blur + additive noise
  - optional vignette
"""

from __future__ import annotations

from functools import lru_cache

import math
import time

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

FRONT_RAW = '/duburi/sim/front_camera/image_raw'
BOTTOM_RAW = '/duburi/sim/bottom_camera/image_raw'
FRONT_RANGE = '/duburi/sim/front_camera/range'
BOTTOM_RANGE = '/duburi/sim/bottom_camera/range'
FRONT_FX = '/duburi/sim/front_camera/image_fx'
BOTTOM_FX = '/duburi/sim/bottom_camera/image_fx'
GROUND_TRUTH = '/duburi/sim/ground_truth'

# Water attenuation coefficients (1/m), roughly Jerlov coastal water.
ATTEN_RGB = np.array([0.45, 0.18, 0.12], dtype=np.float32)
HAZE_RGB = np.array([0.05, 0.22, 0.28], dtype=np.float32)

# Pool water is far clearer than the open ocean ATTEN_RGB describes. See the
# per-pixel block in apply_underwater_fx.
RANGE_ATTEN = 0.22
RANGE_FLOOR = 0.30


PARTICLE_RGB = np.array([0.80, 0.84, 0.80], dtype=np.float32)


def _msg_to_bgr(msg: Image) -> np.ndarray:
    channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1}.get(
        msg.encoding
    )
    if channels is None:
        raise ValueError(f'unsupported encoding {msg.encoding}')
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    expected = msg.height * msg.step
    img = raw[:expected].reshape((msg.height, msg.step))[:, : msg.width * channels]
    img = img.reshape((msg.height, msg.width, channels))
    if msg.encoding == 'rgb8':
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    if msg.encoding == 'rgba8':
        return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
    if msg.encoding == 'bgra8':
        return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    if msg.encoding == 'mono8':
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    return img


def _bgr_to_msg(bgr: np.ndarray, stamp, frame_id: str) -> Image:
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height, msg.width = rgb.shape[:2]
    msg.encoding = 'rgb8'
    msg.is_bigendian = 0
    msg.step = msg.width * 3
    msg.data = rgb.tobytes()
    return msg


_NOISE_BANK: dict = {}
_NOISE_TILES = 8


def _noise_tile(shape: tuple) -> np.ndarray:
    """A unit-variance noise field, drawn from a small precomputed bank.

    `np.random.normal` over a 640x480x3 frame measured 22.0 ms -- on its own
    most of a 30 Hz budget, and doubled because one node degrades both cameras.
    It was the single reason underwater_fx pinned a core, and that CPU comes
    straight out of Gazebo's render loop: the camera published 2.9 Hz with
    0.44 s of jitter, which is what the operator sees as laggy teleop video and
    juddery dataset playback.

    Eight fields are drawn once and one is picked per frame with a random
    circular shift, so successive frames do not share a pattern. For sensor
    noise this is not an approximation worth apologising for -- real sensor
    noise is spatially correlated anyway, and nothing downstream does
    statistics on it. Costs ~0.5 ms.
    """
    bank = _NOISE_BANK.get(shape)
    if bank is None:
        rng = np.random.default_rng(0xD00B)
        bank = [rng.standard_normal(shape, dtype=np.float32)
                for _ in range(_NOISE_TILES)]
        _NOISE_BANK[shape] = bank
    tile = bank[np.random.randint(_NOISE_TILES)]
    return np.roll(tile, (np.random.randint(shape[0]), np.random.randint(shape[1])),
                   axis=(0, 1))


@lru_cache(maxsize=8)
def _vignette_mask(shape: tuple, vignette: float) -> np.ndarray:
    """Radial falloff mask, cached on (shape, strength).

    This used to be rebuilt per frame: an `np.mgrid` pair, two float32
    480x640 arrays, a sqrt and a power, thirty times a second. Measured
    2026-08-28 it was a large part of underwater_fx pinning a full core at
    106 %, which starves Gazebo's render loop -- the camera published 2.9 Hz
    with a 0.44 s standard deviation, and that jitter is what the operator
    sees as laggy teleop video and juddery dataset playback.

    Nothing in the mask depends on the image, only on its size and the
    strength, so there are at most a couple of distinct masks in a run.
    """
    h, w = shape[0], shape[1]
    yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)
    cx, cy = (w - 1) / 2.0, (h - 1) / 2.0
    r = np.sqrt(((xx - cx) / cx) ** 2 + ((yy - cy) / cy) ** 2)
    mask = (1.0 - vignette * np.clip(r, 0.0, 1.0) ** 2).astype(np.float32)
    return np.ascontiguousarray(np.repeat(mask[:, :, None], shape[2], axis=2))



# Sun through water: warm, and biased away from blue because the blue is what
# the water already scatters back at you everywhere else.
GLARE_RGB = np.array([1.00, 0.96, 0.86], dtype=np.float32)

# The vehicle's UNDERWATER horizontal FOV, 57.7 deg. NOT the 80 deg on the
# datasheet: that is the in-air figure and Snell's law narrows it through a flat
# port. Used to turn a pixel into a ray, so a caustic lands where the pixel is
# actually looking rather than 28 % off.
CAMERA_HFOV = 1.0064

# How tightly caustic light concentrates into filaments. 1.0 is the raw surface
# curvature and renders as soft blotches; real caustics are an envelope where
# rays cross, which is far peakier than the surface that makes them.
CAUSTIC_SHARPNESS = 2.6

class SunlightField:
    """Moving caustics and surface glare, the way an outdoor pool actually looks.

    WHY THIS IS HERE AND NOT IN THE RENDERER
    ----------------------------------------
    RoboSub and SAUVC are both run in OUTDOOR pools under direct sun, so the
    wobbling light net on the floor and the glare off the surface are not
    decoration -- they are the dominant appearance of most frames a mission
    consumes, and the sim has never had either.

    Three renderer routes were tried and measured before writing this:

      * `<scene><fog>` -- never reaches camera sensors (18 m -> 3 m left the far
        wall pixel-identical). Already replaced by the LUT above.
      * `particle_emitter` -- never reaches camera sensors (stddev 1.4700 with
        vs 1.4678 without). Already replaced by ParticleField above.
      * `LensFlare` -- DOES attach (gz prints "Lens flare attached to camera
        named" at -v 4) and produces NO measurable output in this scene, with
        the directional sun and with a purpose-added positional light:
        flare off mean 99.8 / p99 118, flare on mean 99.5 / p99 117.

    So this is the third effect in this file that exists because the renderer
    would not do it. That is a pattern, not an accident, and it is the reason
    the answer to "are we at Gazebo\'s ceiling" is "for THESE effects, yes --
    and the CPU path already works".

    CAUSTICS ARE WORLD-ANCHORED, WHICH IS THE WHOLE DIFFICULTY
    ----------------------------------------------------------
    A caustic pattern painted in image space swims with the camera and reads as
    a dirty lens. Real caustics are fixed to the world: the vehicle moves
    THROUGH them. So the pattern is sampled at the world position each pixel is
    looking at, recovered from the range image the depth camera already
    publishes -- which is why `range_cameras` being on by default matters
    beyond attenuation.

    The interference maths is the same one `gen_pool_texture._caustics()` bakes
    into the floor albedo: nine travelling waves, amplitude 1/k^2, Laplacian
    positive part, because light focuses where the surface is concave. Two
    failure modes are already recorded there and inherited here -- too few long
    waves gives parallel diagonal BANDS (a directional prior a detector will
    learn), and 1/k amplitude lets the shortest wave dominate after the k^2
    Laplacian.

    The bake stays: it is the still, always-correct floor pattern. This adds the
    MOTION on top, and unlike the bake it lands on every surface in view -- the
    props and the pool walls included, which the floor-only bake never touched.
    """

    # Nine waves, wavelengths 0.22-0.65 m, as the floor bake uses.
    _K = np.array([2.0 * np.pi / w for w in
                   (0.65, 0.58, 0.49, 0.44, 0.38, 0.33, 0.29, 0.25, 0.22)],
                  dtype=np.float32)

    def __init__(self, seed: int = 11) -> None:
        rng = np.random.default_rng(seed)
        ang = rng.uniform(0.0, 2.0 * np.pi, len(self._K)).astype(np.float32)
        self._dir = np.stack([np.cos(ang), np.sin(ang)], axis=1)
        # Deep-water dispersion: omega = sqrt(g k). Short waves travel faster,
        # which is what stops the pattern moving as one rigid sheet.
        self._omega = np.sqrt(9.81 * self._K).astype(np.float32)
        self._amp = (1.0 / self._K ** 2).astype(np.float32)
        self._amp /= self._amp.sum()
        self._phase = rng.uniform(0.0, 2.0 * np.pi, len(self._K)).astype(np.float32)

    def sample(self, wx, wy, t):
        """Caustic intensity at world (wx, wy) and time t, mean ~1."""
        s = np.zeros_like(wx, dtype=np.float32)
        for i in range(len(self._K)):
            ph = (self._K[i] * (self._dir[i, 0] * wx + self._dir[i, 1] * wy)
                  - self._omega[i] * t + self._phase[i])
            # The LAPLACIAN of the surface, not the surface: light focuses where
            # it is concave, which is -k^2 * amplitude * sin/cos of the phase.
            s += (self._amp[i] * self._K[i]) * np.cos(ph)
        # POSITIVE PART, THEN MEAN-CENTRED -- and both halves matter.
        #
        # Light FOCUSES where the surface is concave and merely thins where it
        # is convex; there is no such thing as negative light. The raw sum runs
        # symmetrically about zero, and used directly it drove the frame gain to
        # -0.082, i.e. a negative image. Clamping at zero gives the bright
        # filaments their characteristic sparse, peaky look.
        #
        # Subtracting the mean afterwards keeps the frame's average brightness
        # unchanged, so turning caustics on does not silently expose the whole
        # image up -- the pattern redistributes light rather than adding it.
        s = np.maximum(s, 0.0)
        # SHARPEN INTO FILAMENTS. The positive part alone is smooth, and the
        # first render showed exactly that: soft blotches rather than the thin
        # bright net a pool actually has. Real caustics are a CAUSTIC -- an
        # envelope where rays cross -- so the intensity is concentrated far more
        # tightly than the surface curvature that produces it. Raising the
        # focused part to a power reproduces that: peaks stay, the broad
        # low-level glow collapses.
        peak = float(s.max())
        if peak > 1e-6:
            s = (s / peak) ** CAUSTIC_SHARPNESS
        return s - float(s.mean())

    def apply(self, img, range_m, pose, t, strength, hfov):
        """Multiply the frame by a world-anchored, animated caustic net."""
        if range_m is None or strength <= 0.0 or range_m.shape[:2] != img.shape[:2]:
            return img
        h, w = img.shape[:2]
        r = np.nan_to_num(range_m, nan=6.0, posinf=6.0, neginf=6.0)
        r = np.clip(r, 0.15, 25.0).astype(np.float32)

        # Pixel ray directions in the camera frame, then the world XY the ray
        # lands on. Only x/y matter: caustics are a function of position on the
        # horizontal plane, which is what makes them stay put as the hull moves.
        fx = (0.5 * w) / np.tan(0.5 * hfov)
        u = (np.arange(w, dtype=np.float32) - 0.5 * w) / fx
        v = (np.arange(h, dtype=np.float32) - 0.5 * h) / fx
        uu, vv = np.meshgrid(u, v)
        px, py, pz, yaw = pose
        c, s_ = np.cos(yaw), np.sin(yaw)
        # camera looks along +x body; +u is to the right (-y), +v is down (-z)
        bx, by = 1.0, -uu
        wx = px + r * (bx * c - by * s_)
        wy = py + r * (bx * s_ + by * c)

        net = self.sample(wx, wy, t)
        # Caustics wash out with distance: the pattern is projected from the
        # surface and scattering blurs it long before the geometry disappears.
        fade = np.clip(1.0 - (r - 1.0) / 9.0, 0.15, 1.0)
        # Clamped positive: the darkest a caustic shadow gets is still lit by
        # scattered light, and a negative gain is an inverted image.
        gain = np.clip(1.0 + strength * net * fade, 0.25, 3.0)
        return img * gain[:, :, None]


class SurfaceGlare:
    """Brightening and bloom toward the water surface.

    Looking up in an outdoor pool, the surface is a bright, blown-out sheet with
    the sun disc smeared across it -- the single strongest cue that the vehicle
    is shallow and pointing up, and a genuine hazard for a detector, which sees
    a washed-out frame exactly when it is trying to find a gate near the
    surface.

    Applied in image space because that is where it lives: the effect is the
    camera looking at a bright thing, not a property of the geometry. It keys
    off the RANGE image so the bloom lands on distant bright regions rather
    than being smeared over the whole frame.
    """

    def apply(self, img, range_m, strength):
        if strength <= 0.0:
            return img
        lum = img.mean(axis=2)
        # The blown-out part only: a soft knee well above the frame mean, so a
        # merely bright floor does not bloom.
        thr = float(np.percentile(lum, 96.0))
        hot = np.clip((lum - thr) / max(1e-3, 1.0 - thr), 0.0, 1.0)
        if range_m is not None and range_m.shape[:2] == img.shape[:2]:
            r = np.nan_to_num(range_m, nan=25.0, posinf=25.0, neginf=25.0)
            # Only FAR bright things bloom: the surface and the sun through it,
            # not a prop 40 cm from the lens.
            hot *= np.clip((r - 2.0) / 6.0, 0.0, 1.0)
        bloom = cv2.GaussianBlur(hot, (0, 0), sigmaX=max(2.0, img.shape[1] / 40.0))
        return img + (strength * bloom)[:, :, None] * GLARE_RGB.reshape(1, 1, 3)


class ParticleField:
    """Drifting suspended particulate, composited into the camera image.

    THIS EXISTS BECAUSE GAZEBO'S PARTICLE EMITTER DOES NOT REACH CAMERA
    SENSORS. Measured: 0.4 m particles at 4000/s, emitter confirmed alive with
    a subscriber on its topic, and the frame from `front_camera/image_fx` was
    pixel-for-pixel as clean as with the emitter off. It is the same split that
    made `<scene><fog>` useless -- gz-sim renders a GUI scene and a sensor
    scene, and particles only land in the first. The Gazebo emitter is kept for
    the operator's view; everything the VISION PIPELINE sees comes from here.

    Particles persist frame to frame and sink. That is the whole point: a
    per-frame random speckle is just noise, which `noise` already provides, and
    a detector is unbothered by it. Coherent specks that drift are what put
    spurious small blobs in front of a bounding box across consecutive frames,
    which is the thing worth testing against.
    """

    def __init__(self, count: int = 260, seed: int = 7) -> None:
        rng = np.random.default_rng(seed)
        # x, y in [0,1] image fractions; z is a pseudo-distance in [0.15, 1]
        # that sets both size and brightness, so the field reads as a volume
        # rather than a decal.
        self.xy = rng.random((count, 2), dtype=np.float32)
        self.z = rng.uniform(0.15, 1.0, count).astype(np.float32)
        self.drift = rng.normal(0.0, 0.004, (count, 2)).astype(np.float32)
        self.rng = rng

    def step(self, dt: float) -> None:
        dt = float(np.clip(dt, 0.0, 0.5))
        # Sink, plus each particle's own lateral drift. Nearer particles (small
        # z) sweep faster -- parallax, and it is what makes the field read as
        # depth instead of a flat overlay.
        # Rates are SLOW on purpose. The first cut moved a particle ~10 px in
        # 100 ms, which does not overlap its own previous position at 1-4 px
        # radius -- that is a fresh speckle every frame, i.e. noise, and the
        # coherence this class exists for was absent. These give roughly
        # 2-20 px/s depending on depth, which a tracker can follow.
        speed = (1.0 / np.maximum(self.z, 0.15))[:, None]
        self.xy += self.drift * speed * dt
        self.xy[:, 1] += 0.004 * dt * speed[:, 0]
        # Wrap. A particle that leaves is replaced at a fresh random depth so
        # the field does not slowly sort itself into layers.
        out = (self.xy < -0.05) | (self.xy > 1.05)
        rows = out.any(axis=1)
        n = int(rows.sum())
        if n:
            self.xy[rows] = self.rng.random((n, 2), dtype=np.float32)
            self.z[rows] = self.rng.uniform(0.15, 1.0, n).astype(np.float32)

    def render(self, shape: tuple, strength: float) -> np.ndarray:
        """An (h, w) float32 alpha map in [0, 1]."""
        h, w = shape[0], shape[1]
        mask = np.zeros((h, w), dtype=np.float32)
        if strength <= 1e-4:
            return mask
        px = (self.xy[:, 0] * w).astype(np.int32)
        py = (self.xy[:, 1] * h).astype(np.int32)
        # Radius from 1 px at the back to ~4 px at the front.
        rad = np.clip((1.0 / self.z) * 1.1, 1.0, 4.0).astype(np.int32)
        alpha = np.clip((1.0 / self.z) * 0.16, 0.05, 0.85) * strength
        for i in range(len(px)):
            if 0 <= px[i] < w and 0 <= py[i] < h:
                cv2.circle(mask, (int(px[i]), int(py[i])), int(rad[i]),
                           float(alpha[i]), -1)
        # One blur turns hard discs into out-of-focus motes. Suspended matter a
        # few centimetres from a lens is never in focus.
        return cv2.GaussianBlur(mask, (5, 5), 1.4)


# Stateless, so one module-level instance is correct and costs nothing.
GLARE = SurfaceGlare()


def apply_underwater_fx(
    bgr: np.ndarray,
    depth_m: float,
    turbidity: float,
    backscatter: float,
    blur_sigma: float,
    noise: float,
    vignette: float,
    atten_scale: float = 1.0,
    # Per-pixel path length, metres, same shape as the frame. None ->
    # uniform attenuation by the vehicle's own depth, the old behaviour.
    range_m=None,
    # (h, w) float32 alpha map of suspended particulate, from ParticleField.
    particles=None,
    # Sunlight. `sun` is (SunlightField, (x, y, z, yaw), t_seconds, hfov_rad);
    # the two strengths are separate because caustics belong to the SCENE and
    # glare belongs to the CAMERA, and a session may want one without the other.
    caustics: float = 0.0,
    glare: float = 0.0,
    sun=None,
) -> np.ndarray:
    """Return a degraded BGR image. depth_m is negative below the surface."""
    depth = max(0.05, abs(float(depth_m)))
    t = max(0.0, float(turbidity))

    # ONE LOOKUP TABLE for attenuation, haze and contrast.
    #
    # All three are per-channel AFFINE functions of the uint8 input, and the
    # composition of affine maps is affine -- so the whole chain is exactly a
    # 256-entry table per channel, with no approximation anywhere.
    #
    # Worth doing because the obvious numpy spelling is the slow one:
    # `out *= gain` with gain shaped (1,1,3) measured 3.09 ms, because
    # broadcasting drops numpy off its vectorised inner loop onto the strided
    # path. Materialising or table-ising the operand is ~10x faster than
    # letting it broadcast. cv2.LUT with a float32 table returns float32, so
    # blur/noise/vignette still get a float frame.
    #
    #   v      = x / 255
    #   v      = v * gain + bias                    (attenuate + haze)
    #   v      = v * k + mean_post * (1 - k)        (contrast collapse)
    #   =>  x * (gain * k / 255)  +  (bias * k + mean_post * (1 - k))
    #
    # mean_post is the mean AFTER gain/bias, but mean is linear so it follows
    # from the raw mean without touching the full frame. That raw mean is taken
    # on every 4th pixel in each axis: it is only a contrast anchor, 1/16 the
    # work, and the difference is far below a JPEG quantisation step. Do NOT
    # subsample anything whose per-pixel value is consumed.
    path = depth * (0.35 + 0.65 * t)          # optical path grows with turbidity
    atten = np.exp(-ATTEN_RGB * path * atten_scale)
    scatter = float(np.clip(backscatter * t * (1.0 - np.exp(-0.4 * path)), 0.0, 1.0))
    gain = atten * (1.0 - scatter)
    bias = HAZE_RGB * scatter

    k = 1.0 - 0.35 * t
    mean_raw = bgr[::4, ::4].reshape(-1, 3).mean(axis=0) / 255.0
    mean_post = mean_raw * gain + bias

    ramp = np.arange(256, dtype=np.float32)
    lut = np.empty((1, 256, 3), dtype=np.float32)
    for c in range(3):
        lut[0, :, c] = ramp * (gain[c] * k / 255.0) + (bias[c] * k + mean_post[c] * (1.0 - k))
    out = cv2.LUT(bgr, lut)

    if range_m is not None and range_m.shape[:2] == bgr.shape[:2]:
        # The LUT above applied ONE path length to every pixel. Correct each by
        # the ratio between its true path and that one, so near pixels get less
        # attenuation and far pixels more. Doing it as a correction keeps the
        # LUT -- which is what makes this fast -- for one exponential per frame.
        r = np.nan_to_num(range_m, nan=depth, posinf=depth, neginf=depth)
        r = np.clip(r, 0.05, 40.0).astype(np.float32)
        # RANGE_ATTEN scales the along-path coefficient down from the open-ocean
        # figures in ATTEN_RGB. Those are right for the sea and much too strong
        # for a chlorinated pool: applied raw, the far wall of a 25 m pool went
        # essentially black (red down to 0.004 of its value at 20 m) while a
        # real pool photo shows the far end clearly, just blue and low-contrast.
        #
        # The floor keeps the far field visible-but-degraded, which is the point
        # -- a detector must find props at range through worse imagery, not be
        # handed a black frame it cannot possibly work with.
        rel = (r * (0.35 + 0.65 * t) - path)[:, :, None]
        att = np.exp(-ATTEN_RGB.reshape(1, 1, 3) * rel * atten_scale * RANGE_ATTEN)
        out *= np.maximum(att, RANGE_FLOOR)
        # Haze accumulates along the path too: a distant surface is not merely
        # dimmer, it is washed toward the colour of the water in between.
        far = np.clip(rel * 0.02 * t, 0.0, 0.55)
        out = out * (1.0 - far) + HAZE_RGB.reshape(1, 1, 3) * far

    # SUNLIGHT, after attenuation and before blur.
    #
    # Order matters and is not arbitrary: caustics are light arriving at a
    # surface, so they belong with the surface's own colour and must be dimmed
    # by the same water the surface is seen through -- applying them after the
    # attenuation LUT would paint a crisp light net onto a wall that is
    # supposed to be washed out. Glare is the opposite: it is what the CAMERA
    # does with a bright source, so it sits outside the water term and is added
    # last of the light effects, before blur smears it.
    if sun is not None and (caustics > 0.0 or glare > 0.0):
        field, pose, t_now, hfov = sun
        if caustics > 0.0:
            out = field.apply(out, range_m, pose, t_now, caustics, hfov)
        if glare > 0.0:
            out = GLARE.apply(out, range_m, glare)

    # ONE uint8 conversion, at the very end.
    #
    # Blur, noise and vignette each used to round-trip the whole frame
    # uint8 -> float32 -> clip -> uint8. Measured, each of those round trips is
    # ~5.6 ms on a 640x480x3 frame -- more than the effect it was applying.
    # Staying in float32 [0,1] and converting once cuts the function roughly in
    # half. cv2.GaussianBlur takes float32 directly, so nothing is given up.
    sigma = blur_sigma * (0.4 + 1.2 * t)
    if sigma > 0.15:
        k = max(3, int(round(sigma * 2) * 2 + 1))
        out = cv2.GaussianBlur(out, (k, k), sigma)

    if noise > 1e-4:
        out += _noise_tile(out.shape) * (noise * (0.5 + t))

    if particles is not None:
        # Composited BEFORE vignette and AFTER attenuation: a mote floating a
        # few centimetres from the lens is not dimmed by 8 m of water, but it
        # does fall off toward the frame edge with everything else.
        a = particles[:, :, None]
        out = out * (1.0 - a) + PARTICLE_RGB.reshape(1, 1, 3) * a

    if vignette > 1e-4:
        # Full (h, w, 3), not (h, w, 1) broadcast -- same 3.09 ms -> 0.6 ms
        # reason as the gain above.
        out *= _vignette_mask(out.shape, vignette)

    out *= np.float32(255.0)
    np.clip(out, 0.0, 255.0, out=out)
    return out.astype(np.uint8)


class UnderwaterFx(Node):
    def __init__(self) -> None:
        super().__init__('underwater_fx')

        self.declare_parameter('turbidity', 0.45)
        self.declare_parameter('backscatter', 0.55)
        self.declare_parameter('blur_sigma', 0.8)
        self.declare_parameter('noise', 0.012)
        self.declare_parameter('vignette', 0.25)
        self.declare_parameter('atten_scale', 1.0)
        # Suspended particulate. 0 disables it. This is NOT the Gazebo marine
        # snow emitter -- that one is invisible to camera sensors (see
        # ParticleField) -- it is the only particulate the detector ever sees.
        self.declare_parameter('particulate', 0.35)
        # SUNLIGHT. Off by default: both change every frame a detector sees,
        # so they are a stated choice per session rather than something that
        # quietly appears in every capture -- the same stance as `lens_flare`.
        self.declare_parameter('caustics', 0.0)
        self.declare_parameter('glare', 0.0)
        self.declare_parameter('randomize_on_start', False)
        self.declare_parameter('enabled', True)

        self._depth = -0.8
        self._pose = (0.0, 0.0, -0.8, 0.0)
        self._sun = SunlightField()
        self._t0 = time.monotonic()
        # Latest range image per camera, for per-pixel attenuation. None until
        # one arrives, and the filter falls back to uniform if it never does --
        # so a missing depth sensor degrades to the old behaviour rather than
        # to a crash or a black frame.
        self._range = {'front': None, 'bottom': None}
        self._particles = ParticleField()
        self._particle_t = time.monotonic()
        self._reload_params()
        if self.get_parameter('randomize_on_start').value:
            self._randomize()

        self.create_subscription(Odometry, GROUND_TRUTH, self._on_odom, 10)
        self._pub_front = self.create_publisher(Image, FRONT_FX, qos_profile_sensor_data)
        self._pub_bottom = self.create_publisher(Image, BOTTOM_FX, qos_profile_sensor_data)
        self.create_subscription(
            Image, FRONT_RANGE, lambda m: self._on_range('front', m),
            qos_profile_sensor_data
        )
        self.create_subscription(
            Image, BOTTOM_RANGE, lambda m: self._on_range('bottom', m),
            qos_profile_sensor_data
        )
        self.create_subscription(
            Image, FRONT_RAW, lambda m: self._on_image(m, self._pub_front, 'front'), qos_profile_sensor_data
        )
        self.create_subscription(
            Image, BOTTOM_RAW, lambda m: self._on_image(m, self._pub_bottom, 'bottom'), qos_profile_sensor_data
        )
        self.add_on_set_parameters_callback(self._on_params)
        self.get_logger().info(
            f'underwater_fx ready  turbidity={self._turbidity:.2f}  '
            f'publishing {FRONT_FX} / {BOTTOM_FX}'
        )

    def _reload_params(self) -> None:
        self._turbidity = float(self.get_parameter('turbidity').value)
        self._backscatter = float(self.get_parameter('backscatter').value)
        self._blur_sigma = float(self.get_parameter('blur_sigma').value)
        self._noise = float(self.get_parameter('noise').value)
        self._vignette = float(self.get_parameter('vignette').value)
        self._atten_scale = float(self.get_parameter('atten_scale').value)
        self._particulate = float(self.get_parameter('particulate').value)
        self._caustics = float(self.get_parameter('caustics').value)
        self._glare = float(self.get_parameter('glare').value)
        self._enabled = bool(self.get_parameter('enabled').value)

    def _randomize(self) -> None:
        rng = np.random.default_rng()
        self.set_parameters(
            [
                rclpy.parameter.Parameter('turbidity', value=float(rng.uniform(0.2, 0.85))),
                rclpy.parameter.Parameter('backscatter', value=float(rng.uniform(0.3, 0.8))),
                rclpy.parameter.Parameter('blur_sigma', value=float(rng.uniform(0.3, 1.4))),
                rclpy.parameter.Parameter('noise', value=float(rng.uniform(0.005, 0.025))),
            ]
        )
        self._reload_params()

    def _on_params(self, params) -> SetParametersResult:
        for p in params:
            if p.name == 'turbidity':
                self._turbidity = float(p.value)
            elif p.name == 'backscatter':
                self._backscatter = float(p.value)
            elif p.name == 'blur_sigma':
                self._blur_sigma = float(p.value)
            elif p.name == 'noise':
                self._noise = float(p.value)
            elif p.name == 'vignette':
                self._vignette = float(p.value)
            elif p.name == 'atten_scale':
                self._atten_scale = float(p.value)
            elif p.name == 'particulate':
                self._particulate = float(p.value)
            # A parameter declared and read at startup but MISSING here is
            # silently un-tunable: `ros2 param set` reports success, the value
            # changes in the parameter server, and the node keeps using the one
            # it read at construction. Measured on the first run of this
            # feature -- caustics 0.0 -> 0.45 moved the frame diff by 0.4 out of
            # 10.5, which reads as "the effect does nothing" and was in fact
            # "the effect was never switched on".
            elif p.name == 'caustics':
                self._caustics = float(p.value)
            elif p.name == 'glare':
                self._glare = float(p.value)
            elif p.name == 'enabled':
                self._enabled = bool(p.value)
        return SetParametersResult(successful=True)

    def _on_odom(self, msg: Odometry) -> None:
        self._depth = float(msg.pose.pose.position.z)
        # FULL POSE, not just depth. Caustics are sampled at the world position
        # each pixel looks at, so the pattern stays put in the world while the
        # vehicle moves through it. Anchored to the camera instead, it swims
        # with the view and reads as a dirty lens.
        pos, ori = msg.pose.pose.position, msg.pose.pose.orientation
        self._pose = (float(pos.x), float(pos.y), float(pos.z),
                      math.atan2(2.0 * (ori.w * ori.z + ori.x * ori.y),
                                 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)))

    def _on_range(self, cam: str, msg: Image) -> None:
        # 32FC1 from Gazebo's depth_camera: metres along the optical axis.
        try:
            a = np.frombuffer(msg.data, dtype=np.float32)
            self._range[cam] = a[:msg.height * msg.width].reshape(
                msg.height, msg.width)
        except Exception:
            self._range[cam] = None

    def _on_image(self, msg: Image, pub, cam: str = 'front') -> None:
        # NOTHING unless somebody is watching this camera.
        #
        # This node is the single biggest drag on simulator smoothness, and the
        # cost is not the filter -- that is ~4 ms/frame -- it is moving two
        # extra ~1 MB image streams per camera through DDS and re-encoding
        # them. Measured 2026-08-28 on the SAUVC final course:
        #
        #     fx off :  8.21 Hz,  jitter (stdev) 5 ms
        #     fx on  :  2.83 Hz,  jitter        440 ms
        #
        # That jitter is exactly what the operator reports as laggy teleop
        # video and juddery dataset playback. Almost always only one camera is
        # actually being looked at, so gating on subscriber count gives the
        # other one back for free -- and with no viewer at all (a headless
        # mission run) the node costs nothing.
        #
        # Cheap and exact: rmw already tracks this, and image_fx is a
        # debug/dataset topic, so there is no late-joiner to miss a frame that
        # mattered. Do NOT copy this to a control-path publisher.
        if pub.get_subscription_count() == 0:
            return
        try:
            bgr = _msg_to_bgr(msg)
        except ValueError as exc:
            self.get_logger().warn(str(exc))
            return
        particles = None
        if self._enabled and self._particulate > 1e-4:
            # Advance on MEASURED elapsed time, not a per-frame constant. The
            # two cameras publish at different and variable rates, and a fixed
            # step made the drift speed depend on frame rate -- the same trap
            # the T200 spin-up filter fell into.
            now = time.monotonic()
            self._particles.step(now - self._particle_t)
            self._particle_t = now
            particles = self._particles.render(bgr.shape, self._particulate)
        if self._enabled:
            bgr = apply_underwater_fx(
                bgr,
                self._depth,
                self._turbidity,
                self._backscatter,
                self._blur_sigma,
                self._noise,
                self._vignette,
                self._atten_scale,
                range_m=self._range.get(cam),
                particles=particles,
                caustics=self._caustics,
                glare=self._glare,
                sun=(self._sun, self._pose, time.monotonic() - self._t0,
                     CAMERA_HFOV),
            )
        pub.publish(_bgr_to_msg(bgr, msg.header.stamp, msg.header.frame_id))

    def snapshot_params(self) -> dict:
        return {
            'turbidity': self._turbidity,
            'backscatter': self._backscatter,
            'blur_sigma': self._blur_sigma,
            'noise': self._noise,
            'vignette': self._vignette,
            'atten_scale': self._atten_scale,
            'enabled': self._enabled,
            'depth_m': self._depth,
            'particulate': self._particulate,
            'caustics': self._caustics,
            'glare': self._glare,
        }


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = UnderwaterFx()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # Guarded: `ros2 launch` SIGINTs the whole group, and rclpy may have
        # already torn the context down. An unguarded call then raises
        # "rcl_shutdown already called" and the node exits 1 -- a clean
        # ctrl-c reports three processes as DIED.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
