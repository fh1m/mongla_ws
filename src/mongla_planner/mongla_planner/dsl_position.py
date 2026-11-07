"""Where the vehicle is: bearings, ranges, position fixes, the fused pose, the anchored heading.

Mixed into `MonglaMission` (mongla_dsl.py), which owns the client, the
node and the caches these methods read.
"""
from __future__ import annotations

import time as _time

import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from .model_context import ClassRef

from .dsl_detections import SeeMixin
from .dsl_support import FUSED_POSE_MAX_AGE_S


class WhereMixin:
    # In-water horizontal FOV per camera, used ONLY when that camera has not
    # published a calibrated CameraInfo. Derived from the committed calibration
    # files through the flat port (`optics.fov_air_to_water`):
    #   downward  Microdia global shutter, fx 1027.87 @1280: 63.8 air -> 46.7 water
    #   forward   Fantech Luminous C30,    fx  851.23 @1280: 73.9 air -> 53.6 water
    # ⛔ This was ONE constant, 46.7, measured on the global-shutter unit when it
    # was still called "forward". The cameras were re-assigned on 2026-09-07 and
    # the constant stayed, so every forward-camera range read 17 % long and every
    # forward bearing 13 % short, with nothing logging a fault.
    HFOV_WATER_DEG_BY_CAMERA = {'forward': 53.6, 'downward': 46.7}

    def _optics(self, camera):
        """(fx, fy, cx, cy, width, height) in AIR from CameraInfo, or None."""
        ks = getattr(self, '_cam_k', None)
        sizes = getattr(self, '_img_size', None)
        if not isinstance(ks, dict) or not isinstance(sizes, dict):
            return None
        k, wh = ks.get(camera), sizes.get(camera)
        if not k or not wh or not wh[0]:
            return None
        return (*k, float(wh[0]), float(wh[1]))

    def _water_camera(self, camera):
        """(rectifier, K_rect) for `camera` from its live CameraInfo, or None.

        ⛔ A FLAT PORT HAS NO SINGLE FOCAL LENGTH. The in-water focal grows with
        field angle (forward camera at 640 px: ~567 px at the centre, ~634 at the
        edge), so ANY one focal -- the old shared 46.7 deg, or a per-camera FOV --
        is wrong by up to ~12 % somewhere in the frame. Rectified pixels ARE a
        pinhole at `K_rect`, which is the model `lock_node` and `pnp_node` use;
        the DSL now uses the same one instead of a second copy.
        """
        o = WhereMixin._optics(self, camera)
        if o is None:
            return None
        from mongla_vision.optics import rectifier_for
        fx, fy, cx, cy, _w, _h = o
        medium = getattr(self, 'medium', 'water')
        medium = medium if isinstance(medium, str) else 'water'
        D = (getattr(self, '_cam_d', None) or {}).get(camera)
        rect, K_rect, _note = rectifier_for(
            [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], medium, D)
        if rect is None:                # air, no lens model: K is the pinhole
            class _Identity:
                @staticmethod
                def rectify(pts):
                    return pts
            rect = _Identity()
        return rect, K_rect

    def hfov_water_deg(self, camera: str | None = None) -> float:
        """In-water horizontal FOV of `camera`: its live calibration, else its fallback."""
        import math as _math
        from mongla_vision.optics import fov_air_to_water
        cam = camera or getattr(self, 'camera', 'forward')
        o = WhereMixin._optics(self, cam)
        if o is not None:
            fx, _fy, _cx, _cy, w, _h = o
            return fov_air_to_water(2.0 * _math.degrees(_math.atan(w / 2.0 / fx)))
        return float(WhereMixin.HFOV_WATER_DEG_BY_CAMERA.get(
            str(cam), WhereMixin.HFOV_WATER_DEG_BY_CAMERA['forward']))

    def _box_row(self, prop_class, camera, default):
        """Image row of the box `where_offset` read -- the largest one."""
        name = str(getattr(prop_class, 'class_name', prop_class)).strip().lower()
        boxes = [r for r in self._records(camera, 1.0) if r[0] == name]
        if not boxes:
            return float(default)
        return float(max(boxes, key=lambda r: r[3] * r[4])[2])

    def bearing_to(self, prop_class: str, *, camera: str | None = None,
                   hfov_deg: float | None = None) -> float | None:
        """Bearing from the hull to a visible prop, in WORLD terms. None if unseen.

        Pixel to angle, then relative to absolute through the anchored
        heading. With live CameraInfo the box centre is unprojected in full --
        lens off, then the port, then the pinhole at `K_rect` (issue #24); only
        without it does this fall back to the FOV reading of `where_offset`,
        which is good to about a degree near the centre and worse at the edge.

        Unanchored, the number returned is relative to boot, so a fix built
        from it is in a rotated frame. `fix_position` says so rather than
        letting the frames silently mix.
        """
        off = self.where_offset(prop_class, camera=camera)
        if off is None:
            return None
        cam = camera or getattr(self, 'camera', 'forward')
        wc = None if hfov_deg is not None else WhereMixin._water_camera(self, cam)
        if wc is not None:
            import math as _math
            rect, K_rect = wc
            w = WhereMixin._optics(self, cam)[4]
            u = (float(off) + 1.0) * w / 2.0
            # ⛔ THE BOX'S OWN ROW, NOT THE PRINCIPAL ROW (issue #24). Lens and
            # port are both RADIAL, so where a point lands horizontally depends
            # on how far it is from the centre in BOTH axes. Reading every box
            # on row cy undid them for a point that was not there.
            v = WhereMixin._box_row(self, prop_class, cam, K_rect[1][2])
            u_r = float(rect.rectify([[u, v]])[0][0])
            # Azimuth of a pinhole ray is atan(x/f) whatever its height.
            rel = _math.degrees(_math.atan((u_r - K_rect[0][2]) / K_rect[0][0]))
            # ⛔ THE HEADING OF THAT FRAME, not of now (issue #37): mid-turn
            # the detection is a detector latency old, and the hull has
            # turned since by yaw_rate x latency.
            then = self.absolute_heading() - SeeMixin._turned_since(self, cam)
            return (then + rel) % 360.0
        half = float(hfov_deg if hfov_deg is not None
                     else WhereMixin.hfov_water_deg(self, camera)) / 2.0
        return (self.absolute_heading() + float(off) * half) % 360.0

    def fix_position(self, *, props: list | None = None,
                     camera: str | None = None) -> object:
        """Where we are in the pool, from bearings to props we can see.

        ⛔ THE COURSE IS THE LANDMARK FIELD. A team that could not localise
        underwater added obstacles to their pool to make features, and wrote
        the approach off because a venue will not let you. That is backwards:
        the competition course is the densest set of surveyed, known-size,
        already-detected objects we will ever operate in. The thing they had to
        fake is what the venue hands us.

        Needs a loaded course for the positions, two visible props whose
        positions were measured, and enough angle between them. Returns a
        `Fix` whose `ok` is False with a stated reason -- a thin geometry is
        refused rather than reported with a big residual, because two nearly
        parallel sight lines still cross, just far away and wrongly.
        """
        from mongla_localization.resection import Fix, fix_from_bearings

        course = getattr(self, '_course', None)
        if course is None:
            return Fix(False, reason='no course loaded: call use_course(<name>)')
        if getattr(self, '_heading_offset', None) is None:
            return Fix(False, reason='heading is not anchored, so bearings are '
                                     'relative to boot and a fix would be in a '
                                     'rotated frame. anchor_on(<prop>) first.')
        positions, sightings = {}, {}
        for name, prop in course.props.items():
            if props is not None and name not in props:
                continue
            if not prop.has_position:
                continue
            cls = prop.detect_class or name
            bearing = self.bearing_to(cls, camera=camera)
            if bearing is None:
                continue
            positions[name] = (float(prop.x_m), float(prop.y_m))
            sightings[name] = bearing
        got = fix_from_bearings(sightings, positions)
        if got.ok:
            # FEED THE FILTER, not just the log. `update_position` is the only
            # channel that bounds horizontal drift, and until this line the fix
            # was computed, printed, and thrown away -- the filter had no
            # caller for the one measurement it cannot replace. Best-effort:
            # a missing localization node must not fail a mission step.
            self._publish_fix(got.x_m, got.y_m, camera=camera or self.camera)
            self.log.info(
                f'[FIX  ] pool position ({got.x_m:+.2f}, {got.y_m:+.2f}) m from '
                f'{got.used} props, residual {got.residual_m:.3f} m, '
                f'spread {got.separation_deg:.0f} deg')
        else:
            self.log.warning(f'[FIX  ] no position fix: {got.reason}')
        return got

    def pose(self, *, timeout: float = 2.0, any_frame: bool = False):
        """Where the filter thinks we are: `(x_m, y_m, depth_m, yaw_deg)` or None.

        The mission-facing read of `/mongla/odom`. None when the localization
        node is not running, has not published yet, or -- the part that
        matters -- when its answer is not yet in the POOL frame.

            here = mongla.pose()
            if here and here[0] > 4.0:
                mongla.turn(180)

        ⛔ NONE IS ALSO THE ANSWER FOR A POSITION THAT EXISTS AND IS WRONG.
        Two ways that happens, and neither looks any different from a good
        reading at this interface:

          * BEFORE THE HEADING IS ANCHORED the estimate is in the board's boot
            frame, so comparing it against a course coordinate is a comparison
            in a rotated frame. The node says so in `header.frame_id` -- it
            publishes `odom` until an anchor arrives and `pool` after -- and
            this reads that field rather than trusting the numbers.
          * WITH NO VELOCITY AIDING the position runs away. Measured on the
            vehicle with flow absent and ZUPT off: 635 m in 95 s, while the
            node published a healthy-looking pose at 10 Hz throughout. The
            node warns in ITS log; a mission holding a bare tuple cannot hear
            that, which is this repo's recurring defect with the roles
            reversed -- an honest producer and a deaf consumer.

        Returning None rather than a tuple is the same rule already applied to
        a NaN yaw in `_effective_yaw_deg`: absence is safe, a plausible wrong
        number is not.

        `any_frame=True` returns the unanchored reading anyway, for a
        diagnostic that genuinely wants to watch the filter converge. It is
        opt-in precisely so it cannot be the thing a mission does by accident.

        Pairs with `fix_position()` / `fix_from_prop()`, which push a resected
        position INTO the filter; this reads the fused result back out.
        """
        import math
        import time as _t
        from nav_msgs.msg import Odometry
        node = self.client.node
        if self._odom_sub is None:
            def _keep(msg):
                self._odom = msg
            self._odom_sub = node.create_subscription(
                Odometry, '/mongla/odom', _keep, 10)
        deadline = _t.monotonic() + float(timeout)
        # Pump rather than sleep: the mission thread owns this executor, so a
        # bare sleep here would spin the clock and receive nothing.
        while self._odom is None and _t.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if self._odom is None:
            return None
        frame = str(self._odom.header.frame_id)
        if frame != 'pool' and not any_frame:
            if not self._pose_frame_warned:
                self._pose_frame_warned = True
                self.log.warning(
                    f'[LOCAL] pose() withheld: the filter is publishing in '
                    f'{frame!r}, not the pool frame. anchor_on(<prop>) makes '
                    f'it absolute. (pose(any_frame=True) to read it anyway.)')
            return None
        p = self._odom.pose.pose.position
        q = self._odom.pose.pose.orientation
        yaw = math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                      1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
        # /mongla/odom is NED: z is +depth, and this API returns depth negative.
        return (float(p.x), float(p.y), -float(p.z), yaw)

    def motion(self, *, timeout: float = 1.0):
        """Is the hull moving the way it is being told to?

        `'ok'` | `'blocked'` | `'unknown'`, or None when localization is not
        running. `'blocked'` means thrust is commanded and the floor is not
        moving: against a prop, snagged, or a thruster is dead.

            mongla.vision.move('gate', fwd=80)
            if mongla.motion() == 'blocked':
                mongla.move_back(duration=2)

        ⛔ A TIMED MOVE REPORTS SUCCESS WHETHER OR NOT THE HULL WENT ANYWHERE.
        This is the only reading that can tell them apart without a detection.
        `'unknown'` is common and honest: no flow, no learned model, or a board
        primitive running (the host does not know its demand). Treat unknown as
        "no evidence", never as "ok".
        """
        import time as _t
        from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
        from std_msgs.msg import String
        node = self.client.node
        if getattr(self, '_motion_sub', None) is None:
            self._motion = None

            def _keep(msg):
                self._motion = str(msg.data)
            latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                                 durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
            self._motion_sub = node.create_subscription(
                String, '/mongla/localization/motion', _keep, latched)
        deadline = _t.monotonic() + float(timeout)
        while self._motion is None and _t.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        # Pump once more so a change since the last call is seen.
        rclpy.spin_once(node, timeout_sec=0.0)
        return self._motion

    def floor_height(self, *, max_age_s: float = 1.0, timeout: float = 1.0):
        """Height above the floor in metres, from the floor's own tiles, or None.

        Measured by the downward camera off the tile grating (`tile_m` must be
        set for the venue), NOT depth minus a typed pool depth. None when no
        tiles are visible or the last reading is older than `max_age_s`:
        a stale height reads as a fresh one to every comparison.

            h = mongla.floor_height()
            if h is None:
                ...                     # no tiles in view: do not guess
            elif h < 0.6:
                ...                     # too close to the floor for the drop
        """
        import time as _t
        from sensor_msgs.msg import Range
        node = self.client.node
        if getattr(self, '_floor_h_sub', None) is None:
            self._floor_h = None

            def _keep(msg):
                self._floor_h = (float(msg.range), _t.time())
            self._floor_h_sub = node.create_subscription(
                Range, '/mongla/vision/downward/floor_height', _keep, 10)
        deadline = _t.monotonic() + float(timeout)
        while self._floor_h is None and _t.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        rclpy.spin_once(node, timeout_sec=0.0)
        if self._floor_h is None:
            return None
        h, rx = self._floor_h
        return h if _t.time() - rx <= float(max_age_s) else None

    def flare_order(self, *, timeout: float = 0.0, max_age_s: float = 3.0):
        """The flare order the operator sent over LoRa this mission, or None.

        Returns a tuple of colour names, e.g. ('red', 'blue', 'yellow'). Blocks up to
        `timeout` seconds for one to arrive (0 = just look). None when no order has been
        received, when the manager stopped republishing it more than `max_age_s` ago,
        or when the board latched it BEFORE this mission started -- a practice order
        still in the board's RAM must never stand in for this run's.

            order = mongla.flare_order(timeout=45.0)   # hold at the listen station
            if order is None:
                ...                                    # no order: bump all, earn 20 each
        """
        import json
        import time as _t
        from std_msgs.msg import String
        from rclpy.qos import QoSProfile, DurabilityPolicy
        node = self.client.node
        if getattr(self, '_flare_sub', None) is None:
            self._flare = None

            def _keep(msg):
                try:
                    d = json.loads(msg.data)
                    self._flare = (tuple(d['colours']), float(d['latched_at']), _t.time())
                except (ValueError, KeyError, TypeError):
                    pass
            self._flare_sub = node.create_subscription(
                String, '/mongla/flare_order', _keep,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        def _current():
            if self._flare is None:
                return None
            colours, latched_at, rx = self._flare
            if latched_at < self._mission_start_wall or _t.time() - rx > float(max_age_s):
                return None
            return colours

        deadline = _t.monotonic() + float(timeout)
        rclpy.spin_once(node, timeout_sec=0.0)
        while _current() is None and _t.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        return _current()

    def range_to(self, prop_class: str, *, camera: str | None = None,
                 stale_after: float = 1.0):
        """Metres to a visible prop of known width, or None. Also its sigma.

        Returns `(range_m, sigma_m)`. The pinhole relation `Z = f W / w` needs
        nothing but the prop's real width and its box width in pixels, both of
        which we already have -- `target_geometry` is the measured width table
        and the detector gives the box.

        ⛔ THE SIGMA IS DERIVED, NOT CHOSEN. Differentiating gives
        `dZ/dw = -Z^2 / (f W)`, so range error grows with the SQUARE of range:
        one pixel of box-width noise is 6 mm at 1 m and 15 cm at 5 m for a gate.
        A constant sigma would tell the filter a far reading is as good as a
        near one, which is the single most common way a landmark fix poisons a
        position estimate. This matches the published analytical models for
        planar-marker pose variance, which are functions of range and angle.
        """
        from mongla_vision.target_geometry import width_for

        if isinstance(prop_class, ClassRef):
            prop_class = prop_class.class_name
        real_w = width_for(str(prop_class))
        if real_w <= 0.0:
            return None
        cam = camera or self.camera
        self._subscribe_detections(cam)
        self._pump_detections(cam)
        target = str(prop_class).strip().lower()
        boxes = [r for r in self._records(cam, stale_after) if r[0] == target]
        if not boxes:
            return None
        # The LARGEST box, not the most confident -- size is what the range is
        # read off, and a small confident box is a worse range than a large
        # doubtful one. Same rule as `identity.pick_structure`.
        big = max(boxes, key=lambda r: float(r[3]))
        w_px = float(big[3])
        if w_px <= 0.0:
            return None
        wc = WhereMixin._water_camera(self, cam)
        if wc is not None:
            # Both EDGES through the port, then the pinhole at K_rect: the box
            # width compresses by the refraction at ITS field angle.
            rect, K_rect = wc
            e = rect.rectify([[big[1] - w_px / 2.0, big[2]], [big[1] + w_px / 2.0, big[2]]])
            w_px = abs(float(e[1][0]) - float(e[0][0]))
            f_px = float(K_rect[0][0])
            if w_px <= 0.0:
                return None
            z = f_px * real_w / w_px
            return (z, (z * z) * 1.0 / (f_px * real_w))
        img_w = self._img_size.get(cam, (0.0, 0.0))[0] or 640.0
        f_px = self.focal_px(img_w, camera=cam)
        z = f_px * real_w / w_px
        # One pixel of box-width noise, which is optimistic for a YOLO box and
        # stated as such rather than padded with an invented factor.
        sigma = (z * z) * 1.0 / (f_px * real_w)
        return (z, sigma)

    def floor_range(self, target: str, *, plane_below_m: float, pitch_deg: float,
                    camera: str | None = None,
                    plane_sigma_m: float = 0.0, stale_after: float = 1.0):
        """Metres to a target standing on (or hanging from) a known plane.

        Returns `(range_m, sigma_m)` like `range_to`, or None. Needs NO prop
        width: the ray through the box's foot meets the floor at one point.
        `plane_below_m` = floor depth under the target minus the camera's
        depth (positive); pass a negative value for a surface-hung target and
        its TOP edge is used instead. Use when `range_to` has nothing to say --
        unknown width, or a box cut by the frame side -- and take whichever
        sigma is smaller when both answer. Detail and failure modes:
        `mongla_localization.floor_plane`.

        ⛔ BOTH GEOMETRY ARGUMENTS ARE REQUIRED, ON PURPOSE. Pitch is the
        dominant error term (17 cm per degree at h 1 m, R 3 m), and the bench
        hull read +1.77 deg nose-up, so a silent level-camera default would be
        a ~30 cm bias no test sees. `pitch_deg` is the optical axis BELOW
        horizontal: mount pitch minus the hull's nose-up pitch (forward camera
        on a hull pitched 1.77 deg nose-up: `pitch_deg=-1.77`).

        ⛔ SIGNS. Depths here are negative below the surface, and getting the
        subtraction backwards yields a NEGATIVE plane, which is read as a
        surface-hung target -- a confident wrong answer, not a refusal:

            hull = mongla.pose()[2]                   # e.g. -0.60
            plane_below_m = abs(-1.60) - abs(hull)    # floor 1.60 m -> 1.00

        Pixels are assumed square (`focal_px` is fx and is passed as fy too),
        which holds for the cameras we fly.

        ⛔ A box touching the frame edge on the side we read is REFUSED: the
        foot is outside the image and the box edge is not the foot, which would
        return a confident range to the frame border.
        """
        from mongla_localization.floor_plane import intersect

        if isinstance(target, ClassRef):
            target = target.class_name
        cam = camera or self.camera
        self._subscribe_detections(cam)
        self._pump_detections(cam)
        name = str(target).strip().lower()
        boxes = [r for r in self._records(cam, stale_after) if r[0] == name]
        if not boxes:
            return None
        _, cx, cy, w, h, _ = max(boxes, key=lambda r: r[3] * r[4])
        img_w, img_h = self._img_size.get(cam, (0.0, 0.0))
        img_w, img_h = (img_w or 640.0), (img_h or 480.0)
        v = cy + h / 2.0 if plane_below_m > 0.0 else cy - h / 2.0
        if v >= img_h - 2.0 or v <= 2.0:
            return None
        wc = WhereMixin._water_camera(self, cam)
        if wc is not None:
            # The foot through the port, then the pinhole at K_rect -- which also
            # carries the calibrated principal point: the forward camera's cy sits
            # 70 px above centre at 720 p, ~6 deg of pitch, about a metre at 3 m.
            rect, K_rect = wc
            cx, v = (float(x) for x in rect.rectify([[cx, v]])[0])
            f_px, fy_px = float(K_rect[0][0]), float(K_rect[1][1])
            pcx, pcy = float(K_rect[0][2]), float(K_rect[1][2])
        else:
            f_px = self.focal_px(img_w, camera=cam)
            pcx, pcy, fy_px = img_w / 2.0, img_h / 2.0, f_px
        hit = intersect(cx, v, fx=f_px, fy=fy_px, cx=pcx, cy=pcy,
                        plane_below_m=float(plane_below_m),
                        pitch_deg=float(pitch_deg),
                        plane_sigma_m=float(plane_sigma_m))
        return None if hit is None else (hit.range_m, hit.sigma_m)

    def fix_from_prop(self, prop: str, *, camera: str | None = None):
        """Pool position from ONE surveyed prop: range and bearing together.

        ⛔ THIS IS THE DVL WE DO NOT HAVE. `fix_position()` needs TWO props
        with 12 degrees between them, because bearings alone cannot fix a
        range -- and two props are visible together far less often than one.
        Adding the range collapses that requirement: a single prop of known
        width at a surveyed position pins the hull completely.

        Everything it needs already existed separately and was never combined:
        the surveyed position from the course map, the absolute bearing from
        the anchored heading, and the range from the width table. The
        arithmetic is one line; the capability is the difference between
        localising when the course cooperates and localising whenever anything
        is in view.

        Returns a `Fix`. Publishes to the filter on success, with the
        range-derived sigma rather than a constant.
        """
        import math as _m
        from mongla_localization.resection import Fix

        course = getattr(self, '_course', None)
        if course is None:
            return Fix(False, reason='no course loaded: call use_course(<name>)')
        if getattr(self, '_heading_offset', None) is None:
            return Fix(False, reason='heading is not anchored, so the bearing '
                                     'is relative to boot and the fix would be '
                                     'in a rotated frame. anchor_on(<prop>) first.')
        p = course.props.get(prop)
        if p is None:
            return Fix(False, reason=f'{prop!r} is not in course {course.name!r}')
        if not p.has_position:
            return Fix(False, reason=f'{prop!r} has no measured position; a '
                                     f'prop we cannot place cannot place us')
        cls = p.detect_class or prop
        bearing = self.bearing_to(cls, camera=camera)
        if bearing is None:
            return Fix(False, reason=f'{cls!r} is not visible right now')
        got = self.range_to(cls, camera=camera)
        if got is None:
            return Fix(False, reason=f'no width for {cls!r}, so no range')
        rng, sigma = got
        # The hull sits one range BACK along the bearing from the prop.
        rad = _m.radians(bearing)
        x = float(p.x_m) - rng * _m.cos(rad)
        y = float(p.y_m) - rng * _m.sin(rad)
        self._publish_fix(x, y, sigma=sigma, camera=camera or self.camera)
        self.log.info(
            f'[FIX  ] pool position ({x:+.2f}, {y:+.2f}) m from {prop!r} at '
            f'{rng:.2f} m bearing {bearing:.0f} deg (sigma {sigma:.2f} m)')
        return Fix(True, x_m=x, y_m=y, used=1, residual_m=0.0,
                   separation_deg=0.0)

    def _publish_fix(self, x_m: float, y_m: float,
                     sigma: float | None = None, camera: str | None = None) -> None:
        """Hand a resected pool position to the invariant filter.

        A topic rather than a direct call, because the filter runs in its own
        process on its own thread and the mission is a client. Lazily created,
        same pattern as `_publish_active_camera`.
        """
        try:
            from geometry_msgs.msg import PointStamped
            if getattr(self, '_fix_pub', None) is None:
                self._fix_pub = self.client.node.create_publisher(
                    PointStamped, '/mongla/localization/fix', 10)
            m = PointStamped()
            m.header.stamp = self.client.node.get_clock().now().to_msg()
            # When the fix came from a camera frame, stamp it with THAT frame's
            # capture time: the hull was there then, not at publish time.
            cap = self.__dict__.get('_det_capture', {}).get(camera) if camera else None
            if cap and (cap[0] or cap[1]):
                m.header.stamp.sec, m.header.stamp.nanosec = cap
            m.header.frame_id = 'pool'
            m.point.x = float(x_m)
            m.point.y = float(y_m)
            # z carries the sigma. PointStamped has no covariance field and
            # inventing a message type for one float is worse than documenting
            # this; the filter reads it back the same way. <= 0 means "use the
            # node's default", so a caller that has no sigma is not forced to
            # invent one.
            m.point.z = float(sigma if sigma is not None else 0.0)
            self._fix_pub.publish(m)
        except Exception as exc:            # noqa: BLE001 -- best-effort
            self.log.warning(f'[FIX  ] fix not published to the filter: {exc}')

    def focal_px(self, width_px: float = 640.0, *, camera: str | None = None) -> float:
        """In-water focal length of `camera`, DERIVED from its field of view.

        `fx = (W/2) / tan(HFOV/2)` with the camera's own in-water FOV
        (`hfov_water_deg`). For the downward camera at 640 px that is 741 px,
        which the flow node independently reports; the forward camera is 634 px.
        """
        import math as _math
        return (float(width_px) / 2.0) / _math.tan(
            _math.radians(WhereMixin.hfov_water_deg(self, camera) / 2.0))

    def use_course(self, name: str):
        """Load the prop priors for a course. Returns the `Course`.

        The prior half of perceive-then-move: dead-reckon to roughly where a
        prop should be and let perception take the last few metres. A mission
        that searches for every prop from scratch spends its run turning.

        Deck-first search (`MONGLA_COURSE_DIR`, then `~/.mongla/courses`, then
        the package), so a course re-measured at the venue beats one committed
        months earlier without a rebuild. Unmeasured props are named in the log
        rather than left to be discovered by driving to them::

            mongla.use_course('robosub26')
            mongla.anchor_on('gate')          # the course supplies the bearing
        """
        from mongla_localization.course_map import load_course

        course = self._course = load_course(name)
        unmeasured = course.unmeasured()
        self.log.info(f'[CRSE ] {course.name} loaded from {course.source} '
                      f'({len(course.props)} props)')
        if unmeasured:
            self.log.warning(
                f'[CRSE ] NOT measured for this pool: {", ".join(unmeasured)}. '
                f'Those props refuse a dead-reckon and the mission must search '
                f'for them.')
        return course

    def anchor_on(self, prop: str, *, camera: str | None = None,
                  timeout: float = 6.0):
        """Anchor the heading on a course prop, taking its bearing from the map.

        ⛔ ONE NUMBER, ONE PLACE. A prop's face bearing is what an absolute
        heading needs AND what approaching it from the front needs. Passing it
        by hand at each call site is how the two come to disagree, so this
        reads it from the loaded course and refuses if it was never measured --
        a guessed bearing writes a wrong heading zero, and every later turn
        inherits it.
        """
        course = getattr(self, '_course', None)
        if course is None:
            raise RuntimeError(
                'anchor_on() needs a course: call use_course(<name>) first')
        bearing = course.bearing_of(prop)      # raises when unmeasured
        return self.anchor_heading(bearing_deg=bearing, camera=camera,
                                   timeout=timeout)

    def anchor_heading(self, *, bearing_deg: float, camera: str | None = None,
                       timeout: float = 6.0) -> object:
        """Re-zero the heading against a prop whose world bearing is known.

        ⛔ WHAT THIS BUYS. The BNO is deliberately magnetometer-free, so its yaw
        is relative to wherever the board booted: drift under 0.01 deg/min, and
        an arbitrary zero. Every mission that says `turn(90)` means 90 from
        boot, so a hull powered on at a different angle flies a different course
        from the same file. A prop the rulebook fixes the orientation of is a
        heading reference already in the pool.

        `bearing_deg` is the compass bearing the prop's FACE points along, which
        is per-COURSE knowledge -- where this gate was installed -- and not a
        property of the class, so it is passed in rather than read from the
        committed geometry table::

            a = mongla.anchor_heading(bearing_deg=270)   # gate faces west
            if a.ok:
                mongla.turn(mongla.absolute_to_relative(90))   # true east

        Consumes the FUSED pose, never a raw one: the planar flip would write a
        heading wrong by twice the off-axis angle, and every later turn would
        inherit it. Returns an `Anchor` whose `ok` is False with a stated
        reason rather than a number, and does NOT change what `turn()` means --
        a mission opts in by converting explicitly.
        """
        from mongla_localization.heading_anchor import anchor_from

        cam = str(camera or self.camera).strip().lower()
        fused = self._wait_fused_pose(cam, timeout=timeout)
        if fused is None:
            from mongla_localization.heading_anchor import Anchor
            return Anchor(False, reason=f'no fused pose on {cam} within '
                                        f'{timeout:.0f}s')
        got = anchor_from(fused, self.head(), float(bearing_deg))
        if got.ok:
            self._heading_offset = got.offset_deg
            # TELL THE FILTER. Until this line the anchor was a mission-local
            # variable: `absolute_heading()` used it, `bearing_to()` used it,
            # and the estimator -- the one component whose whole output is
            # labelled with a frame -- never heard about it. Its attitude came
            # from the board, which is boot-relative or magnetic, so
            # `/mongla/odom` claimed a pool frame it had no way to be in, and
            # flow would have dead-reckoned off along that unmeasured offset.
            # Same no-caller defect as the filter itself, one layer up.
            self._publish_heading(got.absolute_deg)
            self.log.info(
                f'[ANCH ] heading anchored: relative {self.head():.1f} is truly '
                f'{got.absolute_deg:.1f} (offset {got.offset_deg:+.1f}, '
                f'{got.support} frames, rule={got.rule})')
        else:
            self.log.warning(f'[ANCH ] heading NOT anchored: {got.reason}')
        return got

    def _publish_heading(self, absolute_deg: float) -> None:
        """Hand the anchored world heading to the invariant filter.

        Latched, because the anchor is a one-shot event and the filter may
        start after the mission does -- a VOLATILE publish would be heard by
        nobody and leave the estimator in the boot frame while every log line
        said it was anchored.
        """
        try:
            from std_msgs.msg import Float32
            if getattr(self, '_heading_pub', None) is None:
                qos = QoSProfile(
                    depth=1,
                    reliability=QoSReliabilityPolicy.RELIABLE,
                    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
                self._heading_pub = self.client.node.create_publisher(
                    Float32, '/mongla/localization/heading', qos)
            self._heading_pub.publish(Float32(data=float(absolute_deg)))
        except Exception as exc:            # noqa: BLE001 -- best-effort
            self.log.warning(f'[ANCH ] heading not published to the filter: {exc}')

    def absolute_heading(self) -> float:
        """The hull's heading in WORLD terms, or the relative one if unanchored.

        Passing through unanchored is deliberate: a mission written against
        boot-relative headings keeps working exactly as before, and silently
        changing what every existing `turn()` means would be far more dangerous
        than making the conversion explicit.
        """
        from mongla_localization.heading_anchor import apply_offset
        return apply_offset(self.head(), getattr(self, '_heading_offset', None))

    def absolute_to_relative(self, absolute_deg: float) -> float:
        """A WORLD heading -> the number to hand `turn()`.

        `turn()` speaks the hull's own relative frame and this does not change
        that. Unanchored, it is the identity -- so a mission can be written in
        world headings and still run on a hull that never saw its landmark,
        just without the correction.
        """
        off = getattr(self, '_heading_offset', None)
        if off is None or off != off:
            return float(absolute_deg)
        return float(absolute_deg) - float(off)

    def _wait_fused_pose(self, camera: str, *, timeout: float):
        """Latest DECIDED fused pose for `camera`, or None.

        Subscribes lazily: a mission that never anchors pays nothing, and a
        stack launched without `lock:=true` has no such topic to subscribe to.
        """
        from mongla_interfaces.msg import TargetPose

        subs = getattr(self, '_fused_subs', None)
        if subs is None:
            subs = self._fused_subs = {}
            self._fused_pose = {}
        if camera not in subs:
            topic = f'/mongla/vision/{camera}/target_pose_fused'
            # Stored WITH its arrival instant (monotonic): the fuser publishes on
            # every input pose, so silence means the camera or solver stopped.
            subs[camera] = self.client.node.create_subscription(
                TargetPose, topic,
                lambda msg, c=camera: self._fused_pose.__setitem__(
                    c, (msg, _time.monotonic())), 10)
            self.log.info(f'[ANCH ] listening on {topic}')
        from mongla_localization.pose_cluster import Fused

        def fresh():
            """The cached message if it arrived within FUSED_POSE_MAX_AGE_S.

            ⛔ ISSUE #57: this returned whatever was cached, however old. After
            the camera or solver stopped, `anchor_heading` paired a fused yaw
            from minutes ago with the hull's heading NOW -- a heading zero off
            by however far the hull had turned since.
            """
            entry = self._fused_pose.get(camera)
            if entry is None:
                return None
            msg, arrived = entry
            return msg if _time.monotonic() - arrived <= FUSED_POSE_MAX_AGE_S else None

        deadline = _time.monotonic() + float(timeout)
        while _time.monotonic() < deadline:
            rclpy.spin_once(self.client.node, timeout_sec=0.05)
            msg = fresh()
            if msg is not None and msg.ok:
                return Fused(decided=True, yaw_deg=float(msg.yaw_deg),
                             range_m=float(msg.range_m),
                             support=int(msg.n_points),
                             spread_deg=float(msg.yaw_spread_deg),
                             rule=str(msg.reason))
        # ⛔ A REFUSAL IS AN ANSWER. The fuser publishes ok=False WITH the
        # reason -- for a mirror pair, both candidate yaws (issue #55). Timing
        # out as "no fused pose" would throw that away; hand it back instead.
        msg = fresh()
        if msg is not None:
            return Fused(decided=False, reason=str(msg.reason))
        entry = self._fused_pose.get(camera)
        if entry is not None:
            return Fused(decided=False, reason=(
                f'last fused pose on {camera} is '
                f'{_time.monotonic() - entry[1]:.1f}s old (> '
                f'{FUSED_POSE_MAX_AGE_S:.1f}s): the camera or solver stopped'))
        return None
