"""The course and the heading: load the prop priors, and anchor the boot-relative yaw on one.

Mixed into `MonglaMission` (mongla_dsl.py), which owns the client, the node
and the caches these methods read. `WhereMixin` (dsl_position.py) reads
`self._course`, which `use_course` sets here.
"""
from __future__ import annotations

import time as _time

import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from .dsl_support import FUSED_POSE_MAX_AGE_S


class HeadingMixin:
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

        One number, one place. A prop's face bearing is what an absolute
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
        # The bearing is THIS prop's, so the pose must be too: a fused pose of
        # another prop would write a heading zero off by the angle between
        # their faces (#76). Unknown when the course names no detect class.
        return self.anchor_heading(bearing_deg=bearing, camera=camera,
                                   timeout=timeout,
                                   target_class=course.prop(prop).detect_class or '')

    def anchor_heading(self, *, bearing_deg: float, camera: str | None = None,
                       timeout: float = 6.0, target_class: str = '') -> object:
        """Re-zero the heading against a prop whose world bearing is known.

        What this buys. The BNO is deliberately magnetometer-free, so its yaw
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

        `target_class` restricts it to a pose solved for that class (#76);
        `anchor_on` always passes the course's own `detect_class`.
        """
        from mongla_localization.heading_anchor import anchor_from

        cam = str(camera or self.camera).strip().lower()
        fused = self._wait_fused_pose(cam, timeout=timeout,
                                      target_class=target_class)
        if fused is None:
            from mongla_localization.heading_anchor import Anchor
            return Anchor(False, reason=f'no fused pose on {cam} within '
                                        f'{timeout:.0f}s')
        got = anchor_from(fused, self.head(), float(bearing_deg))
        if got.ok:
            self._heading_offset = got.offset_deg
            # Tell the filter. Until this line the anchor was a mission-local
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

    def _wait_fused_pose(self, camera: str, *, timeout: float,
                         target_class: str = ''):
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

            ISSUE #57: this returned whatever was cached, however old. After
            the camera or solver stopped, `anchor_heading` paired a fused yaw
            from minutes ago with the hull's heading NOW -- a heading zero off
            by however far the hull had turned since.
            """
            entry = self._fused_pose.get(camera)
            if entry is None:
                return None
            msg, arrived = entry
            if target_class and getattr(msg, 'target_label', '') != target_class:
                return None                     # another prop's pose (#76)
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
        # A refusal is an answer. The fuser publishes ok=False WITH the
        # reason -- for a mirror pair, both candidate yaws (issue #55). Timing
        # out as "no fused pose" would throw that away; hand it back instead.
        msg = fresh()
        if msg is not None:
            return Fused(decided=False, reason=str(msg.reason))
        entry = self._fused_pose.get(camera)
        label = getattr(entry[0], 'target_label', '') if entry is not None else ''
        if entry is not None and target_class and label != target_class:
            return Fused(decided=False, reason=(
                f'the fused pose on {camera} is of {label or "an unlabelled target"!r}, '
                f'not {target_class!r}: refusing to anchor on another prop'))
        if entry is not None:
            return Fused(decided=False, reason=(
                f'last fused pose on {camera} is '
                f'{_time.monotonic() - entry[1]:.1f}s old (> '
                f'{FUSED_POSE_MAX_AGE_S:.1f}s): the camera or solver stopped'))
        return None
