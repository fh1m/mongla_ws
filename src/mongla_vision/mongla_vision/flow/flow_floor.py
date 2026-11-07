"""FlowVelocityNode, part 3 of 5: what the floor itself says -- tile grid heading and lane lines.
"""
import math
from sensor_msgs.msg import Range

from mongla_interfaces.msg import FloorHeading


class FlowFloorMixin:
    """Methods of `FlowVelocityNode`; see `flow_node.py` for the node."""

    def _read_the_floor(self, gray, t: float) -> None:
        """Read the tile grating, at a low rate, and keep what it says.

        Why this is not in the per-frame path. A full-frame pass costs
        28.90 ms, which is an entire frame budget at 30 Hz; decimated x3 it is
        ~1.5 ms, and height and heading move slowly enough that 2 Hz is ample.
        Running it per frame would have made a free measurement expensive.

        `tile_m` unset means OFF, and that is the default. A tile size is a
        VENUE constant -- ours is not the competition's -- and a wrong one does
        not fail, it scales every height by the ratio of the two with nothing
        logged. Off until measured.
        """
        if gray is None or (self._tile_m <= 0.0 and not self._lane_lines):
            return
        if t < self._tile_next_t:
            return
        self._tile_next_t = t + self._tile_period_s
        if self._lane_lines:
            self._read_the_lane(gray, t)
        if self._tile_m <= 0.0:
            return
        try:
            from mongla_localization.tile_grating import measure
        except Exception:                       # noqa: BLE001
            self._tile_m = 0.0                  # not installed: stop asking
            return
        g = measure(gray, decimate=self._tile_decimate)
        if g is None:
            # The common case on an untiled floor, and NOT an error. Clearing
            # rather than holding: a stale height is worse than no height,
            # because the caller cannot tell it is stale.
            self._tile_height = None
            self._tile_angle = None
            return
        f_px = self._f_px
        if not f_px or f_px <= 0.0:
            return
        self._tile_height = g.height_m(f_px, self._tile_m)
        self._tile_angle = g.heading_deg()
        if self._tile_height and math.isfinite(self._tile_height):
            r = Range()
            r.header.stamp.sec = int(t)
            r.header.stamp.nanosec = int((t - int(t)) * 1e9)
            r.header.frame_id = f'{self._cam}_cam'
            r.field_of_view = 0.0
            r.min_range = 0.0
            r.max_range = float('inf')
            r.range = float(self._tile_height)
            self._pub_floor_h.publish(r)
        self._pub_grid.publish(_floor_heading(t, self._cam, self._tile_angle))
        # Report the disagreement, never silently pick. Same rule the existing
        # optical cross-check follows: a divergence does not say WHICH input is
        # wrong, and `pool_depth_m` is the one nobody measures.
        #
        # This used to be one sample and a bare `d > 0.20`, so a single bad
        # grating read -- a caustic, a drain cover, one frame of a passing
        # shadow -- latched a warning that never cleared, and a slow genuine
        # drift that never spiked was never reported at all.
        # `flow/scale_check.py` was written for exactly this comparison, with
        # the same 0.20 bar plus a MEDIAN over at least 5 pairs, a 60 s age
        # limit so a calibration cannot outlive its conditions, and confidence
        # from the coefficient of variation. It sat orphaned beside this worse
        # inline copy.
        if self._last_height and self._tile_height:
            self._scale_x.add(self._tile_height, self._last_height)
            chk = self._scale_x.check()
            if chk.ok and chk.disagree and not self._tile_warned:
                self._tile_warned = True
                self.get_logger().warning(
                    f'[FLOW ] the FLOOR says {self._tile_height:.2f} m, the '
                    f'pool_depth path says {self._last_height:.2f} m -- '
                    f'{chk.reason} (median of {chk.samples} pairs, confidence '
                    f'{chk.confidence:.2f}). The floor needs no pool_depth_m '
                    f'and is preferred -- check pool_depth_m and the tile_m '
                    f'you set.')

    def _read_the_lane(self, gray, t: float) -> None:
        """Publish the lane line's heading, modulo 180, or nothing.

        Decimated by `tile_decimate`: 27.5 ms full-frame at 720p against
        3.4 ms at x3, with the heading unchanged to 0.01 deg -- orientation is
        invariant under isotropic scaling, so the pixels bought nothing.

        Off by default, and not because lane lines vary. World Aquatics fixes
        them. It is off because a PATH MARKER is also a dark elongated band on
        the floor, and a heading pulled toward one is wrong by exactly the
        angle the marker was laid at. Localization bounds the pull at
        `lane_max_correction_deg` and only applies it while anchored, but that
        bounds the damage; it does not remove it. Turn it on where the floor is
        known to carry lanes and no markers.
        """
        try:
            import cv2
            from mongla_localization.pool_lines import detect
        except Exception:                       # noqa: BLE001
            self._lane_lines = False            # not installed: stop asking
            return
        d = max(1, int(self._tile_decimate))
        small = gray if d == 1 else cv2.resize(
            gray, None, fx=1.0 / d, fy=1.0 / d, interpolation=cv2.INTER_AREA)
        line = detect(small)
        if line is None:
            self._lane_angle = None             # bare floor: say nothing
            return
        self._lane_angle = line.heading_deg()
        self._pub_lane.publish(_floor_heading(t, self._cam, self._lane_angle))


def _floor_heading(t: float, cam: str, angle_deg: float) -> FloorHeading:
    """A floor heading stamped with the frame it was read from (BUGS B-91)."""
    m = FloorHeading()
    m.header.stamp.sec = int(t)
    m.header.stamp.nanosec = int((t - int(t)) * 1e9)
    m.header.frame_id = f'{cam}_cam'
    m.heading_deg = float(angle_deg)
    return m
