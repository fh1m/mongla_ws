#!/usr/bin/env python3

"""Scored elements the sim did not model: style points, the coin flip, gate side.

The sim could show you a run. It could not tell you what the run SCORED, so
the parts of the rulebook that are pure judging -- and that a mission must be
written around -- were never exercised. This node watches ground truth and
keeps the score, so a practice run ends with a number rather than an opinion.

STYLE POINTS. Handbook p. 32, verbatim:

    "Teams can gain extra points by passing through the gate with 'style'. For
     every 90 degree change in orientation, the AUV increases the accumulated
     points. However, returning to the last previous orientation won't count.
     I.e. an AUV that rolls 90 degrees and then back to 0 would not get
     points. Roll and Pitch are worth more than Yaw."

That second sentence is the whole difficulty, and it is why this is not simply
"count 90 degree steps": a roll to 90 and back to 0 is TWO 90-degree changes
and scores ONE. The rule is a walk over quadrants where stepping back to where
you just were is worth nothing, so the scorer has to remember the previous
quadrant per axis, not just the current one.

THE COIN FLIP. Handbook p. 32: heads puts the vehicle "approximately parallel
to the gate", tails puts it "with its tail approximately facing the gate (the
AUV is backward)". Extra points for accepting it -- and a mission that has
quietly hard-coded its start heading fails the moment it is taken up. This
node can perform the flip and place the vehicle, so that assumption breaks in
practice instead of at the competition.

TORPEDOES (handbook p. 36), verbatim:

    "Points are awarded for firing torpedoes through any opening. A torpedo
     must pass through the opening for full points. Partial points are awarded
     if the torpedo touches the board without passing through. ... Additional
     points are awarded for firing torpedoes further away from the board. The
     'far' distance is denoted by the horizontal bars at the bottom of the
     board."

So a shot is graded on three things the sim could not previously observe:
whether it went THROUGH, which opening, and how far the vehicle was when it
fired. Each fired round is tracked from the muzzle until it stops, and the
range at the moment of firing is recorded -- not at impact, because the vehicle
may drift while the round is in flight and it is the firing position the rule
names.

BINS: a dropper scores by landing inside the bin, so its resting position is
tested against the bin footprint, again after it has actually settled.

GATE SIDE. "The AUV chooses a marine animal by passing under a specific side",
so the side taken IS the role for the rest of the run. The transit watcher
reports which half the vehicle passed through and at what depth.

SAUVC FLARE SEQUENCE. Task 4 hands the team a colour order topside and the
vehicle must bump the flares in it. Order is the entire task -- hitting all
three in the wrong sequence is not a partial pass -- so scoring it needs the
order recorded, which nothing did. A flare counts as bumped when its golf ball
leaves the cup, which is the same physical event the judges watch.

NOT MODELLED: magnetically-activated bin lights. They are not in the 2025
handbook, so there is no rule text to implement against and inventing one
would give a confidently wrong number.
"""

from __future__ import annotations

import json
import math
import os
import random
import subprocess
import threading
import time

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from std_msgs.msg import Int32, String



# Handbook: "Roll and Pitch are worth more than Yaw." It does not publish the
# split, so these are OUR weighting and are exposed as parameters rather than
# buried -- a scorecard that looks official but invented its own numbers is
# worse than one that says which numbers it chose.
STYLE_POINTS_ROLL = 2.0
STYLE_POINTS_PITCH = 2.0
STYLE_POINTS_YAW = 1.0

QUADRANT_DEG = 90.0
# How close to the NEW cardinal the vehicle must get before the change counts.
# Not a noise filter -- a correctness one. The quadrant boundary sits at 45
# degrees, so without this a vehicle that rolled to 60 and stopped would score
# a full "90 degree change in orientation" for a 60 degree move. 25 degrees
# means the hull must reach 65 of the 90 before it counts -- recognisably on
# its side -- and it also stops a vehicle hovering on the boundary from
# ratcheting the score.
QUADRANT_ARRIVAL_DEG = 25.0

SCORE_TOPIC = '/duburi/sim/score'


def _quadrant(angle_deg: float) -> int:
    """Which 90-degree sector an angle sits in, 0..3."""
    return int(((angle_deg % 360.0) + QUADRANT_DEG / 2.0) % 360.0 // QUADRANT_DEG)


class AxisStyle:
    """Quadrant walk for one axis, with the 'no going back' rule."""

    def __init__(self, points: float) -> None:
        self.points = points
        self.current = None
        self.previous = None
        self.score = 0.0
        self.changes = 0
        self.rejected_returns = 0

    def update(self, angle_deg: float) -> float:
        q = _quadrant(angle_deg)
        if self.current is None:
            self.current = q
            return 0.0
        if q == self.current:
            return 0.0
        # Hysteresis: only accept the change once the angle is properly inside
        # the new quadrant, so noise on a boundary cannot ratchet the score.
        centre = q * QUADRANT_DEG
        if abs((angle_deg - centre + 180.0) % 360.0 - 180.0) > QUADRANT_ARRIVAL_DEG:
            return 0.0
        gained = 0.0
        if q == self.previous:
            # "returning to the last previous orientation won't count"
            self.rejected_returns += 1
        else:
            gained = self.points
            self.score += gained
            self.changes += 1
        self.previous = self.current
        self.current = q
        return gained

    def snapshot(self) -> dict:
        return {'score': self.score, 'changes': self.changes,
                'rejected_returns': self.rejected_returns,
                'quadrant': self.current}


class Scoring(Node):
    def __init__(self) -> None:
        super().__init__('scoring')
        self.declare_parameter('world', 'robosub26_full')
        self.declare_parameter('vehicle', 'duburi')
        self.declare_parameter('style_roll', STYLE_POINTS_ROLL)
        self.declare_parameter('style_pitch', STYLE_POINTS_PITCH)
        self.declare_parameter('style_yaw', STYLE_POINTS_YAW)
        # Gate geometry, so the transit watcher knows what it is watching.
        self.declare_parameter('gate_x', -5.0)
        self.declare_parameter('gate_y', 0.0)
        self.declare_parameter('gate_width', 3.0)
        # Coin flip: '' leaves the vehicle alone, 'flip' draws, or force
        # 'heads'/'tails' to rehearse one.
        self.declare_parameter('coin', '')
        self.declare_parameter('seed', 0)
        # SAUVC flare sequence: the colour order handed to the team topside.
        # Empty disables the check (RoboSub courses have no flares).
        self.declare_parameter('flare_sequence', [''])
        self.declare_parameter('flare_drop_m', 0.25)
        # Torpedo board. Openings are (y OFFSET from board_y, z in world,
        # radius) -- an offset rather than absolute y because the board is
        # placed with yaw = pi in robosub26_full, which mirrors its own y, and
        # an absolute figure silently describes the wrong hole. Defaults match
        # that course: large opening high and to one side, small low and the
        # other.
        self.declare_parameter('board_x', 8.0)
        self.declare_parameter('board_y', 3.0)
        self.declare_parameter('board_openings',
                               [0.15, -1.10, 0.10, -0.15, -1.40, 0.065])
        self.declare_parameter('standoff_far', 1.0)
        self.declare_parameter('standoff_farther', 1.5)
        # Bin footprint for a dropper, "24 in x 12 in" (610 x 305 mm).
        self.declare_parameter('bin_x', 0.0)
        self.declare_parameter('bin_y', 0.0)
        self.declare_parameter('bin_size', [0.61, 0.305])
        self.declare_parameter('score_payload', True)

        self._world = self.get_parameter('world').value
        self._vehicle = self.get_parameter('vehicle').value
        self._rng = random.Random(
            int(self.get_parameter('seed').value) or None)

        self._roll = AxisStyle(float(self.get_parameter('style_roll').value))
        self._pitch = AxisStyle(float(self.get_parameter('style_pitch').value))
        self._yaw = AxisStyle(float(self.get_parameter('style_yaw').value))

        self._pose = None
        self._side = None          # which half of the gate was taken
        self._transits = []
        self._prev_x = None
        self._coin = None

        seq = [c.strip().lower()
               for c in (self.get_parameter('flare_sequence').value or [])
               if c and c.strip()]
        self._flare_expected = seq
        self._flare_hits = []
        self._flare_home = {}
        self._ball_z_cache = {}
        # Fired rounds, keyed by model name, tracked from muzzle to rest.
        self._shots = {}
        self._shot_results = []
        self._shot_z_cache = {}

        self._gz_connect()
        self._pub = self.create_publisher(String, SCORE_TOPIC, 10)
        # The timer runs unconditionally and no-ops on an empty sequence. The
        # colour order is handed to the team TOPSIDE, minutes before the run,
        # so it has to be settable at runtime -- reading it once at
        # construction made `ros2 param set` return "successful" and do
        # nothing, which is the worst of both.
        self.create_timer(0.5, self._check_flares)
        self.create_timer(0.5, self._score_shots)
        self.create_subscription(Int32, '/duburi/sim/payload/fired',
                                 self._on_fired, 10)
        if seq:
            self.get_logger().info(
                f'[SCORE] flare sequence to hit: {" -> ".join(seq)}')
        self.add_on_set_parameters_callback(self._on_params)
        self.create_timer(0.5, self._publish)
        self.create_timer(10.0, self._report)

        coin = str(self.get_parameter('coin').value or '').strip().lower()
        if coin:
            self.create_timer(2.0, lambda: self._do_coin_once(coin))

        self.get_logger().info(
            '[SCORE] watching style / gate transit '
            f'(roll={self._roll.points} pitch={self._pitch.points} '
            f'yaw={self._yaw.points} per 90 deg)')

    # -- truth -------------------------------------------------------------

    def _gz_connect(self) -> None:
        try:
            from gz.msgs10.odometry_pb2 import Odometry
            from gz.transport13 import Node as GzNode
        except ImportError:
            self.get_logger().error('[SCORE] no gz-transport bindings')
            self._gz = None
            return
        self._gz = GzNode()
        if not self._gz.subscribe(Odometry, f'/model/{self._vehicle}/odometry',
                                  self._on_odom):
            self.get_logger().error('[SCORE] could not subscribe to odometry')
        self._stop_stream = False
        self._pose_proc = None
        threading.Thread(target=self._pose_stream, name='pose-stream',
                         daemon=True).start()

    def _on_odom(self, msg) -> None:
        p, q = msg.pose.position, msg.pose.orientation
        # Full roll/pitch/yaw, because style is scored on all three and roll
        # and pitch are worth the most.
        sinr = 2.0 * (q.w * q.x + q.y * q.z)
        cosr = 1.0 - 2.0 * (q.x ** 2 + q.y ** 2)
        roll = math.degrees(math.atan2(sinr, cosr))
        sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
        pitch = math.degrees(math.asin(sinp))
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y ** 2 + q.z ** 2)
        yaw = math.degrees(math.atan2(siny, cosy))

        self._pose = (p.x, p.y, p.z, roll, pitch, yaw)
        for axis, angle in ((self._roll, roll), (self._pitch, pitch),
                            (self._yaw, yaw)):
            gained = axis.update(angle)
            if gained:
                self.get_logger().info(
                    f'[SCORE] +{gained:.0f} style (total '
                    f'{self.style_total:.0f})')
        self._check_transit(p.x, p.y, p.z)

    # -- gate --------------------------------------------------------------

    def _check_transit(self, x: float, y: float, z: float) -> None:
        gx = float(self.get_parameter('gate_x').value)
        gy = float(self.get_parameter('gate_y').value)
        half = float(self.get_parameter('gate_width').value) / 2.0
        if self._prev_x is None:
            self._prev_x = x
            return
        crossed = (self._prev_x - gx) * (x - gx) < 0.0
        self._prev_x = x
        if not crossed or abs(y - gy) > half:
            return
        # "The AUV chooses a marine animal by passing under a specific side."
        side = 'port' if y < gy else 'starboard'
        self._side = side
        self._transits.append({'side': side, 'depth_m': round(z, 3),
                               'y_m': round(y - gy, 3)})
        self.get_logger().warn(
            f'[SCORE] GATE TRANSIT on the {side} side at {z:.2f} m '
            f'({y - gy:+.2f} m off centre) -- that is the role for this run')

    # -- SAUVC flares ------------------------------------------------------

    def _pose_stream(self) -> None:
        """Every dynamic model's pose, as one consistent stream.

        ORDER IS THE ENTIRE TASK, so this cannot be polled per ball. The first
        version shelled out to `gz model -m <ball> -p` once per flare inside a
        0.5 s timer -- three blocking subprocesses that together outlasted the
        timer period, so detections came out in POLLING order rather than the
        order the balls actually fell. Measured: knocking red, then yellow,
        then blue scored as yellow, red, blue and reported a clean pass. A
        false pass on the one thing the task grades.

        A gz-transport subscription would be the obvious fix and does not
        work: `Pose_V`'s repeated `Pose` field fails to resolve in the Python
        bindings' descriptor pool ("No message class registered for
        'gz.msgs.Pose'"), thrown inside gz's own callback thread where it
        prints a traceback and is otherwise swallowed -- a subscription that
        looks alive and delivers nothing. Importing `pose_pb2` first, at
        module scope, does not help.

        So: one long-lived `gz topic -e --json-output`, which is a single
        consistent snapshot per message and needs no descriptor pool at all.
        """
        cmd = ['gz', 'topic', '-e', '--json-output', '-t',
               f'/world/{self._world}/dynamic_pose/info']
        while not self._stop_stream:
            try:
                proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                        stderr=subprocess.DEVNULL, text=True)
            except OSError as exc:
                self.get_logger().warn(f'[SCORE] pose stream: {exc}')
                return
            self._pose_proc = proc
            for line in proc.stdout:
                if self._stop_stream:
                    break
                line = line.strip()
                if not line.startswith('{'):
                    continue
                try:
                    msg = json.loads(line)
                except ValueError:
                    continue
                for pose in msg.get('pose', []):
                    name = pose.get('name', '')
                    if name.startswith('flare_') and name.endswith('_ball'):
                        z = (pose.get('position') or {}).get('z')
                        if z is not None:
                            self._ball_z_cache[name] = float(z)
                    elif name.startswith('payload_shot_'):
                        pos = pose.get('position') or {}
                        self._shot_z_cache[name] = (
                            float(pos.get('x', 0.0)), float(pos.get('y', 0.0)),
                            float(pos.get('z', 0.0)))
            proc.wait()
            if not self._stop_stream:
                time.sleep(1.0)

    def _set_sequence(self, colours) -> None:
        seq = [c.strip().lower() for c in (colours or []) if c and c.strip()]
        if seq == self._flare_expected:
            return
        self._flare_expected = seq
        self._flare_hits = []
        self._flare_home = {}
        self.get_logger().warn(
            f'[SCORE] flare sequence set: {" -> ".join(seq) or "(none)"}')

    def _check_flares(self) -> None:
        """A flare is bumped when its golf ball leaves the cup.

        By HEIGHT against where the ball started, never by whether it is
        moving: a ball resting on the pool floor is perfectly stable, and a
        stability check passed once while all three were already down.
        """
        if not self._flare_expected:
            return
        drop = float(self.get_parameter('flare_drop_m').value)
        for colour in self._flare_expected:
            name = f'flare_{colour}_ball'
            if colour in [h['colour'] for h in self._flare_hits]:
                continue
            z = self._ball_z_cache.get(name)
            if z is None:
                continue
            if colour not in self._flare_home:
                self._flare_home[colour] = z
                continue
            if self._flare_home[colour] - z > drop:
                order = len(self._flare_hits)
                expected = self._flare_expected[order]
                ok = expected == colour
                self._flare_hits.append({
                    'colour': colour, 'position': order + 1,
                    'in_order': ok, 't': round(time.time(), 1),
                })
                self.get_logger().warn(
                    f'[SCORE] FLARE {colour.upper()} bumped '
                    f'({order + 1} of {len(self._flare_expected)}) -- '
                    + ('in order' if ok else
                       f'OUT OF ORDER, expected {expected}'))


    # -- fired rounds ------------------------------------------------------

    def _on_fired(self, msg) -> None:
        """A payload channel fired. Record the range NOW, not at impact.

        The rule names the firing position -- "firing torpedoes further away
        from the board" -- and the vehicle can drift several centimetres while
        a round is in flight, which is the difference between "far" and not on
        a 1.0 m boundary.
        """
        if not self.get_parameter('score_payload').value or self._pose is None:
            return
        channel = int(msg.data)
        kind = 'torpedo' if channel in (1, 2) else 'dropper'
        x, y, z = self._pose[0], self._pose[1], self._pose[2]
        if kind == 'torpedo':
            bx = float(self.get_parameter('board_x').value)
            by = float(self.get_parameter('board_y').value)
            rng = math.hypot(bx - x, by - y)
        else:
            bx = float(self.get_parameter('bin_x').value)
            by = float(self.get_parameter('bin_y').value)
            rng = math.hypot(bx - x, by - y)
        self._pending_shot = {
            'channel': channel, 'kind': kind,
            'fired_range_m': round(rng, 3),
            'fired_from': [round(x, 3), round(y, 3), round(z, 3)],
            't': time.monotonic(), 'name': None, 'settled': False,
        }
        self.get_logger().info(
            f'[SCORE] {kind} fired at {rng:.2f} m from the target')

    def _adopt_new_shots(self) -> None:
        """Attach the pending fire record to whichever model just appeared."""
        pending = getattr(self, '_pending_shot', None)
        if pending is None:
            return
        for name in self._shot_z_cache:
            if name not in self._shots:
                pending['name'] = name
                self._shots[name] = pending
                self._pending_shot = None
                return

    def _score_shots(self) -> None:
        """Grade every round that has come to rest."""
        self._adopt_new_shots()
        for name, shot in list(self._shots.items()):
            if shot['settled']:
                continue
            pos = self._shot_z_cache.get(name)
            if pos is None:
                continue
            last = shot.get('last_pos')
            shot['last_pos'] = pos
            if last is None:
                continue
            moved = math.dist(pos, last)
            shot['still_for'] = (shot.get('still_for', 0.0) + 0.5
                                 if moved < 0.01 else 0.0)
            # A round in flight passes the board plane; record the crossing
            # while it happens, because where it ENDS says nothing about
            # whether it went through.
            if shot['kind'] == 'torpedo':
                bx = float(self.get_parameter('board_x').value)
                if (last[0] - bx) * (pos[0] - bx) < 0.0:
                    shot['crossed_plane'] = True
                    shot['crossing'] = (pos[1], pos[2])
            if shot['still_for'] < 1.0:
                continue
            shot['settled'] = True
            self._grade(shot, pos)

    def _grade(self, shot: dict, rest) -> None:
        if shot['kind'] == 'torpedo':
            result = self._grade_torpedo(shot)
        else:
            result = self._grade_dropper(shot, rest)
        result.update({k: shot[k] for k in ('channel', 'kind',
                                            'fired_range_m', 'fired_from')})
        far = float(self.get_parameter('standoff_far').value)
        farther = float(self.get_parameter('standoff_farther').value)
        rng = shot['fired_range_m']
        result['distance_band'] = ('farther' if rng >= farther
                                   else 'far' if rng >= far else 'near')
        self._shot_results.append(result)
        self.get_logger().warn(
            f"[SCORE] {shot['kind'].upper()} {result['outcome'].upper()} "
            f"-- fired from {rng:.2f} m ({result['distance_band']})"
            + (f", opening {result['opening']}" if result.get('opening')
               else ''))

    def _grade_torpedo(self, shot: dict) -> dict:
        """Through an opening, or merely on the board?"""
        if not shot.get('crossed_plane'):
            return {'outcome': 'miss', 'opening': None,
                    'note': 'never reached the board plane'}
        y, z = shot['crossing']
        vals = list(self.get_parameter('board_openings').value or [])
        openings = [vals[i:i + 3] for i in range(0, len(vals) - 2, 3)]
        by = float(self.get_parameter('board_y').value)
        for idx, (oy, oz, r) in enumerate(openings):
            if math.hypot(y - (by + oy), z - oz) <= r:
                return {'outcome': 'through',
                        'opening': 'large' if idx == 0 else 'small',
                        'miss_dist_m': 0.0}
        # It crossed the plane outside every opening. With the board's
        # collision plate now genuinely holed, that means it went past the
        # edge; a round that STRUCK the board never crosses at all.
        nearest = min((math.hypot(y - (by + oy), z - oz) - r)
                      for oy, oz, r in openings) if openings else None
        return {'outcome': 'past_board', 'opening': None,
                'miss_dist_m': round(nearest, 3) if nearest else None}

    def _grade_dropper(self, shot: dict, rest) -> dict:
        bx = float(self.get_parameter('bin_x').value)
        by = float(self.get_parameter('bin_y').value)
        sx, sy = list(self.get_parameter('bin_size').value)[:2]
        inside = abs(rest[0] - bx) <= sx / 2.0 and abs(rest[1] - by) <= sy / 2.0
        return {
            'outcome': 'in_bin' if inside else 'outside_bin',
            'rest': [round(v, 3) for v in rest],
            'miss_dist_m': round(math.hypot(rest[0] - bx, rest[1] - by), 3),
        }

    # -- coin flip ---------------------------------------------------------

    def _do_coin_once(self, mode: str) -> None:
        if self._coin is not None:
            return
        self._coin = (self._rng.choice(('heads', 'tails'))
                      if mode == 'flip' else mode)
        if self._pose is None:
            self.get_logger().warn('[SCORE] coin flip deferred -- no pose yet')
            self._coin = None
            return
        x, y, z = self._pose[0], self._pose[1], self._pose[2]
        gx = float(self.get_parameter('gate_x').value)
        gy = float(self.get_parameter('gate_y').value)
        facing = math.atan2(gy - y, gx - x)
        # heads: "approximately parallel to the gate"; tails: "tail
        # approximately facing the gate (the AUV is backward)".
        yaw = (facing + math.pi / 2.0 if self._coin == 'heads'
               else facing + math.pi)
        # A few degrees of slop, because "approximately" is the rulebook's own
        # word and a mission must not depend on an exact placement.
        yaw += math.radians(self._rng.uniform(-8.0, 8.0))
        self._teleport(x, y, z, yaw)
        self.get_logger().warn(
            f'[SCORE] COIN FLIP: {self._coin.upper()} -- vehicle placed at '
            f'yaw {math.degrees(yaw):.1f} deg. A mission that hard-codes its '
            f'start heading will fail from here.')

    def _teleport(self, x, y, z, yaw) -> None:
        req = (f'name: "{self._vehicle}", position: {{x: {x}, y: {y}, z: {z}}}, '
               f'orientation: {{x: 0, y: 0, z: {math.sin(yaw / 2.0)}, '
               f'w: {math.cos(yaw / 2.0)}}}')
        subprocess.run(
            ['gz', 'service', '-s', f'/world/{self._world}/set_pose',
             '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
             '--timeout', '3000', '--req', req],
            check=False, capture_output=True)

    # -- plumbing ----------------------------------------------------------

    @property
    def style_total(self) -> float:
        return self._roll.score + self._pitch.score + self._yaw.score

    def _on_params(self, params) -> SetParametersResult:
        for p in params:
            if p.name == 'flare_sequence':
                self._set_sequence(p.value)
            if p.name == 'coin' and str(p.value or '').strip():
                self._coin = None
                self._do_coin_once(str(p.value).strip().lower())
        return SetParametersResult(successful=True)

    def snapshot(self) -> dict:
        return {
            'style': {
                'total': self.style_total,
                'roll': self._roll.snapshot(),
                'pitch': self._pitch.snapshot(),
                'yaw': self._yaw.snapshot(),
            },
            'gate': {'side': self._side, 'transits': self._transits},
            'coin': self._coin,
            'shots': self._shot_results,
            'flares': {
                'expected': self._flare_expected,
                'hits': self._flare_hits,
                'sequence_correct': (
                    bool(self._flare_expected)
                    and len(self._flare_hits) == len(self._flare_expected)
                    and all(h['in_order'] for h in self._flare_hits)),
            },
        }

    def _publish(self) -> None:
        self._pub.publish(String(data=json.dumps(self.snapshot())))

    def _report(self) -> None:
        s = self.snapshot()
        self.get_logger().info(
            f"[SCORE] style {s['style']['total']:.0f} "
            f"(roll {s['style']['roll']['changes']} "
            f"pitch {s['style']['pitch']['changes']} "
            f"yaw {s['style']['yaw']['changes']} changes, "
            f"{sum(s['style'][k]['rejected_returns'] for k in ('roll','pitch','yaw'))} "
            f"returns rejected) | gate {s['gate']['side'] or 'not transited'}")


    def destroy_node(self) -> bool:
        # Leave no orphan `gz topic` behind: it survives this node and keeps a
        # subscription open for the rest of the session.
        self._stop_stream = True
        if getattr(self, '_pose_proc', None):
            try:
                self._pose_proc.terminate()
            except OSError:
                pass
        return super().destroy_node()


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = Scoring()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
