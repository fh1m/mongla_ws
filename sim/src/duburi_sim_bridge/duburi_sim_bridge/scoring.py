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
import yaml
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from std_msgs.msg import Int32, String

from . import rulebook
from .paths import runtime_dir



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


def _course_yaml(world: str) -> dict:
    """The course definition a running world was generated from."""
    try:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory('duburi_sim_worlds'),
                            'courses', f'{world}.yaml')
        with open(path) as fh:
            return yaml.safe_load(fh) or {}
    except Exception:
        return {}


def _spec_yaml(competition: str) -> dict:
    try:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory('duburi_sim_worlds'),
                            'spec', f'{competition}.yaml')
        with open(path) as fh:
            return yaml.safe_load(fh) or {}
    except Exception:
        return {}


def _resolve_competition(world: str) -> str:
    """Which rulebook a running course is scored against.

    `competition:` is a course-yaml key that the world GENERATOR consumes and
    never writes into the .world, so at runtime there is nothing to read and
    the web UI had resorted to prefix-matching the course name. Read the course
    file back and apply the generator's own fallback chain, so the two cannot
    disagree: competition -> pool -> sauvc.
    """
    try:
        course = _course_yaml(world)
        return course.get('competition', course.get('pool', 'sauvc'))
    except Exception:
        return 'sauvc'


class Scorecard:
    """What the run has earned, against one competition's table.

    Awards are IDEMPOTENT per key: passing the gate twice is one gate score,
    and the scorer calls `award` freely from a 2 Hz timer without having to
    remember whether it already did. Items the rulebook allows more than once
    (two markers, two torpedoes, eight style rotations) carry a `repeat` cap
    and count up to it.
    """

    def __init__(self, competition: str) -> None:
        self.competition = competition
        self.rules = rulebook.book(competition)
        self._earned = {}       # key -> {'count': n, 'evidence': [...]}
        self._penalties = []
        self._bonus = {}        # computed values (timing), key -> points

    def _item(self, key):
        for task in self.rules['tasks']:
            for it in task['items']:
                if it['key'] == key:
                    return it
        return None

    def award(self, key, evidence='') -> bool:
        """Record one achievement. False if it was already at its cap."""
        it = self._item(key)
        if it is None:
            return False
        rec = self._earned.setdefault(key, {'count': 0, 'evidence': []})
        if rec['count'] >= it['repeat']:
            return False
        rec['count'] += 1
        if evidence:
            rec['evidence'].append(evidence)
        return True

    def penalise(self, key, evidence='') -> None:
        for it in self.rules['penalties']:
            if it['key'] == key:
                self._penalties.append(
                    {'key': key, 'label': it['label'],
                     'points': it['points'], 'evidence': evidence})
                return

    def set_bonus(self, key, points) -> None:
        self._bonus[key] = points

    @property
    def total(self) -> float:
        t = 0.0
        for key, rec in self._earned.items():
            it = self._item(key)
            if it is not None and it['state'] == rulebook.SCORED:
                t += it['points'] * rec['count']
        t += sum(self._bonus.values())
        t += sum(p['points'] for p in self._penalties)
        return round(t, 2)

    def snapshot(self) -> dict:
        full, reach = rulebook.maxima(self.rules)
        tasks = []
        for task in self.rules['tasks']:
            items = []
            for it in task['items']:
                rec = self._earned.get(it['key'], {'count': 0, 'evidence': []})
                items.append({
                    'key': it['key'], 'label': it['label'],
                    'points': it['points'], 'repeat': it['repeat'],
                    'state': it['state'], 'note': it['note'],
                    'count': rec['count'],
                    'evidence': rec['evidence'],
                    'earned': (self._bonus.get(it['key'])
                               if it['key'] in self._bonus
                               else it['points'] * rec['count']
                               if it['state'] == rulebook.SCORED else 0),
                })
            tasks.append({'key': task['key'], 'label': task['label'],
                          'items': items})
        return {
            'competition': self.competition,
            'name': self.rules['name'],
            'total': self.total,
            'max_rulebook': full,
            'max_reachable': reach,
            'tasks': tasks,
            'penalties': self._penalties,
        }


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
        # Pool geometry, for the contact penalties. SAUVC deducts 5 points per
        # touch of the bottom or a wall and 2 for touching the gate, and
        # nothing detected either before -- the one part of its table a
        # practice run can actually lose points on.
        self.declare_parameter('pool_length', 25.0)
        self.declare_parameter('pool_width', 16.0)
        self.declare_parameter('pool_depth', 1.6)
        self.declare_parameter('pool_edge_depth', 0.0)
        self.declare_parameter('contact_margin', 0.18)
        # 'start' / 'stop' / 'reset'. Both time bonuses need a run clock, and
        # the run does not begin when the node does.
        self.declare_parameter('run', '')

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

        self._competition = _resolve_competition(self._world)
        self._card = Scorecard(self._competition)
        self._adopt_course_geometry()
        self._run_t0 = None
        self._run_end = None
        self._touching = False
        self._touch_s = 0.0
        self._touch_n = 0
        self._last_tick = None
        self._card_path = None

        self.get_logger().info(
            f'[SCORE] scoring {self._world} against {self._card.rules["name"]}')

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
        # The run starts when the vehicle arms, because that is when a
        # competition run starts. Waiting for an operator to remember a param
        # would mean every scorecard's time bonus was quietly wrong.
        try:
            from duburi_interfaces.msg import DuburiState
            self.create_subscription(DuburiState, '/duburi/state',
                                     self._on_state, 10)
        except ImportError:
            self.get_logger().info(
                '[SCORE] duburi_interfaces not on the path -- start the run '
                "clock by hand: ros2 param set /scoring run start")
        if seq:
            self.get_logger().info(
                f'[SCORE] flare sequence to hit: {" -> ".join(seq)}')
        self.add_on_set_parameters_callback(self._on_params)
        self.create_timer(0.5, self._check_contact)
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
        # First crossing is the gate, the second is Return Home -- the same
        # gate scores twice in RoboSub and the rulebook counts them apart.
        if len(self._transits) == 1:
            self._card.award('gate_pass', f'{side} side at {z:.2f} m')
            if self._competition == 'robosub':
                self._card.award('gate_control', self._control_evidence())
                self._card.award('random_role', f'took the {side} side')
        elif len(self._transits) == 2:
            self._card.award('return_home', f'back through at {z:.2f} m')
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
                self._card.award('flare_bump', f'{colour} ({order + 1} of 3)')
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
        self._award_shot(result)
        self.get_logger().warn(
            f"[SCORE] {shot['kind'].upper()} {result['outcome'].upper()} "
            f"-- fired from {rng:.2f} m ({result['distance_band']})"
            + (f", opening {result['opening']}" if result.get('opening')
               else ''))

    def _award_shot(self, result: dict) -> None:
        """Score a graded shot against the table.

        The distance bonuses are ADDITIVE on a scoring shot, not alternatives:
        a torpedo through an opening from "farther" earns the pass-through and
        the band. A shot that misses earns neither, however far away it was
        fired from -- the band rewards a hard shot, not a distant one.
        """
        band, ev = result.get('distance_band'), f"{result['fired_range_m']:.2f} m"
        if result['kind'] == 'torpedo':
            if result['outcome'] != 'through':
                return
            self._card.award('torp_any', f"{result['opening']} opening, {ev}")
            if band == 'farther':
                self._card.award('torp_farther', ev)
            elif band == 'far':
                self._card.award('torp_far', ev)
            # "Larger opening first, then smaller."
            hits = [r for r in self._shot_results
                    if r['kind'] == 'torpedo' and r['outcome'] == 'through']
            if [h.get('opening') for h in hits] == ['large', 'small']:
                self._card.award('torp_sequence', 'large then small')
        elif result['outcome'] == 'in_bin':
            if self._competition == 'robosub':
                self._card.award('bin_any', ev)
            else:
                self._card.award('drum_red', ev)

    def _control_evidence(self) -> str:
        """Evidence for "maintain control" -- heading held across the transit.

        The handbook draws the line at actively holding a heading versus
        drifting through, so the honest report is the heading the vehicle was
        actually on when it crossed. A number an operator can dispute beats a
        bare tick.
        """
        if self._pose is None:
            return 'no pose'
        return f'heading {math.degrees(self._pose[5]):.0f} deg at the gate'

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
            elif p.name == 'run' and str(p.value).strip():
                self._run_control(str(p.value).strip().lower())
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
            'run': {
                'competition': self._competition,
                'running': self._run_t0 is not None and self._run_end is None,
                'elapsed_s': round(self._elapsed(), 1),
                'limit_s': self._card.rules['run_seconds'],
                'contact_s': round(self._touch_s, 1),
                'touches': self._touch_n,
                'aborted': self._abort_reason(),
            },
            'card': self._card.snapshot(),
        }

    def _adopt_course_geometry(self) -> None:
        """Take the gate, board, bin and pool geometry FROM THE COURSE.

        These were defaults matching `robosub26_full`, and the launch passed
        only the world name -- so on the other twelve courses the scorer was
        watching for a gate transit at x = -5 when the gate was somewhere else
        entirely, and reported "not transited" for a run that went through it.
        Nothing logged, because nothing was wrong: it was looking exactly where
        it had been told to.

        An explicitly-set parameter still wins, so a course with an unusual
        layout can be corrected without editing it.
        """
        course = _course_yaml(self._world)
        spec = _spec_yaml(self._competition)
        if not course:
            return
        pool = spec.get('pool', {})
        overrides = {}
        if pool:
            overrides['pool_length'] = float(pool.get('length', 25.0))
            overrides['pool_width'] = float(pool.get('width', 16.0))
            overrides['pool_depth'] = float(pool.get('depth', 1.6))
            overrides['pool_edge_depth'] = float(pool.get('floor_edge_depth', 0.0))

        for entry in course.get('props') or []:
            model = str(entry.get('model', ''))
            xy = entry.get('xy') or [0.0, 0.0]
            if 'gate' in model:
                overrides['gate_x'], overrides['gate_y'] = float(xy[0]), float(xy[1])
            elif 'torpedo' in model:
                overrides['board_x'], overrides['board_y'] = float(xy[0]), float(xy[1])
            elif 'bins' in model or 'drum' in model:
                overrides.setdefault('bin_x', float(xy[0]))
                overrides.setdefault('bin_y', float(xy[1]))

        board = (spec.get('props') or {}).get('torpedo_board') or {}
        if board:
            overrides['standoff_far'] = float(board.get('standoff_far', 1.0))
            overrides['standoff_farther'] = float(board.get('standoff_farther', 1.5))

        from rclpy.parameter import Parameter
        if overrides:
            self.set_parameters([Parameter(k, value=v)
                                 for k, v in overrides.items()])
            self.get_logger().info(
                f'[SCORE] geometry from {self._world}: '
                f'gate x={overrides.get("gate_x", "?")} '
                f'board x={overrides.get("board_x", "?")} '
                f'pool {overrides.get("pool_length", "?")}'
                f'x{overrides.get("pool_width", "?")}'
                f'x{overrides.get("pool_depth", "?")} m')

    def _on_state(self, msg) -> None:
        armed = bool(getattr(msg, 'armed', False))
        if armed and self._run_t0 is None:
            self._run_control('start')
        elif not armed and self._run_t0 and not self._run_end:
            self._run_control('stop')

    # -- run clock ---------------------------------------------------------

    def _elapsed(self) -> float:
        if self._run_t0 is None:
            return 0.0
        return (self._run_end or time.time()) - self._run_t0

    def _abort_reason(self):
        """SAUVC aborts a run on sustained or repeated pool contact."""
        limit_s = self._card.rules.get('abort_contact_s')
        limit_n = self._card.rules.get('abort_touches')
        if limit_s and self._touch_s > limit_s:
            return f'{self._touch_s:.1f} s of pool contact (limit {limit_s})'
        if limit_n and self._touch_n > limit_n:
            return f'{self._touch_n} touches (limit {limit_n})'
        return None

    def _run_control(self, action: str) -> None:
        if action == 'start':
            self._run_t0, self._run_end = time.time(), None
            self.get_logger().info('[SCORE] run clock started')
        elif action == 'stop' and self._run_t0 and not self._run_end:
            self._run_end = time.time()
            self._score_time_bonus()
            self._write_card()
            self.get_logger().info(
                f'[SCORE] run ended at {self._elapsed():.1f} s -- '
                f'{self._card.total:.1f} points')
        elif action == 'reset':
            self._card = Scorecard(self._competition)
            self._run_t0 = self._run_end = None
            self._touch_s = 0.0
            self._touch_n = 0
            self.get_logger().info('[SCORE] scorecard reset')

    def _score_time_bonus(self) -> None:
        """The two competitions compute this differently; both need the clock."""
        left = max(0.0, self._card.rules['run_seconds'] - self._elapsed())
        if self._competition == 'robosub':
            # "whole minutes remaining plus fractional seconds", x100.
            self._card.award('time', f'{left:.0f} s remaining')
            self._card.set_bonus('time', round(left / 60.0 * 100.0, 2))
        else:
            # SAUVC: (900 - run) x 0.03, and only with two tasks complete.
            self._card.award('timing', f'{self._elapsed():.0f} s run')
            self._card.set_bonus('timing', round(left * 0.03, 2))

    # -- penalties ---------------------------------------------------------

    def _check_contact(self) -> None:
        """Pool bottom / wall contact, from ground truth rather than a sensor.

        There is no contact sensor in the world template, and adding one would
        report every physics step rather than every touch. Proximity to the
        pool shell is cheaper, survives a restart, and -- the part that matters
        -- is EDGE TRIGGERED: a penalty lands once when the hull arrives at the
        wall, not sixty times a second while it sits there. That distinction
        is the whole difference between a -5 and a -300.
        """
        now = time.time()
        dt = 0.0 if self._last_tick is None else now - self._last_tick
        self._last_tick = now
        if self._pose is None or self._run_t0 is None or self._run_end:
            return
        x, y, z = self._pose[0], self._pose[1], self._pose[2]
        m = float(self.get_parameter('contact_margin').value)
        half_l = float(self.get_parameter('pool_length').value) / 2.0
        half_w = float(self.get_parameter('pool_width').value) / 2.0
        floor = -self._floor_depth(x)
        touching = (abs(x) > half_l - m or abs(y) > half_w - m
                    or z < floor + m)
        if touching:
            self._touch_s += dt
            if not self._touching:
                self._touch_n += 1
                where = ('the floor' if z < floor + m else 'a wall')
                self._card.penalise('touch_pool', f'{where} at {self._elapsed():.0f} s')
                self.get_logger().warning(
                    f'[SCORE] PENALTY: touched {where} '
                    f'(touch {self._touch_n}, {self._touch_s:.1f} s total)')
        self._touching = touching

    def _floor_depth(self, x: float) -> float:
        """Pool depth at x. SAUVC's floor slopes; RoboSub's does not."""
        deep = float(self.get_parameter('pool_depth').value)
        edge = float(self.get_parameter('pool_edge_depth').value)
        if edge <= 0.0:
            return deep
        half = float(self.get_parameter('pool_length').value) / 2.0
        return deep - (deep - edge) * min(1.0, abs(x) / half)

    def _write_card(self) -> None:
        """Persist beside the autonomy mission scorecards.

        Everything above lived in memory, so a sim restart -- which is also how
        you change course -- erased the run you had just done. DUBURI_RUN_DIR
        is the tree the mission scorecards already use, so a practice run's
        two halves land under one timestamp.
        """
        try:
            base = os.environ.get('DUBURI_RUN_DIR') or os.path.expanduser(
                '~/duburi_runs')
            os.makedirs(base, exist_ok=True)
            stamp = time.strftime('%Y%m%dT%H%M%S')
            self._card_path = os.path.join(
                base, f'score_{self._world}_{stamp}.json')
            with open(self._card_path, 'w') as fh:
                json.dump(self.snapshot(), fh, indent=2)
            self.get_logger().info(f'[SCORE] scorecard -> {self._card_path}')
        except Exception as exc:                       # noqa: BLE001
            self.get_logger().warning(f'[SCORE] could not write scorecard: {exc}')

    def _sync_awards(self) -> None:
        """Mirror the continuously-measured elements onto the card.

        Style and the coin flip are not events, they are running totals, so
        they are re-synced each tick rather than awarded once. `award` is
        capped and idempotent, which is what makes calling it from a timer
        safe -- the count only ever walks up to the rulebook's own limit.
        """
        if self._competition != 'robosub':
            return
        for _ in range(self._yaw.changes
                       - self._card._earned.get('style_yaw', {}).get('count', 0)):
            self._card.award('style_yaw', f'{self._yaw.changes} x 90 deg')
        rp = self._roll.changes + self._pitch.changes
        for _ in range(rp - self._card._earned.get(
                'style_rp', {}).get('count', 0)):
            self._card.award('style_rp', f'{rp} x 90 deg')
        if self._coin:
            self._card.award('coin_flip', f'{self._coin} -- vehicle replaced')

    def _publish(self) -> None:
        self._sync_awards()
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
