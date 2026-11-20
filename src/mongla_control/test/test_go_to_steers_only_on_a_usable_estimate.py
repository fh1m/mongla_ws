"""`go_to` drives on the filter only while the filter can be trusted.

Each refusal is checked by what reaches the board: a refused or lost run sends
no thrust frame after the estimate went bad.
"""
import math
from types import SimpleNamespace as NS

import pytest

from mongla_control import localization_verbs as lv


def _state(x=0.0, y=0.0, yaw=0.0, sigma=0.1, usable=True, reason='',
           motion='ok'):
    return NS(x_m=x, y_m=y, depth_m=-1.0, yaw_deg=yaw, sigma_xy_m=sigma,
              sigma_yaw_deg=1.0, usable=usable, reason=reason, motion=motion,
              aiding='aided', frame='pool', flow_fused=10, flow_refused=0,
              fixes=1)


class _FC:
    def __init__(self):
        self.frames = []

    def manual(self, **kw):
        self.frames.append(kw)


class _Hull:
    """Moves the state toward each frame's demand, so the loop can arrive."""

    def __init__(self, s, step_m=0.05):
        self.s, self.fc, self.step = s, _FC(), step_m

    def provider(self):
        if self.fc.frames:
            f = self.fc.frames[-1]
            psi = math.radians(self.s.yaw_deg)
            # body -> pool
            self.s.x_m += self.step * (math.cos(psi) * f['fwd'] - math.sin(psi) * f['lat']) / 0.3
            self.s.y_m += self.step * (math.sin(psi) * f['fwd'] + math.cos(psi) * f['lat']) / 0.3
        return self.s


class _Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def sleep(self, dt):
        self.t += dt


def _run(provider, fc, **kw):
    c = _Clock()
    args = dict(x_m=2.0, y_m=1.0, tolerance_m=0.3, duration=60.0, gain=30.0,
                abort_fn=lambda: False, log=None)
    args.update(kw)
    return lv.go_to_loop(fc=fc, provider=provider, clock=c, sleep=c.sleep,
                         mixer_aware=False, **args)


@pytest.mark.parametrize('yaw', [0.0, 90.0, -135.0, 180.0])
def test_it_arrives_from_any_heading(yaw):
    h = _Hull(_state(yaw=yaw))
    out = _run(h.provider, h.fc)
    assert out.code == lv.ARRIVED, out
    assert out.remaining_m <= 0.3
    assert all(f['yaw'] == 0.0 and f['up'] == 0.0 for f in h.fc.frames)


def test_the_body_rotation_is_ned():
    # Facing east (yaw 90), a goal to the north is to port: negative starboard.
    fwd, stb = lv.body_demand(1.0, 0.0, 90.0, 30.0)
    assert fwd == pytest.approx(0.0, abs=1e-9) and stb == pytest.approx(-30.0)
    fwd, stb = lv.body_demand(0.0, 2.0, 90.0, 30.0)
    assert fwd == pytest.approx(30.0) and stb == pytest.approx(0.0, abs=1e-9)


def test_the_cap_keeps_the_direction():
    fwd, stb = lv.body_demand(10.0, 10.0, 0.0, 30.0)
    assert math.hypot(fwd, stb) == pytest.approx(30.0)
    assert fwd == pytest.approx(stb)


@pytest.mark.parametrize('s,why', [
    (None, 'no localization state'),
    (_state(usable=False, reason='heading not anchored'), 'anchored'),
    (_state(sigma=0.6), 'worse than'),
    (_state(sigma=math.nan), 'worse than'),
    (_state(motion='blocked'), 'blocked'),
])
def test_it_refuses_before_any_thrust(s, why):
    fc = _FC()
    out = _run(lambda: s, fc)
    assert out.code in (lv.LOST, lv.BLOCKED) and why in out.reason
    assert fc.frames == []


def test_an_estimate_that_goes_bad_mid_run_stops_the_thrust():
    good = _state()
    seq = [good] * 5 + [None] * 3

    def prov():
        return seq.pop(0) if seq else None
    fc = _FC()
    out = _run(prov, fc)
    assert out.code == lv.LOST
    assert len(fc.frames) == 5


def test_abort_and_timeout():
    fc = _FC()
    assert _run(lambda: _state(), fc, abort_fn=lambda: True).code == lv.ABORTED
    assert fc.frames == []
    out = _run(lambda: _state(), _FC(), duration=1.0)    # a hull that never moves
    assert out.code == lv.TIMEOUT and out.remaining_m == pytest.approx(math.hypot(2, 1))


def test_the_registry_and_the_facade_agree():
    import inspect
    from mongla_control import COMMANDS
    from mongla_control.mongla import Mongla
    for verb in ('locate', 'go_to'):
        params = list(inspect.signature(getattr(Mongla, verb)).parameters)[1:]
        assert params == COMMANDS[verb]['fields'], verb
