"""The viewer stream's rate follows the slowest browser's lag (link.py).

foxglove_bridge queues one client's messages in arrival order, so when the
pictures outrun the link the vehicle's state waits behind them: three
captures on 2026-11-23 showed it seconds old with the stack healthy. The
governor halves the cameras' `stream` rate (the viewers' copy; detection
reads image_raw) while any browser reports lag, and steps it back up only
after a calm spell, so it cannot oscillate faster than the queue drains.
"""
from mongla_batighor.link import StreamGovernor


def test_lag_halves_the_rate_and_calm_brings_it_back_in_steps():
    g = StreamGovernor()
    assert g.scale == 1.0
    assert g.report(0.05, 0.0) is None              # calm at full rate: nothing to do
    assert g.report(1.2, 1.0) == 0.5                # lagging: halve
    assert g.report(1.2, 2.0) is None               # too soon: the queue needs time to drain
    assert g.report(1.2, 3.5) == 0.25               # still lagging after the hold: halve again
    assert g.report(0.05, 8.0) is None              # calm, but not for long enough
    assert g.report(0.05, 13.6) == 0.5              # 10 s calm since the last lag
    assert g.report(0.05, 23.7) == 0.75             # one step per calm spell
    assert g.report(0.05, 33.8) == 1.0
    assert g.report(0.05, 60.0) is None             # never above the camera's own rate


def test_any_lagging_browser_holds_the_rate_down():
    g = StreamGovernor()
    g.report(1.0, 0.0)                              # a slow browser: 0.5
    for t in range(1, 30):
        g.report(0.02, float(t))                    # a fast one, calm every second
        if t % 5 == 0:
            g.report(0.9, t + 0.5)                  # the slow one still lagging
    assert g.scale <= 0.5


def test_the_floor_keeps_a_picture():
    g = StreamGovernor()
    t = 0.0
    for _ in range(20):
        g.report(5.0, t)
        t += 3.0
    assert g.scale == g.floor and 5.0 / 30.0 <= g.floor * 1.0001
    assert g.rate(30.0) >= 5.0 - 1e-9 and g.rate(10.0) >= 5.0 - 1e-9


def test_a_report_that_is_not_a_number_changes_nothing():
    g = StreamGovernor()
    for bad in (float('nan'), float('inf'), -1.0, None, 'x'):
        assert g.report(bad, 100.0) is None
    assert g.scale == 1.0


def test_the_screen_that_cut_the_video_is_named_until_it_is_back():
    """One stream serves every screen, so the slowest sets it for all; the
    operator must be able to see which screen that is and close it."""
    g = StreamGovernor()
    assert g.by == ''
    g.report(0.02, 0.0, 'a1b2')
    assert g.by == ''                               # a calm report names nobody
    g.report(1.2, 1.0, 'c3d4')
    assert g.by == 'c3d4'
    g.report(1.2, 3.5, 'e5f6')                      # the next cut names its own screen
    assert g.by == 'e5f6'
    for t in (14.0, 24.1):
        g.report(0.02, t, 'a1b2')
    assert g.scale == 0.75 and g.by == 'e5f6'       # still cut: still that screen's doing
    g.report(0.02, 34.2, 'a1b2')
    assert g.scale == 1.0 and g.by == ''
