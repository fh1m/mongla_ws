"""The guarantees, not the tree library.

Retry the flaky thing, never let cleanup kill a run, and always leave a scoring
attempt on the table. The decisive tests are the ones where something raises,
because that is when a mission either degrades or dies.
"""
import pytest

from mongla_planner.resilience import Attempt, never_fails, retry, selector


class _Flaky:
    def __init__(self, succeed_on, raises=False):
        self.calls = 0
        self.succeed_on = succeed_on
        self.raises = raises

    def __call__(self):
        self.calls += 1
        if self.calls < self.succeed_on:
            if self.raises:
                raise RuntimeError('service not up yet')
            return False
        return True


# --- retry ------------------------------------------------------------------


def test_a_step_that_works_first_time_is_not_retried():
    f = _Flaky(succeed_on=1)
    got = retry(f, times=3)
    assert got.ok and got.tries == 1 and f.calls == 1


def test_a_flaky_step_succeeds_on_a_later_try():
    f = _Flaky(succeed_on=3)
    got = retry(f, times=3)
    assert got.ok and got.tries == 3


def test_a_raise_counts_as_a_failed_try_not_a_crash():
    # A service that is not up yet raises, and that is exactly what retry is for.
    f = _Flaky(succeed_on=2, raises=True)
    got = retry(f, times=3)
    assert got.ok and got.tries == 2


def test_a_genuinely_broken_step_stops_after_the_budget():
    f = _Flaky(succeed_on=99)
    got = retry(f, times=2)
    assert not got.ok and got.tries == 2 and f.calls == 2
    assert 'falsy' in got.error


def test_the_last_error_is_reported_not_swallowed():
    def boom():
        raise ValueError('no detector')
    got = retry(boom, times=1)
    assert not got.ok and 'ValueError: no detector' in got.error


def test_times_below_one_still_runs_once():
    f = _Flaky(succeed_on=1)
    assert retry(f, times=0).ok and f.calls == 1


# --- never_fails ------------------------------------------------------------


def test_a_raising_cleanup_is_contained():
    # A camera restore that raises in a finally masks the real exit.
    def boom():
        raise RuntimeError('detector gone')
    got = never_fails(boom)
    assert not got.ok and 'RuntimeError' in got.error   # recorded, not raised


def test_a_working_step_reports_success():
    assert never_fails(lambda: True).ok


def test_a_falsy_step_is_a_recorded_failure():
    got = never_fails(lambda: False)
    assert not got.ok and 'falsy' in got.error


# --- selector ---------------------------------------------------------------


def test_the_first_working_branch_wins_and_the_rest_do_not_run():
    ran = []
    got = selector([('precise', lambda: ran.append('p') or True),
                    ('blind', lambda: ran.append('b') or True)])
    assert got.ok and got.branch == 'precise' and ran == ['p']


def test_a_perception_miss_falls_through_to_the_scoring_branch():
    # THE POINT. A miss should cost the precision points, not all of them.
    got = selector([('precise', lambda: False),
                    ('blind_fire', lambda: True)])
    assert got.ok and got.branch == 'blind_fire'


def test_a_RAISING_branch_still_reaches_the_fallback():
    # The fallback exists precisely for when things go wrong; a raise that
    # escaped the selector would skip it.
    def boom():
        raise RuntimeError('align blew up')
    got = selector([('precise', boom), ('blind_fire', lambda: True)])
    assert got.ok and got.branch == 'blind_fire'


def test_every_branch_failing_reports_the_last_reason():
    got = selector([('precise', lambda: False), ('blind', lambda: False)])
    assert not got.ok and 'blind' in got.error and got.tries == 2


def test_an_empty_selector_fails_rather_than_claiming_success():
    assert not selector([]).ok


# --- the mission cleanup, which is where this earns its place ---------------


def _full_mission_source():
    import pathlib
    return (pathlib.Path(__file__).resolve().parents[1] / 'mongla_planner'
            / 'missions' / 'task_full_2026.py').read_text()


def test_release_heading_and_stop_are_contained():
    # Running them as one block meant the first failure ate the rest: a
    # release_heading that raised left the vehicle armed.
    src = _full_mission_source()
    assert 'never_fails(mongla.release_heading' in src
    assert 'never_fails(mongla.stop' in src


def test_disarm_is_NOT_contained():
    # The one failure this stack cannot tolerate is a disarm that reports
    # success. Every other cleanup step is tidying; this one is the safety path.
    src = _full_mission_source()
    assert 'never_fails(mongla.disarm' not in src, \
        'disarm must raise, loudly, even from a finally'
    assert 'mongla.disarm()' in src


def test_a_failing_first_cleanup_step_no_longer_blocks_the_others():
    # The behaviour the source change buys, driven directly.
    calls = []

    def release():
        calls.append('release')
        raise RuntimeError('lock node gone')

    never_fails(release, name='release_heading')
    never_fails(lambda: calls.append('stop') or True, name='stop')
    assert calls == ['release', 'stop']
