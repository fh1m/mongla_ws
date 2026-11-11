"""The order a vision process opens the Hailo-8 in (hailo_device._open).

The owner's order, 2026-11-11: through the HailoRT service; if it is down,
wait for systemd to bring it back; then the forward process opens the chip
itself and the downward one runs without it.
"""
import pytest

from mongla_vision.detection import hailo_device as hd


class Maker:
    def __init__(self, service_fails=False):
        self.calls = []
        self.service_fails = service_fails

    def __call__(self, *, service):
        self.calls.append(service)
        if service and self.service_fails:
            raise RuntimeError('HAILO_RPC_FAILED')
        return 'svc' if service else 'direct'


def _open(**kw):
    kw.setdefault('sleep', lambda s: None)
    return hd._open(**kw)


def test_the_service_is_used_when_it_runs():
    m = Maker()
    assert _open(installed=True, running=lambda: True, make=m,
                 fallback='none') == 'svc'
    assert m.calls == [True] and hd.opened_via == 'service'


def test_a_restarting_service_is_waited_for():
    ticks = iter([False, False, False, True, True])
    m = Maker()
    slept = []
    assert hd._open(installed=True, running=lambda: next(ticks), make=m,
                    sleep=slept.append, wait_s=3.0, fallback='none') == 'svc'
    assert len(slept) == 3


def test_forward_opens_the_chip_when_the_service_stays_down():
    m = Maker()
    slept = []
    assert hd._open(installed=True, running=lambda: False, make=m,
                    sleep=slept.append, wait_s=1.0, fallback='direct') == 'direct'
    assert m.calls == [False] and hd.opened_via == 'direct'
    assert 9 <= len(slept) <= 11, 'waited about the bound, not for ever'


def test_downward_refuses_rather_than_take_the_chip():
    with pytest.raises(RuntimeError, match='MONGLA_HAILO_FALLBACK=none'):
        _open(installed=True, running=lambda: False, make=Maker(),
              wait_s=0.5, fallback='none')


def test_a_stale_pid_file_falls_through_to_the_fallback():
    m = Maker(service_fails=True)
    assert _open(installed=True, running=lambda: True, make=m,
                 fallback='direct') == 'direct'
    assert m.calls == [True, False]
    with pytest.raises(RuntimeError, match='HAILO_RPC_FAILED'):
        _open(installed=True, running=lambda: True, make=Maker(True),
              fallback='none')


def test_no_service_installed_opens_directly_without_waiting():
    m = Maker()
    slept = []
    assert hd._open(installed=False, running=lambda: False, make=m,
                    sleep=slept.append, fallback='direct') == 'direct'
    assert slept == [] and m.calls == [False]


def test_a_misspelt_fallback_is_refused(monkeypatch):
    monkeypatch.setenv(hd.FALLBACK_ENV, 'directly')
    with pytest.raises(ValueError, match='expected direct or none'):
        _open(installed=False, make=Maker())


def test_the_unit_restarts_without_limit_and_runs_the_installed_binary():
    from pathlib import Path
    unit = (Path(__file__).resolve().parents[3] / 'kutum' / 'system'
            / 'systemd' / 'hailort.service').read_text()
    assert f'ExecStart={hd.SERVICE_BIN}' in unit
    assert f'PIDFile={hd.SERVICE_PID}' in unit
    assert 'Restart=always' in unit and 'StartLimitIntervalSec=0' in unit


def test_the_service_address_is_one_value_in_three_places():
    """The unit listens where it is told, the shells and the vision package
    connect where they are told. Three copies of one path: held equal."""
    import pathlib
    import re
    from mongla_vision.detection.hailo_device import SERVICE_ADDR
    kutum = pathlib.Path(__file__).resolve().parents[3] / 'kutum'
    unit = (kutum / 'system/systemd/hailort.service').read_text()
    env = (kutum / 'env.sh').read_text()
    assert re.search(rf'^Environment=HAILORT_SERVICE_ADDRESS={re.escape(SERVICE_ADDR)}$', unit, re.M)
    assert re.search(rf'^export HAILORT_SERVICE_ADDRESS={re.escape(SERVICE_ADDR)}$', env, re.M)
    # Not the default: the unit's private /tmp would hide it from every client.
    assert '/tmp/' not in SERVICE_ADDR


def test_importing_detection_sets_the_address_before_the_library_can_load():
    import os
    import subprocess
    import sys
    code = ('import os; os.environ.pop("HAILORT_SERVICE_ADDRESS", None)\n'
            'import mongla_vision.detection\n'
            'print(os.environ["HAILORT_SERVICE_ADDRESS"])')
    env = {k: v for k, v in os.environ.items() if k != 'HAILORT_SERVICE_ADDRESS'}
    env['PYTHONPATH'] = os.pathsep.join(p for p in sys.path if p)   # this test's own imports
    r = subprocess.run([sys.executable, '-c', code], capture_output=True, text=True, env=env)
    assert r.returncode == 0, r.stderr
    assert r.stdout.strip() == 'unix:///run/hailo/hailort_uds.sock'
