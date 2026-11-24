"""Kutum's files agree with each other and with the repo.

Kutum is five shell scripts and the files they install (kutum/README.md). The
scripts run on a machine the test suite cannot reach, so what can be checked
here is that they are internally one truth: what setup installs is what the
doctor compares, the vehicle's pins satisfy the ranges the code is tested
against, and every script at least parses.
"""
import re
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
KUTUM = ROOT / 'kutum'
SCRIPTS = sorted(KUTUM.rglob('*.sh')) + [KUTUM / 'system/systemd/set-cpu-performance',
                                         KUTUM / 'system/networkd-dispatcher/50-kutum-source-routes']


def test_every_script_parses():
    bad = []
    for sh in SCRIPTS:
        r = subprocess.run(['bash', '-n', str(sh)], capture_output=True, text=True)
        if r.returncode:
            bad.append(f'{sh.relative_to(ROOT)}: {r.stderr.strip()}')
    assert not bad, '\n'.join(bad)


@pytest.mark.skipif(not shutil.which('shellcheck'), reason='shellcheck not installed')
def test_every_script_is_shellcheck_clean():
    r = subprocess.run(['shellcheck', '-S', 'warning', *map(str, SCRIPTS)],
                       capture_output=True, text=True)
    assert r.returncode == 0, r.stdout


def _system_files(script, pattern):
    return set(re.findall(pattern, (KUTUM / script).read_text()))


def test_the_doctor_compares_exactly_what_setup_installs():
    """A file setup installs and the doctor never reads can drift on the
    vehicle unseen; one the doctor reads and setup never installs is a
    WARNING no command can clear."""
    installed = _system_files('setup.sh', r'install -m \d+ "\$KUTUM/(system/[^"]+)"')
    compared = _system_files('doctor.sh', r'same "\$KUTUM/(system/[^"]+)"')
    assert installed, 'parsed no install lines from setup.sh -- the pattern broke'
    assert installed == compared, (installed ^ compared)
    for rel in installed:
        assert (KUTUM / rel).is_file(), f'kutum/{rel} is installed but not in the repo'


def _pins(path):
    """{name: (specifier, the line's comment)}"""
    out = {}
    for line in path.read_text().splitlines():
        line, _, note = line.partition('#')
        m = re.match(r'([A-Za-z0-9_.-]+)\s*(.*)$', line.strip())
        if m:
            out[m.group(1).lower()] = (m.group(2).strip(), note)
    return out


def test_the_vehicle_pins_satisfy_the_repo_ranges():
    """The suite runs against requirements.txt's ranges; the vehicle runs
    kutum/requirements.txt's exact pins. A pin outside the range is a vehicle
    running code nothing tested."""
    from packaging.specifiers import SpecifierSet
    ranges, pins = _pins(ROOT / 'requirements.txt'), _pins(KUTUM / 'requirements.txt')
    assert pins, 'kutum/requirements.txt pins nothing'
    bad = []
    for name, (spec, note) in pins.items():
        assert spec.startswith('=='), f'{name}: the vehicle pins exactly, got {spec!r}'
        parent = re.search(r'dependency of (\S+?);', note)
        if name not in ranges:
            if not (parent and parent.group(1) in ranges):
                bad.append(f'{name} is on the vehicle but not in requirements.txt, '
                           'and names no package there it is a dependency of')
        elif spec[2:] not in SpecifierSet(ranges[name][0]):
            bad.append(f'{name}{spec} is outside requirements.txt {ranges[name][0]!r}')
    assert not bad, '\n'.join(bad)


def test_numpy_stays_below_2_on_the_vehicle():
    assert 'numpy<2' in (KUTUM / 'constraints.txt').read_text()
    assert '-c "$KUTUM/constraints.txt"' in (KUTUM / 'setup.sh').read_text()


def test_one_hailort_version():
    """The driver, firmware, runtime and bindings must match exactly, so the
    version is written once."""
    setup = (KUTUM / 'setup.sh').read_text()
    found = re.findall(r'^HAILORT_V=(\d+\.\d+\.\d+)$', setup, re.M)
    assert len(found) == 1, found
    assert setup.count(found[0]) == 1, f'{found[0]} is written outside HAILORT_V'


def test_deploy_keeps_agent_tooling_off_the_vehicle_and_the_book_on_it():
    deploy = (KUTUM / 'deploy.sh').read_text()
    for d in ('.claude/skills/', '.claude/agents/', '.claude/hooks/', '.github/', '.beads/'):
        assert f"--exclude '{d}'" in deploy, d
    assert "--exclude '.claude/'" not in deploy   # the book is read at the pool


def test_the_doctor_runs_after_the_sync():
    """A FAIL that only `setup.sh --system` fixes needs kutum/ on the machine
    first; gating before the sync made a fresh machine undeployable."""
    deploy = (KUTUM / 'deploy.sh').read_text()
    assert deploy.index('rsync -az') < deploy.index('kutum/doctor.sh" ||')


def _verdict(doctor_rc: int, reboot: str):
    """Run setup.sh's own `verdict` function, lifted out of the file."""
    fn = re.search(r'^verdict\(\) \{\n.*?^\}\n', (KUTUM / 'setup.sh').read_text(),
                   flags=re.S | re.M)
    assert fn, 'setup.sh no longer defines verdict()'
    return subprocess.run(['bash', '-c', fn.group(0) + f'verdict {doctor_rc} "{reboot}"'],
                          capture_output=True, text=True)


@pytest.mark.parametrize('rc,reboot,want', [
    (0, '', 0),
    (0, ' rtc-charging', 0),
    (1, '', 1),
    (1, ' netplan', 3),
])
def test_setup_returns_the_doctors_verdict(rc, reboot, want):
    """Issue #69: `--system` swallowed a doctor FAIL and exited 0, so a machine
    that could not run the stack read as provisioned. A FAIL is never 0; one
    that a pending reboot may explain gets its own code and an instruction."""
    r = _verdict(rc, reboot)
    assert r.returncode == want, r.stdout + r.stderr
    if want == 3:
        assert 'Reboot, then run kutum/doctor.sh' in r.stderr


def test_setup_hands_the_doctor_result_to_verdict():
    """The function is only half of it: the call site must not discard the code."""
    s = (KUTUM / 'setup.sh').read_text()
    assert 'doctor.sh" || true' not in s
    assert 'doctor.sh" || rc=$?' in s and 'verdict "$rc" "$REBOOT"' in s


# Root units that listen on nothing: each one a oneshot that sets something and
# exits. Anything else Kutum installs is a service, and runs confined (#67).
# Root because each edits what only root may: the CPU governors, the routing
# policy (reply routes, kutum-source-routes); both exit at once.
_ROOT_ONESHOTS = {'cpu-performance.service', 'kutum-source-routes.service'}
_CONFINED = ('NoNewPrivileges=yes', 'ProtectSystem=strict', 'PrivateTmp=yes')


def _unconfined(unit: Path) -> list:
    text = unit.read_text()
    if unit.name in _ROOT_ONESHOTS:
        return [] if 'Type=oneshot' in text else ['not a oneshot']
    user = re.search(r'^User=(\S+)', text, flags=re.M)
    # Its own user: not root, and not the operator, whose sudo needs no password.
    bad = [] if user and user.group(1) not in ('root', 'fh1m') else ['no own User=']
    return bad + [k for k in _CONFINED if k not in text]


def test_every_kutum_service_is_confined(tmp_path):
    """The operator keeps passwordless sudo (#67, owner's call), so a service
    that ran as the operator or as root would hand that to whoever compromised
    it. Every unit Kutum installs is a known root oneshot, or confined."""
    units = sorted((KUTUM / 'system/systemd').glob('*.service'))
    assert units
    bad = {u.name: _unconfined(u) for u in units if _unconfined(u)}
    assert not bad, bad
    # Injection: a deck service as the operator, unhardened, is refused.
    rogue = tmp_path / 'deck.service'
    rogue.write_text('[Service]\nUser=deck\nExecStart=/bin/true\n')
    assert _unconfined(rogue) == [*_CONFINED]
    for who in ('', 'User=root\n', 'User=fh1m\n'):
        rogue.write_text(f'[Service]\n{who}ExecStart=/bin/true\n')
        assert 'no own User=' in _unconfined(rogue), who


def test_the_hailort_service_keeps_the_one_capability_it_needs():
    """As its own user, the service can only tell a client is alive by
    signalling it, which needs CAP_KILL; without it every client loses its
    network at the first inference (measured-bars 83). It gets that and
    nothing else."""
    text = (KUTUM / 'system/systemd/hailort.service').read_text()
    lines = {l.split('=', 1)[0]: l.split('=', 1)[1] for l in text.splitlines()
             if '=' in l and not l.startswith('#')}
    assert lines.get('User') == 'hailort'
    assert lines.get('AmbientCapabilities') == 'CAP_KILL'
    assert lines.get('CapabilityBoundingSet') == 'CAP_KILL'


def _pin_grader():
    """The Python the doctor runs to compare kutum/requirements.txt with what is installed."""
    s = (ROOT / 'kutum' / 'doctor.sh').read_text()
    start = s.index("import importlib.metadata as md, re, sys")
    return s[start:s.index('\nPY\n', start)]


def test_the_doctor_fails_a_pin_the_machine_does_not_have(tmp_path):
    """deploy.sh never installs the pins (setup.sh does), so the doctor it runs
    from the staged release is what refuses a release whose pins the vehicle
    lacks. Verified on Kutum 2026-11-20: onnxruntime==1.32.0 against an
    installed 1.31.1 is a FAIL, and the four real pins pass."""
    import importlib.metadata as md
    import sys
    have = md.version('pytest')
    req = tmp_path / 'requirements.txt'
    req.write_text(f'pytest=={have}   # comment\nnot-a-real-pkg-mongla==1.0\n'
                   'pytest[testing]==0.0.1\n# pinned==9.9\n')
    script = tmp_path / 'grade.py'
    script.write_text(_pin_grader())
    out = subprocess.run([sys.executable, str(script), str(req)], capture_output=True,
                         text=True, check=True).stdout.splitlines()
    assert out[0] == f'ok pytest=={have}'
    assert out[1] == 'DIFF not-a-real-pkg-mongla==1.0, installed missing'
    assert out[2] == f'DIFF pytest==0.0.1, installed {have}'
    assert len(out) == 3


def test_the_pin_check_runs_before_the_switch():
    doctor = (ROOT / 'kutum' / 'doctor.sh').read_text()
    assert 'DIFF) fail "pin' in doctor
    deploy = (ROOT / 'kutum' / 'deploy.sh').read_text()
    assert deploy.index('kutum/doctor.sh" ||') < deploy.index('release.sh activate')


def test_the_vehicle_masks_the_timers_it_never_runs_and_the_doctor_checks_them():
    """fwupd took 46 % of a core a minute after its refresh timer fired on the
    bench (bead 0yp). setup masks every name in QUIET_TIMERS; the doctor reads
    the same line, so a timer added to one is checked by the other."""
    setup = (KUTUM / 'setup.sh').read_text()
    found = re.findall(r"^QUIET_TIMERS='([^']*)'$", setup, re.M)
    assert found and 'fwupd-refresh.timer' in found[0].split()
    assert 'systemctl mask --now "$unit"' in setup
    assert 'for unit in $QUIET_TIMERS' in setup
    doctor = (KUTUM / 'doctor.sh').read_text()
    assert "s/^QUIET_TIMERS=" in doctor and '"$KUTUM/setup.sh"' in doctor


def _calibration_gate(tmp_path, vehicle, here):
    """Run deploy.sh's calibration gate with the vehicle simulated on disk."""
    text = (ROOT / 'kutum' / 'deploy.sh').read_text()
    gate = text[text.index('CAL=src/mongla_vision'):]
    gate = gate[:gate.index('\nfi\n') + 4]
    cal = 'src/mongla_vision/config/calibration'
    for root, files in (('vehicle', vehicle), ('repo', here)):
        (tmp_path / root / cal).mkdir(parents=True)
        for name, body in files.items():
            (tmp_path / root / cal / name).write_text(body)
    (tmp_path / 'home').mkdir()
    (tmp_path / 'home' / 'mongla_ws').symlink_to(tmp_path / 'vehicle')
    script = ('set -euo pipefail\nr() { bash -c "$*"; }\n'
              'die() { echo "DIE $*"; exit 3; }\nok() { echo "OK $*"; }\n'
              f'REPO_ROOT={tmp_path / "repo"}\nTARGET=fh1m@kutum.local\n' + gate)
    return subprocess.run(['/bin/bash', '-c', script], capture_output=True, text=True,
                          env={'HOME': str(tmp_path / 'home'), 'PATH': '/usr/bin:/bin'})


def test_deploy_refuses_to_replace_a_calibration_only_the_vehicle_holds(tmp_path):
    # calibrate_node installs into the live release; the new release would
    # carry only the committed file and the camera would fly the old lens.
    r = _calibration_gate(tmp_path, {'pi_downward_1280x720.json': 'solved on the vehicle'},
                          {'pi_downward_1280x720.json': 'committed'})
    assert r.returncode == 3 and 'pi_downward_1280x720.json' in r.stdout, r.stdout + r.stderr


def test_deploy_passes_when_the_vehicle_holds_only_what_is_committed(tmp_path):
    same = {'pi_downward_1280x720.json': 'committed', 'pi_forward_1280x720.json': 'f'}
    r = _calibration_gate(tmp_path, same, {**same, 'new.json': 'only here'})
    assert r.returncode == 0 and 'OK no calibration' in r.stdout, r.stdout + r.stderr
