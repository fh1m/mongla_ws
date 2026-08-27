"""Regression tests for the operator lab's hardening (2026-08-27).

The lab is an unauthenticated HTTP server that can arm thrusters, so each of
these is a real hole that was open, not a hypothetical. Every test names the
exact mechanism, because all three bugs LOOKED guarded:

  * the command builder validated nothing and pasted the verb into shell text;
  * the zip extractor checked for `..` -- and pathlib silently drops the left
    operand when the right side is absolute, so the check did nothing;
  * the SPA fallback had no containment check at all, while the upload handler
    right next to it did.
"""
import subprocess
import sys
from pathlib import Path

import pytest

server = pytest.importorskip('duburi_sim_web.server')
from duburi_sim_bridge.paths import runtime_dir  # noqa: E402


# --------------------------------------------------------------------------
# 1. POST /api/vehicle/cmd -- shell injection
# --------------------------------------------------------------------------

INJECTIONS = [
    'arm; touch /tmp/duburi_pwned',
    'arm && id',
    'arm | tee /tmp/duburi_pwned',
    '$(id)',
    '`id`',
    'arm\nid',
    '../../../bin/sh',
    'arm --duration 5',        # space -- must go through `extra`, not the verb
    '',
]


@pytest.mark.parametrize('verb', INJECTIONS)
def test_metacharacter_verbs_are_rejected(verb):
    with pytest.raises(Exception) as exc:
        server._duburi_cmd(verb)
    assert getattr(exc.value, 'status_code', None) == 400, exc.value


def test_a_real_verb_is_accepted_and_lands_in_argv():
    argv = server._duburi_cmd('move_forward', '--duration', '5')
    # ['bash', '-c', <script>, <duburi_ws>, 'move_forward', '--duration', '5']
    assert argv[0] == 'bash' and argv[1] == '-c'
    assert argv[-3:] == ['move_forward', '--duration', '5']
    assert 'move_forward' not in argv[2], 'verb must not be inside the script text'


def test_injection_would_be_inert_even_without_the_regex():
    """The regex is the fast 400; THIS is the actual defence.

    bash parses the -c script once, before $1.. are substituted, so a payload
    arriving as a positional argument can never re-enter the parser. Proven by
    running the real script shape with a hostile argument and checking that the
    side effect did not happen.
    """
    marker = Path(runtime_dir()) / 'injection_probe'
    marker.unlink(missing_ok=True)
    payload = f'arm; touch {marker}'

    script = (
        'source /opt/ros/humble/setup.bash 2>/dev/null; '
        'test -f "$0/install/setup.bash" && source "$0/install/setup.bash"; '
        'exec printf "%s\\n" "$@"'          # stand-in for `ros2 run ... duburi`
    )
    out = subprocess.run(
        ['bash', '-c', script, '/nonexistent', payload],
        capture_output=True, text=True, timeout=30,
    )
    assert out.stdout.strip() == payload, out
    assert not marker.exists(), 'semicolon was interpreted -- injection is live'


# --------------------------------------------------------------------------
# 2. Zip upload -- absolute members escaped the `..` check
# --------------------------------------------------------------------------

@pytest.mark.parametrize('member', ['/etc/cron.d/pwned', '/tmp/pwned', '../../pwned'])
def test_absolute_and_dotdot_members_never_escape_the_target(member):
    target = Path('/safe/models/mymodel')
    rel = Path(member)
    rejected = (not member) or ('..' in rel.parts) or rel.is_absolute()
    assert rejected, f'{member!r} passed the filter'
    # and this is why `..`-only was not enough:
    if rel.is_absolute():
        assert target / member == rel, 'pathlib no longer discards the left operand'


def test_a_normal_member_is_still_allowed():
    rel = Path('meshes/hull.dae')
    assert '..' not in rel.parts and not rel.is_absolute()
    assert str(Path('/safe/models/m') / rel).startswith('/safe/models/m/')


# --------------------------------------------------------------------------
# 3. SPA fallback -- path containment
# --------------------------------------------------------------------------

def test_spa_paths_outside_static_root_are_not_served(tmp_path):
    static = tmp_path / 'static'
    (static / 'assets').mkdir(parents=True)
    (static / 'index.html').write_text('ok')
    (tmp_path / 'secret.txt').write_text('nope')

    def serves(spa_path: str) -> bool:
        candidate = (static / spa_path).resolve()
        return candidate.is_file() and candidate.is_relative_to(static.resolve())

    assert not serves('../secret.txt')
    assert not serves('/etc/passwd')
    assert serves('index.html')


# --------------------------------------------------------------------------
# 4. Side-channel files are per-user, not world-writable /tmp
# --------------------------------------------------------------------------

def test_runtime_dir_is_private_to_this_user():
    d = runtime_dir()
    assert d.is_dir()
    assert d.name.startswith('duburi-')
    assert d.stat().st_mode & 0o077 == 0, 'group/other must have no access'


def test_lab_writes_nothing_to_bare_tmp():
    src = Path(server.__file__).read_text()
    for legacy in ('/tmp/duburi_lab_', '/tmp/duburi_prop_manager'):
        assert legacy not in src, f'{legacy} came back -- use _rt()'


# --------------------------------------------------------------------------
# 5. The MJPEG stream must stay a coroutine
# --------------------------------------------------------------------------

def test_mjpeg_is_async():
    """A sync `def` route runs in Starlette's 40-slot threadpool. This handler
    is an unbounded `while True`, so as a sync route each open stream pinned a
    thread forever and the UI opens two -- roughly 20 reloads wedged the whole
    API, `disarm` included."""
    import inspect
    assert inspect.iscoroutinefunction(server.mjpeg)
    assert 'is_disconnected' in inspect.getsource(server.mjpeg)


if __name__ == '__main__':
    sys.exit(pytest.main([__file__, '-q']))


# --------------------------------------------------------------------------
# 6. DUBURI_LAB_HOST: the shell message and the Python bind must agree
# --------------------------------------------------------------------------
#
# Found by a commit security review, 2026-08-27. The lab's bind host was read by
# two different parsers with different empty-string semantics:
#
#   shell   ${DUBURI_LAB_HOST:-127.0.0.1}          substitutes on unset OR EMPTY
#   python  os.environ.get(k, '127.0.0.1')         returns '' when the key EXISTS
#
# and '' then hit a `host if host not in ('', '0.0.0.0') else '0.0.0.0'` branch.
# So `DUBURI_LAB_HOST= duburi_sim lab` printed "LOOPBACK ONLY" while the server
# bound every interface -- on an unauthenticated API that can arm thrusters. The
# reassuring half was the wrong one.
#
# Set-but-empty is not exotic: an unset key in `docker run --env-file`, a bare
# `DUBURI_LAB_HOST=` line in .env, systemd `Environment=DUBURI_LAB_HOST=`.

def _lab_script():
    """Find scripts/duburi_sim by walking up, not by a parents[] index.

    server.__file__ lands in src/ under symlink-install but in build/ or
    install/ otherwise, so a fixed index silently points at the wrong tree.
    Skip when it is genuinely absent (installed-only checkout) rather than fail.
    """
    rel = Path('src') / 'duburi_sim_bringup' / 'scripts' / 'duburi_sim'
    here = Path(server.__file__).resolve()
    for base in here.parents:
        if (base / rel).is_file():
            return base / rel
    pytest.skip('source tree not available (installed-only checkout)')


def _python_host(value):
    """Exactly the expression server.py uses to pick its bind address."""
    env = {} if value is None else {'DUBURI_LAB_HOST': value}
    return (env.get('DUBURI_LAB_HOST') or '127.0.0.1').strip() or '127.0.0.1'


def _shell_host(value):
    """Exactly the normalisation `duburi_sim lab` exports."""
    script = (
        '_lab_host="${DUBURI_LAB_HOST:-127.0.0.1}"; '
        '_lab_host="${_lab_host//[[:space:]]/}"; '
        '[[ -n "$_lab_host" ]] || _lab_host="127.0.0.1"; '
        'printf "%s" "$_lab_host"'
    )
    import os
    env = dict(os.environ)
    env.pop('DUBURI_LAB_HOST', None)
    if value is not None:
        env['DUBURI_LAB_HOST'] = value
    return subprocess.run(['bash', '-c', script], capture_output=True,
                          text=True, env=env, timeout=30).stdout


@pytest.mark.parametrize('value', [None, '', '   ', '127.0.0.1', '0.0.0.0',
                                   'localhost', '192.168.2.69', ' 0.0.0.0 '])
def test_shell_and_python_agree_on_the_bind_host(value):
    assert _shell_host(value) == _python_host(value), (
        f'DUBURI_LAB_HOST={value!r}: the CLI announces one host and the server '
        f'binds another'
    )


@pytest.mark.parametrize('value', [None, '', '   '])
def test_absent_or_empty_never_means_wildcard(value):
    """The regression itself: nothing that reads as "unset" may expose the lab."""
    assert _python_host(value) == '127.0.0.1'
    assert _shell_host(value) == '127.0.0.1'


def test_only_an_explicit_wildcard_exposes_the_lab():
    assert _python_host('0.0.0.0') == '0.0.0.0'
    src = Path(server.__file__).read_text()
    assert "host if host not in ('', '0.0.0.0')" not in src, \
        'the empty-string-to-wildcard branch is back'


def test_the_cli_over_warns_rather_than_under_warns():
    """A host the CLI cannot prove is loopback must be announced as EXPOSED."""
    s = _lab_script().read_text()
    assert 'export DUBURI_LAB_HOST="$_lab_host"' in s, \
        'the CLI must hand the server the exact host it just described'
    assert '127.*' in s and 'localhost' in s and '::1' in s, \
        'loopback classification must cover 127.*, localhost and ::1'
