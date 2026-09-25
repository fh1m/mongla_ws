"""The ladder's switches come from a deck file, not from launch plumbing.

⛔ THE DEFECT THIS AVOIDS. A switch threaded through launch is only read by the
launch files that DECLARE it, and this package has shipped capabilities
reachable from one launch path and not the one `bringup` includes -- five
times. `anchor/loop_closure.py` fixed that for itself and carries a guard test
that fails if the plumbing returns; `ladder_config.py` is that pattern
generalised so the node behaves identically whichever launch started it.

⚠ THE DEFAULTS EXIST IN TWO PLACES -- the node's `declare_parameter` and
`_KEYS` here -- because the node must work with no deck file at all. Two
copies of a default is exactly how they drift, so one test below compares
them directly rather than trusting a comment.
"""
from __future__ import annotations

import types

import pytest

from mongla_vision.ladder_config import (CONFIG_PATH, _KEYS, defaults,
                                         load_config)


def _log():
    seen = {'info': [], 'warn': []}
    return seen, types.SimpleNamespace(
        info=lambda m: seen['info'].append(m),
        warn=lambda m: seen['warn'].append(m))


def _write(tmp_path, text):
    f = tmp_path / 'ladder.yaml'
    f.write_text(text)
    return str(f)


def test_a_missing_file_gives_the_shipped_defaults():
    """The common case: no deck file, and the ladder runs as shipped."""
    assert load_config('/nonexistent/ladder.yaml') == defaults()


def test_a_deck_file_overrides_one_key_and_leaves_the_rest(tmp_path):
    cfg = load_config(_write(tmp_path, 'act_conf: 0.60\n'))
    assert cfg['act_conf'] == pytest.approx(0.60)
    assert cfg['anchor_semi_dense'] is False, \
        'an unset key must keep its default'


def test_a_string_bool_is_accepted(tmp_path):
    """A hand-edited file carries `true`, `True`, `yes` or `on`. Refusing
    those on a pool deck would be pedantry with a cost."""
    for word in ('true', 'True', 'yes', 'on', '1'):
        cfg = load_config(_write(tmp_path, f'anchor_semi_dense: "{word}"\n'))
        assert cfg['anchor_semi_dense'] is True, word
    for word in ('false', 'no', 'off', '0'):
        cfg = load_config(_write(tmp_path, f'anchor_semi_dense: "{word}"\n'))
        assert cfg['anchor_semi_dense'] is False, word


def test_a_malformed_file_degrades_to_defaults_AND_SAYS_SO(tmp_path):
    """⛔ BOTH HALVES MATTER. Taking the vision stack down over a typo on a
    pool deck is unacceptable; so is silently ignoring a file the operator
    just edited, because they would then watch for a behaviour change that
    was never loaded."""
    seen, log = _log()
    cfg = load_config(_write(tmp_path, 'act_conf: [this is not a float\n'), log)
    assert cfg == defaults()
    assert seen['warn'], 'a malformed file must be reported, not swallowed'
    assert 'defaults' in seen['warn'][0]


def test_a_list_instead_of_a_mapping_is_refused(tmp_path):
    seen, log = _log()
    assert load_config(_write(tmp_path, '- act_conf\n- 0.6\n'),
                       log) == defaults()
    assert seen['warn']


def test_an_unknown_key_is_reported_not_ignored(tmp_path):
    """A typo'd switch is indistinguishable from a switch that does nothing,
    and the operator cannot tell which they are looking at."""
    seen, log = _log()
    cfg = load_config(_write(tmp_path, 'act_konf: 0.6\n'), log)
    assert cfg == defaults()
    assert any('unknown key' in w for w in seen['warn'])
    assert any('act_konf' in w for w in seen['warn'])


def test_what_was_loaded_is_logged(tmp_path):
    """An operator must be able to confirm the file took effect."""
    seen, log = _log()
    load_config(_write(tmp_path, 'act_conf: 0.7\n'), log)
    assert any('act_conf=0.7' in m for m in seen['info'])


def test_the_two_copies_of_every_default_agree():
    """⛔ THE DRIFT GUARD. `lock_node.declare_parameter` and `_KEYS` both hold
    a default because the node must work with no deck file. When they
    disagree, the node's behaviour depends on whether a file exists -- which
    is the worst possible way to discover a mismatch."""
    import re
    from pathlib import Path

    src = (Path(__file__).resolve().parents[1] / 'mongla_vision'
           / 'lock_node.py').read_text()
    for name, (_coerce, want) in _KEYS.items():
        m = re.search(rf"declare_parameter\(\s*'{name}'\s*,\s*([^)]+)\)", src)
        if not m:
            continue                      # not every key is a node parameter
        got = m.group(1).strip()
        if isinstance(want, bool):
            assert got == str(want), f'{name}: node says {got}, config {want}'
        else:
            assert float(got) == pytest.approx(float(want)), \
                f'{name}: node says {got}, config {want}'


def test_the_config_path_is_under_the_operator_dotdir():
    """Alongside `~/.mongla/loop_closure.yaml` and `~/.mongla/courses`, so an
    operator has ONE place to look."""
    assert CONFIG_PATH.startswith('~/.mongla/')
