"""An unknown class name must not filter EVERY detection away.

⛔ `HailoDetector.update_allowlist` mapped requested names to class ids. If a
name did not exist in the model, it contributed no id -- and if EVERY requested
name was unknown, the allowlist became an empty set, which filters everything.
The detector then returns [] on every frame, the graph publishes at full rate,
and one WARN is the only sign. That is the silent-[] failure `CLAUDE.md` §4 has
a rule against, reached from a different direction: one typo'd class name.

Refusing to narrow is the safe direction -- the caller gets everything the model
found and can filter downstream, rather than getting nothing and believing the
water is empty.
"""
import ast
from pathlib import Path

SRC = (Path(__file__).resolve().parents[1] / 'mongla_vision' / 'detection'
       / 'hailo.py')


def _update_allowlist_source() -> str:
    tree = ast.parse(SRC.read_text())
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == 'update_allowlist':
            return ast.get_source_segment(SRC.read_text(), node)
    raise AssertionError('update_allowlist not found')


def test_all_unknown_names_fall_back_to_allow_all():
    src = _update_allowlist_source()
    assert 'if wanted and not ids:' in src, (
        'update_allowlist does not special-case "every requested name is '
        'unknown", so it assigns an EMPTY allowlist and the detector goes '
        'blind on every frame while the pipeline looks healthy.')
    i = src.index('if wanted and not ids:')
    after = src[i:i + 700]
    assert 'self._allow_ids = None' in after, (
        'the all-unknown branch must widen to allow-all, not narrow to none')


def test_an_empty_request_still_means_allow_all():
    """The pre-existing, correct behaviour: no allowlist asked for = no filter."""
    src = _update_allowlist_source()
    assert 'if not wanted:' in src and 'self._allow_ids = None' in src


def test_a_known_name_still_narrows():
    """The guard must not have turned the allowlist into a no-op."""
    src = _update_allowlist_source()
    assert 'self._allow_ids = ids' in src, (
        'a valid allowlist must still narrow, or the class filter is dead')
