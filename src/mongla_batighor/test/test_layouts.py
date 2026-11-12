"""Every topic a shipped Foxglove/Lichtblick layout plots must be one we publish.

A layout pointing at a renamed topic shows an empty panel at the pool, which
looks exactly like a dead sensor.
"""
import json
import pathlib
import re

import pytest

_SRC = pathlib.Path(__file__).resolve().parents[2]
_LAYOUTS = sorted((_SRC / 'mongla_batighor' / 'layouts').glob('*.json'))


def _published():
    text = '\n'.join(p.read_text(errors='ignore') for p in _SRC.rglob('*.py')
                     if '/test/' not in str(p))
    return text


def _topics(layout):
    found = set()
    for cfg in layout['configById'].values():
        image = cfg.get('imageMode', {})
        for v in [cfg.get('topicPath'), image.get('imageTopic')] \
                + list(image.get('annotations', {})) \
                + [p['value'] for p in cfg.get('paths', [])]:
            if v:
                found.add(v.split('.', 1)[0])
    return found


@pytest.mark.parametrize('path', _LAYOUTS, ids=lambda p: p.name)
def test_every_layout_topic_is_published_somewhere(path):
    src = _published()
    layout = json.loads(path.read_text())
    for topic in _topics(layout):
        if topic.startswith('/mongla/move/_action'):
            continue                                    # the action server's own
        # Matched on the last two segments, since 'compressed' alone names
        # every image topic. A camera's own topics are built from `{ns}`.
        cam, leaf = topic.rsplit('/', 2)[-2:]
        literal = topic in src
        alts = [rf"f'[^']*/{re.escape(cam)}/{re.escape(leaf)}'"]
        if cam in ('forward', 'downward'):
            alts.append(rf"f'[^']*{{ns}}/{re.escape(leaf)}'")
        templated = any(re.search(a, src) for a in alts)
        assert literal or templated, f'{path.name}: nothing publishes {topic}'


def test_the_layouts_are_found():
    # A moved folder parametrizes the test above over nothing, and passes.
    assert len(_LAYOUTS) >= 5, _LAYOUTS
