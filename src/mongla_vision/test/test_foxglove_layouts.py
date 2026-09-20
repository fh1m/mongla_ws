"""Every topic a shipped Foxglove/Lichtblick layout plots must be one we publish.

A layout pointing at a renamed topic shows an empty panel at the pool, which
looks exactly like a dead sensor.
"""
import json
import pathlib
import re

import pytest

_SRC = pathlib.Path(__file__).resolve().parents[2]
_LAYOUTS = sorted((_SRC / 'mongla_vision' / 'foxglove').glob('*.json'))


def _published():
    text = '\n'.join(p.read_text(errors='ignore') for p in _SRC.rglob('*.py')
                     if '/test/' not in str(p))
    return text


def _topics(layout):
    found = set()
    for cfg in layout['configById'].values():
        for v in [cfg.get('topicPath'), cfg.get('imageMode', {}).get('imageTopic')] \
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
        tail = topic.rsplit('/', 1)[-1]
        literal = topic in src
        templated = re.search(rf"f'[^']*/{re.escape(tail)}'", src) is not None
        assert literal or templated, f'{path.name}: nothing publishes {topic}'
