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
        for v in [cfg.get('topicPath'), cfg.get('topicToRender'), image.get('imageTopic')] \
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


# std_msgs types carry no header. Lichtblick 1.30 plots a path by its header
# stamp unless the path says `receiveTime`, so a plot of one of these topics
# without it stays empty while the topic publishes (seen on Kutum, 2026-11-12,
# the `flow` layout's distance and quality plots).
_HEADERLESS = ('Float32', 'Float64', 'UInt8', 'Int32', 'Bool', 'String',
               'Float32MultiArray', 'Float64MultiArray')


def _headerless(src, topic):
    # On the last two segments, as above: the leaf alone let a calibration's
    # `{NS}/state` (a String) pass for /mongla/state (stamped).
    parent, leaf = topic.rsplit('/', 2)[-2:]
    names = [rf"f?'[^']*/{re.escape(parent)}/{re.escape(leaf)}'"]
    if parent in ('forward', 'downward'):
        names.append(rf"f'[^']*{{ns}}/{re.escape(leaf)}'")
    kinds = '|'.join(_HEADERLESS)
    return any(re.search(rf"create_publisher\(\s*(?:{kinds})\s*,\s*{n}", src)
               for n in names)


@pytest.mark.parametrize('path', _LAYOUTS, ids=lambda p: p.name)
def test_a_plot_of_a_headerless_topic_is_stamped_on_arrival(path):
    src = _published()
    for name, cfg in json.loads(path.read_text())['configById'].items():
        for p in cfg.get('paths', []):
            topic = p['value'].split('.', 1)[0]
            if _headerless(src, topic):
                assert p.get('timestampMethod') == 'receiveTime', (
                    f'{path.name} {name}: {topic} has no header, so this plot '
                    f'draws nothing unless the path sets timestampMethod receiveTime')


def test_the_headerless_check_sees_a_headerless_topic():
    # A regex that matches nothing passes every layout above.
    src = _published()
    assert _headerless(src, '/mongla/vision/downward/flow_quality')
    assert not _headerless(src, '/mongla/vision/downward/velocity')
    assert not _headerless(src, '/mongla/state')
