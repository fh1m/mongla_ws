"""The layouts are what views.py generates: one table, so a new sensor or camera
is one row there, and no hand-made layout drifts from it."""
import json
import re

from mongla_batighor import views
from mongla_batighor.seat import CAMERAS


def test_every_committed_layout_is_what_the_table_generates():
    for name in [views.CONSOLE, *views.VIEWS]:
        path = views.LAYOUTS / f'{views.FILE.get(name, name)}.json'
        assert path.read_text() == views.render(name), \
            f'{path.name} is stale: ros2 run mongla_batighor views --write'


def test_no_layout_is_made_by_hand():
    made = {views.CONSOLE} | {views.FILE.get(n, n) for n in views.VIEWS}
    assert {p.stem for p in views.LAYOUTS.glob('*.json')} == made


def test_a_split_is_weighted_as_written():
    tree = views.row((1, 'a'), (1, 'b'), (2, 'c'))
    assert tree['splitPercentage'] == 25.0
    assert tree['second']['splitPercentage'] == 33.3
    assert views.col('only') == 'only'


def test_a_new_camera_reaches_every_per_camera_view(monkeypatch):
    monkeypatch.setattr(views, 'CAMERAS', (*CAMERAS, 'aft'))
    for name in ('cockpit', 'vision', 'weak_link'):
        text = json.dumps(views.build(name))
        assert '/mongla/vision/aft/' in text, name


def test_every_panel_in_a_view_is_placed_once():
    for name in views.VIEWS:
        built = views.build(name)
        tree = json.dumps(built['layout'])
        for pid in built['configById']:
            assert tree.count(f'"{pid}"') == 1, (name, pid)


def test_the_tool_checks_and_writes_the_tree_it_is_pointed_at(tmp_path, capsys):
    assert views.main(['--dir', str(tmp_path / 'nowhere')]) == 2
    assert views.main(['--dir', str(tmp_path)]) == 1          # nothing there yet: all stale
    assert views.main(['--dir', str(tmp_path), '--write']) == 0
    assert views.main(['--dir', str(tmp_path)]) == 0
    assert (tmp_path / 'mongla_layout.json').read_text() == views.render('cockpit')


def _published():
    """Every topic a node here publishes: literal, or its last segment when templated."""
    src = views.LAYOUTS.parents[1]
    out = set()
    for p in src.rglob('*.py'):
        if '/test/' in str(p):
            continue
        for m in re.finditer(r"create_publisher\(\s*[\w.]+\s*,\s*f?'([^']+)'", p.read_text()):
            out.add(m.group(1))
    return out


def test_every_topic_the_stack_publishes_is_shown_somewhere():
    """The scorer for the operator center (bead 102.25.8): a topic is shown by a
    view, read by one of our panels, or drawn by the overlay -- or it is in
    NOT_SHOWN with its reason. A new publisher nobody can see fails here."""
    published = _published()
    assert len(published) >= 45, published          # the scan itself still works
    pkg = views.LAYOUTS.parent
    shown = '\n'.join([*(views.render(n) for n in views.VIEWS),
                       *(p.read_text() for p in (pkg / 'panels').rglob('*.js')),
                       (pkg / 'mongla_batighor' / 'overlay.py').read_text()])
    missing = []
    for topic in sorted(published):
        leaf = topic.split('}', 1)[-1] if topic.startswith('{') else topic
        name = leaf.rsplit('/', 1)[-1]
        if name in views.NOT_SHOWN:
            continue
        if not topic.startswith('{'):
            if f"{topic}" in shown:
                continue
        else:
            # A templated topic counts only under a camera, the calibration, or
            # a template of its own (`${c}`, `{ns}`): `/state` alone would match
            # /mongla/state and prove nothing.
            cams = '|'.join((*CAMERAS, 'calibration'))
            pre = rf'(?:/(?:{cams})|\$\{{\w+\}}|\{{\w+\}})'
            if re.search(pre + re.escape(leaf) + r'\b', shown):
                continue
        missing.append(topic)
    assert not missing, f'published and shown nowhere (add to a view, or NOT_SHOWN): {missing}'


def test_every_not_shown_reason_is_still_about_a_published_topic():
    published = {t.rsplit('/', 1)[-1] for t in _published()}
    for name in views.NOT_SHOWN:
        assert name in published, f'{name} is no longer published: drop it from NOT_SHOWN'


def _images(name):
    return {k: c['imageMode'] for k, c in views.build(name)['configById'].items()
            if k.startswith('Image!')}


def test_a_view_for_judging_boxes_shows_each_frame_with_its_own():
    """Boxes come ~2 frames after the picture; unsynced, a box trails a moving
    target. Synced, the picture stops with the detector, so never in the
    cockpit, where a live picture comes first."""
    for name in ('vision', 'bins_downward'):
        cams = [m for m in _images(name).values() if m['imageTopic'].endswith('stream/compressed')]
        assert cams and all(m.get('synchronize') for m in cams), name
    assert not any(m.get('synchronize') for m in _images('cockpit').values())


def test_every_plot_follows_the_last_minute_with_a_narrow_legend():
    """Unset, a plot spans the whole connection and a panel opened later draws
    in a sliver at the right; Lichtblick's 240 px legend took half a plot."""
    plots = [c for name in views.VIEWS for k, c in views.build(name)['configById'].items()
             if k.startswith('Plot!')]
    assert plots
    for c in plots:
        assert c['followingViewWidth'] == views.PLOT_WINDOW_S and c['sidebarDimension'] <= 200
    # A synced state panel pulled the plots beside it out of following.
    for name in views.VIEWS:
        for k, c in views.build(name)['configById'].items():
            if k.startswith('StateTransitions!'):
                assert c['isSynced'] is False, (name, k)
                # Unsynced with no range it spans the whole connection: two
                # hours in, the last minute was a sliver and read as empty.
                assert c['xAxisRange'] == views.PLOT_WINDOW_S, (name, k)


def test_no_plot_mixes_units():
    """A plot has one axis. Depth (-0.5 m) beside the battery (12.4 V) drew
    depth as a flat line at the floor (board_health, desktop app 2026-11-12);
    the pose plot did the same with metres, degrees and pixels."""
    unit = re.compile(r'(?<![\w/])(m/s|m|V|deg|px|Hz|%|rpm|bytes/s)(?![\w/])')
    for name in views.VIEWS:
        for k, c in views.build(name)['configById'].items():
            if k.startswith('Plot!'):
                units = {u for p in c['paths'] for u in unit.findall(p.get('label', ''))}
                assert len(units) <= 1, (name, k, units)


def test_every_plot_says_what_it_is_and_its_legend_fits():
    """Untitled, every plot read "Plot", so its legend had to name it and was
    cut: "forward author", "bearing de" (desktop app, 2026-11-13). The title
    names the plot; a label names one line, short enough for the legend."""
    for name in views.VIEWS:
        for k, c in views.build(name)['configById'].items():
            if k.startswith(('Plot!', 'StateTransitions!')):
                assert c.get('lichtblickPanelTitle'), (name, k)
            if k.startswith('Plot!'):
                for p in c['paths']:
                    assert len(p.get('label', '')) <= views.LABEL_CHARS, (name, k, p['label'])


def test_the_console_holds_every_view_as_a_tab_with_its_own_panels():
    """One layout, a tab per view, built by the same functions: every panel a
    view places is in its tab, under an id no other tab uses."""
    data = views.build(views.CONSOLE)
    tabs = data['configById']['Tab!console']['tabs']
    assert [t['title'] for t in tabs] == [title for _, title in views.TABS]
    assert set(v for v, _ in views.TABS) <= set(views.VIEWS)
    for (view, _), tab in zip(views.TABS, tabs):
        alone = views.build(view)
        placed = set(re.findall(r'"([^"!]+![^"]+)"', json.dumps(tab['layout'])))
        assert placed == {pid.replace('!', f'!{view}.', 1) for pid in alone['configById']}
        for pid in placed:
            assert data['configById'][pid] == alone['configById'][pid.replace(f'!{view}.', '!', 1)]


# Lichtblick draws each state row 64 px tall, plus 30 for the axis, and
# scrolls what does not fit (its StateTransitions bundle, 2026-11-21). A tab's
# body is ~860 px in the desktop app on a 1080 p screen, and a panel's title
# bar takes ~28 of its own.
STATE_ROW_PX, STATE_AXIS_PX, TAB_BODY_PX, TITLE_PX = 64, 30, 860, 28


def _heights(node, h, out):
    if isinstance(node, str):
        out[node] = h
        return out
    f = node['splitPercentage'] / 100 if node['direction'] == 'column' else 1.0
    _heights(node['first'], h * f, out)
    _heights(node['second'], h * (1 - f) if node['direction'] == 'column' else h, out)
    return out


def test_every_state_row_is_visible_without_scrolling():
    """The Vision tab's lock ladder had seven rows in ~150 px: the forward
    camera's two showed and the downward camera's were below the fold. The
    Localization tab's aiding strip showed one row of five."""
    data = views.build(views.CONSOLE)
    for tab in data['configById']['Tab!console']['tabs']:
        for pid, h in _heights(tab['layout'], TAB_BODY_PX, {}).items():
            if pid.startswith('StateTransitions!'):
                rows = len(data['configById'][pid]['paths'])
                need = rows * STATE_ROW_PX + STATE_AXIS_PX
                assert h - TITLE_PX >= need, (tab['title'], pid, round(h), need)
