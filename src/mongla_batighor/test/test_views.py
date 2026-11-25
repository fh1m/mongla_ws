"""The layouts are what views.py generates: one table, so a new sensor or camera
is one row there, and no hand-made layout drifts from it."""
import json
import math
import re
from pathlib import Path

from mongla_batighor import views
from mongla_batighor.seat import CAMERAS


def test_every_committed_layout_is_what_the_table_generates():
    for name in [*views.CONSOLES, *views.VIEWS]:
        path = views.LAYOUTS / f'{views.FILE.get(name, name)}.json'
        assert path.read_text() == views.render(name), \
            f'{path.name} is stale: ros2 run mongla_batighor views --write'


def test_no_layout_is_made_by_hand():
    made = set(views.CONSOLES) | {views.FILE.get(n, n) for n in views.VIEWS}
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
    for name in ('vision', 'downward'):
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
    unit = re.compile(r'(?<![\w/])(m/s|m|V|deg|rad|ratio|px|Hz|%|rpm|bytes/s)(?![\w/])')
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


def _console_tabs():
    """(the console's layout, one of its tabs), for every console."""
    for data in map(views.build, views.CONSOLES):
        for tab in data['configById']['Tab!console']['tabs']:
            yield data, tab


def test_the_pilot_console_starts_on_pilot_and_repeats_nothing_vitals_shows():
    """The main screen beside Vitals: the tab bar is the index, Pilot first;
    no Cockpit tab, which carries the keys and the state Vitals already shows
    (seen twice on two screens, 2026-11-24), and the slim Watch strip in
    place of the full one."""
    data = views.build(views.PILOT_CONSOLE)
    tabs = [t['title'] for t in data['configById']['Tab!console']['tabs']]
    assert tabs[0] == 'Pilot' and 'Cockpit' not in tabs
    assert set(tabs) == {t for _, t in views.TABS if t != 'Cockpit'} | {'Pilot'}
    assert data['layout']['second'] == 'Tab!console'
    assert data['layout']['first'] == views.PANEL + 'Watch!status'
    placed = json.dumps(data['configById']['Tab!console']['tabs'][0])
    assert 'ControlCenter!' not in placed and 'Vehicle!' not in placed and 'Keys!' not in placed


def test_every_console_carries_a_strip_above_its_tabs():
    """The strip marks the tabs and reports this screen's lag to the helm,
    which sets the shared video for the slowest screen. The pilot console
    first shipped without one: no tab marks, and its lag never reported."""
    for name in views.CONSOLES:
        layout = views.build(name)['layout']
        assert layout['second'] == 'Tab!console', name
        assert layout['first'].split('!')[0] in (views.PANEL + 'Status', views.PANEL + 'Watch'), name


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
    for data, tab in _console_tabs():
        for pid, h in _heights(tab['layout'], TAB_BODY_PX, {}).items():
            if pid.startswith('StateTransitions!'):
                rows = len(data['configById'][pid]['paths'])
                need = rows * STATE_ROW_PX + STATE_AXIS_PX
                assert h - TITLE_PX >= need, (tab['title'], pid, round(h), need)


def _all_configs():
    for name in [*views.CONSOLES, *views.VIEWS]:
        yield from ((name, k, c) for k, c in views.build(name)['configById'].items())


def test_everything_drawn_over_a_picture_or_in_the_scene_is_visible():
    """An overlay the layout names but hides is an operator seeing no boxes
    while the overlay node works; a hidden scene topic hides the demand arrow."""
    for name, k, c in _all_configs():
        for topic, a in c.get('imageMode', {}).get('annotations', {}).items():
            assert a.get('visible') is True, (name, k, topic)
        for topic, s in c.get('topics', {}).items():
            assert s.get('visible') is True, (name, k, topic)


def test_only_the_detection_views_make_a_picture_wait_for_its_boxes():
    """Synchronised, a picture is shown only with its boxes, so it froze
    whenever the detector stopped. Vision, Forward and Downward are where a box must sit
    on its own frame; everywhere else (the cockpit, the floor read, a
    calibration) a live picture comes first (views.camera)."""
    synced = {'vision', 'forward', 'downward'}
    seen = set()
    for name in views.VIEWS:
        for k, c in views.build(name)['configById'].items():
            if k.startswith('Image!'):
                seen.add(name)
                assert bool(c['imageMode'].get('synchronize')) == (name in synced), (name, k)
    assert synced <= seen and 'cockpit' in seen


def test_plots_line_up_and_name_their_lines():
    """Synced, the plots in one view share their time axis; each shows its
    legend, the only place a line is named."""
    for name, k, c in _all_configs():
        if k.startswith('Plot!'):
            assert c['isSynced'] is True and c['showLegend'] is True, (name, k)
            assert c['xAxisVal'] == 'timestamp', (name, k)


def test_the_3d_scene_draws_the_hull_not_frame_names():
    """Seven frame names drawn on one point said nothing (views.scene)."""
    scenes = [(name, k, c) for name, k, c in _all_configs() if k.startswith('3D!')]
    assert scenes
    for name, k, c in scenes:
        assert c['scene']['transforms']['showLabel'] is False, (name, k)
        assert c['layers']['hull']['visible'] is True, (name, k)
        assert c['followTf'] == 'mongla', (name, k)


TAB_BODY_W_PX = 1884                  # the desktop app's window at scale 2, measured
MOST_EMPTY = 0.15


def _vehicle_aspect():
    """Width over height of the vehicle's camera profiles, from cameras.yaml:
    one truth. The test once assumed 640x480 (the replay footage); the
    vehicle streams 640x360, and its pictures sat in panels 4:3 wide."""
    import yaml
    cams = yaml.safe_load((Path(views.__file__).resolve().parents[2] / 'mongla_vision'
                           / 'config' / 'cameras.yaml').read_text())
    cams = next(v for v in cams.values() if isinstance(v, dict) and 'cameras' in v)['cameras']
    shapes = {cams[p]['width'] / cams[p]['height'] for p in ('pi_forward', 'pi_downward')}
    assert len(shapes) == 1, shapes
    return shapes.pop()


CAMERA_ASPECT = _vehicle_aspect()
CONTROL_PX = 600                      # keys, helm, command, mission and a few log lines


def _boxes(node, w, h, out):
    if isinstance(node, str):
        out[node] = (w, h)
        return out
    f = node['splitPercentage'] / 100
    if node['direction'] == 'row':
        _boxes(node['first'], w * f, h, out)
        _boxes(node['second'], w * (1 - f), h, out)
    else:
        _boxes(node['first'], w, h * f, out)
        _boxes(node['second'], w, h * (1 - f), out)
    return out


def test_a_camera_picture_fills_its_panel():
    """The Cockpit's cameras sat in 1540 px wide panels showing a 4:3 picture
    890 px wide: four tenths of the screen was black (2026-11-22). A camera
    panel is sized to its picture; the calibration's cameras vary and are left
    out."""
    for data, tab in _console_tabs():
        if tab['title'] == 'Calibrate':
            continue
        for pid, (w, h) in _boxes(tab['layout'], TAB_BODY_W_PX, TAB_BODY_PX, {}).items():
            if pid.startswith('Image!'):
                h -= TITLE_PX
                fit = min(w, h * CAMERA_ASPECT) * min(h, w / CAMERA_ASPECT)
                empty = 1 - fit / (w * h)
                assert empty <= MOST_EMPTY, (tab['title'], pid, round(w), round(h), round(empty, 2))


def test_the_control_center_shows_every_key_without_scrolling():
    """ARM and SEND were below the fold of a 320 px Control Center; a key that
    needs a scroll first is not there when it is wanted."""
    seen = 0
    for data, tab in _console_tabs():
        for pid, (_, h) in _boxes(tab['layout'], TAB_BODY_W_PX, TAB_BODY_PX, {}).items():
            if pid.startswith(views.PANEL + 'ControlCenter!'):
                seen += 1
                assert h - TITLE_PX >= CONTROL_PX, (tab['title'], pid, round(h))
    assert seen


def test_every_tab_carries_its_glyph():
    """The theme keys each tab's glyph on its title: renamed from Bins to
    Downward, the tab showed a blank square (desktop app, 2026-11-22)."""
    css = (Path(views.__file__).resolve().parents[1] / 'theme' / 'batighor.css').read_text()
    keyed = set(re.findall(r'\[data-testid="toolbar-tab"\]\[title="([^"]+)"\]\{--mb-glyph', css))
    titles = {t for tabs in views.CONSOLES.values() for _, t in tabs}
    assert titles <= keyed, titles - keyed
    assert keyed <= titles, f'a glyph for no tab: {keyed - titles}'


def test_lock_authority_keeps_its_whole_range():
    """Authority is 0..1 by construction (`lock_state.authority_for`).
    Autoscaled, a steady 1.0 drew on a 0.94-1.06 axis and read as noise
    (Kutum, 2026-11-22)."""
    import json
    import pathlib
    found = 0
    for f in (pathlib.Path(__file__).resolve().parents[1] / 'layouts').glob('*.json'):
        for cfg in json.loads(f.read_text()).get('configById', {}).values():
            paths = [p.get('value', '') for p in cfg.get('paths', []) if isinstance(p, dict)]
            if any('/lock' in p and p.endswith('.score') for p in paths):
                found += 1
                assert cfg.get('minYValue') == 0 and cfg.get('maxYValue', 0) >= 1, f.name
    assert found, 'no authority plot found: the guard checks nothing'


def test_a_fused_pose_plot_reads_only_what_the_fuse_fills():
    # pose_fuse_node sets the floats it computes and leaves the rest NaN, so
    # a series on any other field is a line that is never drawn.
    fuse = (views.LAYOUTS.parents[1] / 'mongla_vision' / 'mongla_vision'
            / 'pose_fuse_node.py').read_text()
    filled = set(re.findall(r'\bout\.(\w+) =', fuse))
    for name in views.VIEWS:
        for field in re.findall(r'target_pose_fused\.(\w+)', json.dumps(views.build(name))):
            assert field in filled, (name, field)


def test_the_two_screens_share_no_panel_and_hold_one_3d_view():
    """Above, the pilot's tab; below, Vitals. A hull on each, both close in,
    showed nothing either needed, and the map, keys and state were each on
    both at one time or another (owner, 2026-11-24)."""
    kinds = {name: [pid.split('!')[0] for pid in views.build(name)['configById']]
             for name in ('pilot', 'vitals')}
    assert not set(kinds['pilot']) & set(kinds['vitals'])
    assert (kinds['pilot'] + kinds['vitals']).count('3D') == 1
    (cfg,) = [c for p, c in views.build('pilot')['configById'].items() if p.startswith('3D!')]
    assert '/mongla/batighor/pool_scene' in cfg['topics']
    assert cfg['cameraState']['distance'] >= 4             # the pool, not the hull alone


def test_no_two_tabs_of_a_console_are_the_same_view_twice():
    """Forward, Downward, Chain and Vision all showed the cameras; Map and
    Mission both the pool with the mission (2026-11-24)."""
    for name, tabs in views.CONSOLES.items():
        assert len({v for v, _ in tabs}) == len(tabs) <= 14, name
        both = [t for v, t in tabs if all(f'/{c}/stream' in json.dumps(views.build(v)['configById'])
                                          for c in ('forward', 'downward'))]
        assert len(both) <= 2, (name, both)      # the first tab, and Vision in depth
        maps = [t for v, t in tabs if f'{views.PANEL}Map!' in json.dumps(views.build(v)['configById'])]
        assert len(maps) <= 1 + (name == views.CONSOLE), (name, maps)


def test_a_3d_view_draws_its_text_at_its_own_size_and_no_stray_odometry():
    """Every 3D view: labelScaleFactor 1, so the demand's caption and the
    pool's names are drawn at the font_size overlay.py and course.py give
    them (at 0.3 they were 6 px and 4 px); no /mongla/odom arrow, which
    repeats the hull's pose; and no frame-to-parent line, the yellow one
    that ran from the hull off the panel."""
    import json as _json
    found = 0
    for f in views.LAYOUTS.glob("*.json"):
        for k, cfg in _json.loads(f.read_text())['configById'].items():
            if k.startswith('3D!'):
                found += 1
                assert cfg['scene']['labelScaleFactor'] == 1.0, (f.name, k)
                assert '/mongla/odom' not in cfg.get('topics', {}), (f.name, k)
                assert cfg['scene']['transforms']['lineWidth'] == 0, (f.name, k)
    assert found


def test_the_demand_caption_is_the_size_of_the_pools_names():
    """One size for every caption in the 3D scenes: larger, the demand's
    covered the hull in the pool view (Chrome, 2026-11-25)."""
    import re as _re
    from mongla_batighor import overlay
    src = (views.LAYOUTS.parent / 'mongla_batighor' / 'course.py').read_text()
    (px,) = _re.findall(r'def text\(x, y, z, words, c, px=([\d.]+)\)', src)
    assert overlay.DEMAND_LABEL_PX == float(px)


def test_the_whole_hull_fits_the_tall_attitude_panel():
    """The cockpit's attitude panel is about 27 degrees wide (45 high, tall and
    narrow). At 0.95 m the 0.702 m hull ran off both sides of it, and the fix
    once closed its PR without ever being committed."""
    cam = views._scene()['cameraState']
    wide = 2 * cam['distance'] * math.tan(math.radians(27 / 2))
    assert wide >= 0.702, f"{wide:.2f} m across for a 0.702 m hull"


def test_a_wrapped_heading_says_its_modulus():
    """The floor grid repeats every 90 deg and a lane line every 180: beside the
    board's heading they can differ by that much and agree (audit batighor_6 5)."""
    text = views.render('localization')
    assert 'grid mod 90' in text and 'lane mod 180' in text
