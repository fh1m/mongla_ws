"""The stage graph: each node in its stage's column, one edge per joined pair.

Every case is built so the answer is known before the code runs.
"""
import json
import pathlib
import shutil
import subprocess

import pytest

from mongla_batighor import graph

PANELS = pathlib.Path(__file__).resolve().parents[1] / 'panels'


def test_each_node_lands_in_its_stage_and_an_unknown_one_is_not_hidden():
    names = ['/mongla_camera_forward', '/mongla_detector_dual', '/mongla_pnp_near_forward',
             '/mongla_pose_fuse_downward', '/batighor_helm', '/foxglove_bridge', '/someone_new']
    g = graph.group([(n, [], []) for n in names])
    cols = {c['stage']: c['nodes'] for c in g['columns']}
    assert cols['camera'] == ['/mongla_camera_forward']
    assert cols['detector'] == ['/mongla_detector_dual']
    assert cols['pose'] == ['/mongla_pnp_near_forward', '/mongla_pose_fuse_downward']
    assert cols['batighor'] == ['/batighor_helm', '/foxglove_bridge']
    assert cols['other'] == ['/someone_new']
    assert [c['stage'] for c in g['columns']] == ['camera', 'detector', 'pose', 'batighor', 'other']


def test_an_edge_names_every_topic_between_two_nodes_and_rosout_is_none():
    cam, det, me = '/mongla_camera_forward', '/mongla_detector_forward', '/mongla_manager'
    g = graph.group([
        (cam, ['/img', '/info', '/rosout'], ['/parameter_events']),
        (det, ['/dets', '/rosout'], ['/img', '/info', '/parameter_events']),
        (me, ['/rosout', '/state'], ['/demand_in', '/rosout']),
    ])
    assert g['edges'] == [{'from': cam, 'to': det, 'topics': ['/img', '/info']}]
    assert g['unread'] == ['/dets', '/state']
    assert g['unfed'] == ['/demand_in']
    assert g['nodes'] == 3


def test_a_node_reading_its_own_topic_makes_no_edge():
    g = graph.group([('/mongla_lock_forward', ['/x'], ['/x'])])
    assert g['edges'] == [] and g['unread'] == [] and g['unfed'] == []


@pytest.mark.skipif(not shutil.which('node'), reason='node not installed (the Pi, CI)')
def test_the_layout_puts_columns_apart_and_rows_down():
    from mongla_batighor.panels import verbs
    g = graph.group([('/mongla_camera_forward', ['/img'], []), ('/mongla_camera_downward', [], []),
                     ('/mongla_detector_forward', [], ['/img'])])
    pre = ('const module={exports:{}};const CAMERAS=["forward","downward"],TUNABLE={},THEME_CSS="";'
           f'const VERBS={json.dumps(verbs())};')
    js = (pre + (PANELS / 'common.js').read_text() + (PANELS / 'instruments' / 'extension.js').read_text()
          + f'\nconst L=module.exports.graphLayout({json.dumps(g)});'
          + 'process.stdout.write(JSON.stringify(L.at));')
    out = subprocess.run(['node', '-e', js], capture_output=True, text=True, timeout=30)
    assert out.returncode == 0, out.stderr
    at = json.loads(out.stdout)
    cam_f, cam_d, det = at['/mongla_camera_forward'], at['/mongla_camera_downward'], at['/mongla_detector_forward']
    assert cam_f['col'] == cam_d['col'] == 0 and det['col'] == 1
    assert det['x'] > cam_f['x'] + cam_f['w']             # columns do not overlap
    assert cam_d['y'] < cam_f['y']                        # sorted: downward above forward
