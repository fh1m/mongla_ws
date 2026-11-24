"""Batighor plotted target_pose_fused.bearing_deg and .reproj_px, which the
fuser never sets: two legends with no line (audit 2026-11-24, batighor_6).
A fused field is plotted only if `fused_message` assigns it."""
import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
LAYOUTS = ROOT / 'src/mongla_batighor/layouts'
FUSER = ROOT / 'src/mongla_vision/mongla_vision/pose_fuse_node.py'


def test_every_plotted_fused_field_is_set_by_the_fuser():
    src = FUSER.read_text()
    plotted = set()
    for f in LAYOUTS.glob('*.json'):
        plotted |= set(re.findall(r'target_pose_fused\.(\w+)', f.read_text()))
    assert plotted, 'no fused series found: the guard reads nothing'
    unset = sorted(x for x in plotted if not re.search(rf'out\.{x}\s*=', src))
    assert not unset, f'plotted but never set by fused_message: {unset}'
    json.loads((LAYOUTS / 'batighor.json').read_text())
