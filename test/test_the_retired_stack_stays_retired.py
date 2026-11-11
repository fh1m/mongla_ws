"""The retired stack stays retired -- in code, docs, config, skills, agents and tests.

Why this exists. The Pixhawk / ArduSub backend, the Jetson, the BNO085/DVL sensor
package and the Gazebo + SITL simulator left `main` on 2026-11-06 (tag
`last-with-pixhawk-and-sim`). The code went in one commit. The prose did not: a
sweep the same day found **53 live files** still describing the old vehicle -- a
package page drawing `PixhawkFC` beside `SrotFC`, a runbook listing six "refused"
verbs that no longer existed, a video guide whose first step was `sim_vehicle.py
-v ArduSub`, a doc telling missions to call `mongla.can()`, an agent pointing at a
deleted page, and a `downward-camera.md` documenting a fill-driven descent that
`align_loop` now raises `ValueError` on. Each one looked finished. Each one would
have cost a pool day.

The operator consoles Batighor replaced are retired the same way: the mission web
console, with its `web_video_server` stream, left on 2026-11-12, and only the
ledger and the defect history may still name them.

The earlier guard checked four exact strings in four entry points. This one reads
EVERY file git knows about (tracked and untracked-not-ignored), so a stale claim
cannot hide in a skill, an agent prompt, a test docstring or a launch comment.

What is allowed, and why -- every exemption carries its reason:

* A paragraph that names the tag `last-with-pixhawk-and-sim` is a pointer to where
  the old stack lives, which is the one thing a reader should be told about it.
* Filenames on the FIRMWARE team's repository (`JETSON_COMMS.md`, ...) are theirs to
  rename; we reference them as they are (a PR to rename them is the fix).
* The paths in `EXEMPT` describe the old platform BY DESIGN: the before-and-after
  documents, prior art about other teams, the defect history, and the tests that
  name a retired token in order to ban it.

Read as text, never imported: the files are the artefact under test.
"""
from __future__ import annotations

import re
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
TAG = 'last-with-pixhawk-and-sim'

# Each token only ever appears when a file describes the retired stack as if it
# were live (or names a symbol that was deleted with it). Case-insensitive.
TOKENS = re.compile(r'''
      pixhawk | ardusub | jetson | \borin\b | \bsitl\b | gazebo | blueos | nvpmodel
    | tensorrt | mongla_sensors | dvl_connect | flight_controller
    | \b(?:un)?lock_heading\b | release_heading | heading_lock
    | \bmove_(?:forward|back|lateral|left|right)_dist\b | \bstyle_yaw\b
    | UNSUPPORTED_VERBS | PayloadDriver
    | mongla\.can\( | mongla\.require\( | mongla\.backend\b
    | mongla_sim | sim_vehicle | \bsim/src\b | sim/COLCON_IGNORE
    | mission_web | dashboard_state | web_video_server
''', re.I | re.X)

# Names on the firmware team's repositories. Stripped before matching.
FOREIGN = ('JETSON_COMMS', 'JETSON_FEEDBACK')

# path prefix -> why it may name the retired stack
EXEMPT = {
    '.claude/context/sota/': 'prior art: what other teams run is a fact about them',
    '.claude/context/upstream/': 'asks sent to the firmware team, quoted as sent',
    '.claude/context/scouting/': 'what other teams run is a fact about them',
    '.claude/context/BUGS.md': 'the defect history, including the retired backend',
    'docs/evidence/': 'the water lineage: which vehicle, autopilot and stack was in water (#87)',
    '.claude/context/measured-bars.md': 'the ledger keeps every number with where it was measured',
    '.claude/context/the-shift.md': 'the platform change, before and after',
    'docs/the-shift.md': 'the platform change, before and after',
    '.claude/context/capability-map.md': 'each capability beside what the old stack could do',
    'docs/capability-map.md': 'each capability beside what the old stack could do',
    '.claude/context/the-whole-chain.md': 'what other teams buy, against what we own',
    '.claude/context/kutum/pi-hailo-vision-box.md': 'the measurement that chose the Pi, against the companion it replaced',
    '.claude/context/perception/hailo-vision.md': 'the chip measured against the baseline it replaced',
    '.claude/skills/bumblebee-doctrine/': 'describes the champions\' stack, which runs ArduSub',
    'tools/site/make_diagrams.py': 'draws the BEFORE half of the-shift diagram',
    '.claude/settings.local.json': 'user-local permission history, not ours to rewrite',
    # tests that name a retired token in order to ban it
    'test/test_the_retired_stack_stays_retired.py': 'this file',
    'src/mongla_planner/test/test_operator_commands_are_real.py': 'injects a retired command to prove the lint catches it',
    'src/mongla_manager/test/test_launch_and_node_defaults_agree.py': 'bans the retired launch arguments',
    'src/mongla_manager/test/test_preflight_never_opens_the_autopilot.py': 'bans the retired payload driver and flags',
    'src/mongla_manager/test/test_manager_end_to_end.py': 'sends a retired verb to prove it puts nothing on the wire',
    'src/mongla_planner/test/test_every_mission_runs_against_the_real_dsl.py': 'names the retired calls it exists to catch',
    'src/mongla_manager/test/test_docs_contract.py': 'states what the docs must not say',
}

BINARY = {'.png', '.jpg', '.jpeg', '.webp', '.gif', '.svg', '.mp4', '.mkv', '.avi',
          '.hef', '.pt', '.onnx', '.mcap', '.db3', '.bag', '.npz', '.npy', '.pdf',
          '.woff', '.woff2', '.ttf', '.otf', '.ico', '.zip', '.gz', '.tar', '.bin'}


def _files():
    out = subprocess.run(
        ['git', 'ls-files', '-z', '--cached', '--others', '--exclude-standard'],
        cwd=ROOT, capture_output=True, check=True).stdout.decode()
    for rel in sorted(set(filter(None, out.split('\0')))):
        p = ROOT / rel
        if p.suffix.lower() in BINARY or not p.is_file():
            continue
        yield rel, p


def _exempt(rel: str) -> bool:
    return any(rel == k or (k.endswith('/') and rel.startswith(k)) for k in EXEMPT)


def _hits(text: str):
    """(line_no, line) for every retired token outside a paragraph naming the tag."""
    lines = text.splitlines()
    para_has_tag, start = {}, 0
    for i in range(len(lines) + 1):
        if i == len(lines) or not lines[i].strip():
            tagged = any(TAG in l for l in lines[start:i])
            for j in range(start, i):
                para_has_tag[j] = tagged
            start = i + 1
    for i, line in enumerate(lines):
        if para_has_tag.get(i):
            continue
        probe = line
        for f in FOREIGN:
            probe = probe.replace(f, '')
        if TOKENS.search(probe):
            yield i + 1, line


def test_no_live_file_describes_the_retired_stack():
    scanned, bad = 0, []
    for rel, p in _files():
        if _exempt(rel):
            continue
        try:
            text = p.read_text(encoding='utf-8')
        except (UnicodeDecodeError, OSError):
            continue
        scanned += 1
        for n, line in _hits(text):
            bad.append(f'{rel}:{n}: {line.strip()[:110]}')
    assert scanned > 300, f'scanned only {scanned} files -- git ls-files broke, not the repo'
    assert not bad, (
        f'{len(bad)} live line(s) describe the retired stack. Say what the code does '
        f'today, point at the tag `{TAG}`, or -- if the file is history by design -- add '
        f'it to EXEMPT with the reason:\n  ' + '\n  '.join(bad[:40]))


def test_every_exemption_still_exists():
    """An exemption for a file that is gone is a hole waiting for a new file."""
    gone = [k for k in EXEMPT if not (ROOT / k).exists()]
    assert not gone, f'EXEMPT names paths that no longer exist: {gone}'


@pytest.mark.parametrize('line, caught', [
    ('fc = PixhawkFC(port)', True),
    ('mongla.lock_heading(target=0.0)', True),
    ('see JETSON_COMMS.md for the wire', False),
    ('the medium model on the Orin Nano', True),
    ('scoring every frame, storing the view', False),        # `orin` inside words
    ('the old stack lives at `last-with-pixhawk-and-sim`', False),
    ("_SIM_SPEC = ROOT / 'sim' / 'src' / 'mongla_sim_worlds'", True),
])
def test_the_matcher_bites_and_does_not_overreach(line, caught):
    assert bool(list(_hits(line))) is caught
