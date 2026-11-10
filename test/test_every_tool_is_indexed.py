"""Nothing in tools/ is a black box, is misplaced, or hides a test from CI.

Three rules, all found broken on 2026-11-10:

- every tool appears in `tools/README.md` with its own description. 34 tools
  were named nowhere else in the repository, including the benches that answer
  open questions about latency and thermal drift;
- every `test_*.py` lives where the suite collects it (`test/`,
  `src/<pkg>/test/`). Three sat in `tools/` and never ran in CI; one of them,
  the calibration solver's only ground-truth check, had been broken since
  `fov_solve.py` became a shim, and nobody saw;
- every tool sits in the folder for who reaches for it and when (pool, bench,
  ci, site, studies), and only shared libraries sit at the top. 145 tools
  were in one flat folder, so the console an operator needs on a pool day was
  one line among calibration studies and site figure scripts.
"""
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

# A test outside the collected tree needs a reason here and the issue that
# brings it in. No reason, no exemption.
NOT_COLLECTED = {
    'tools/bench/control_bench/': (
        'compiles the board firmware for the host and needs a checkout of '
        'srot-control-board; 94 tests, passing locally. Bringing them into CI '
        'means fetching and building the firmware at a pinned commit (#132).'),
}


def _tracked():
    out = subprocess.run(['git', 'ls-files'], cwd=ROOT, capture_output=True,
                         text=True, check=True).stdout.split()
    return out


def test_the_tools_index_is_current():
    r = subprocess.run([sys.executable, 'tools/ci/gen_tools_index.py', '--check'],
                       cwd=ROOT, capture_output=True, text=True)
    assert r.returncode == 0, r.stdout + r.stderr


def test_every_tool_is_in_its_place():
    sys.path.insert(0, str(ROOT / 'tools' / 'ci'))
    try:
        import gen_tools_index as g
    finally:
        sys.path.pop(0)
    assert not g.misplaced(), (
        'tools/ holds only shared libraries at the top level; move these into '
        f'one of {sorted(g.FOLDERS)} (or, if a tool imports it, add it to '
        f'LIBRARIES): {g.misplaced()}')
    for lib in g.LIBRARIES:
        assert (ROOT / 'tools' / lib).is_file(), f'LIBRARIES names {lib}, which is gone'


def test_no_test_file_hides_outside_the_collected_tree():
    stray = [f for f in _tracked()
             if Path(f).name.startswith('test_') and f.endswith('.py')
             and not f.startswith('test/')
             and not (f.startswith('src/') and '/test/' in f)
             and not any(f.startswith(p) for p in NOT_COLLECTED)]
    assert not stray, (
        'test files CI never runs (it collects src/*/test and test/): '
        + ', '.join(stray))


def test_every_exemption_still_holds_a_test():
    files = _tracked()
    for prefix, reason in NOT_COLLECTED.items():
        assert reason, prefix
        assert any(f.startswith(prefix) and Path(f).name.startswith('test_')
                   for f in files), f'{prefix} holds no tests: drop the exemption'
