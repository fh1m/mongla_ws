"""Git LFS holds the vehicle's model weights and nothing else.

GitHub Pages serves an LFS file as its pointer text, so a site picture or the
hull put in LFS would render as 130 bytes of text on the live site. And a
.hef without its <stem>.yaml sidecar loads with an empty class allowlist and
returns [] every frame while looking healthy (CLAUDE.md, vision).
"""
import pathlib
import subprocess

ROOT = pathlib.Path(__file__).resolve().parents[1]
MODELS = 'src/mongla_vision/models/'


def _lfs_patterns(text: str) -> list:
    return [line.split()[0] for line in text.splitlines()
            if line.strip() and not line.lstrip().startswith('#') and 'filter=lfs' in line]


def _tracked() -> list:
    out = subprocess.run(['git', 'ls-files', '-z'], cwd=ROOT, capture_output=True, check=True)
    return [p for p in out.stdout.decode().split('\0') if p]


def _in_lfs(paths: list) -> set:
    if not paths:
        return set()
    out = subprocess.run(['git', 'check-attr', '-z', 'filter', '--stdin'], cwd=ROOT,
                         input='\0'.join(paths).encode(), capture_output=True, check=True)
    f = out.stdout.decode().split('\0')
    return {f[i] for i in range(0, len(f) - 2, 3) if f[i + 2] == 'lfs'}


def test_every_lfs_pattern_is_inside_the_models_folder():
    pats = _lfs_patterns((ROOT / '.gitattributes').read_text())
    assert pats, 'no LFS pattern: the weights are not versioned'
    assert all(p.startswith(MODELS) for p in pats), pats


def test_nothing_the_site_serves_is_in_lfs():
    docs = [p for p in _tracked() if p.startswith('docs/')]
    assert docs
    assert not _in_lfs(docs)


def test_every_tracked_hef_has_its_sidecar():
    tracked = set(_tracked())
    hefs = [p for p in tracked if p.startswith(MODELS) and p.endswith('.hef')]
    missing = [h for h in hefs if h[:-4] + '.yaml' not in tracked]
    assert not missing, missing


def test_the_patterns_are_checked_by_a_planted_docs_rule():
    """The scanner must see an LFS rule outside the folder: a planted one."""
    planted = 'docs/imgs/*.webp filter=lfs diff=lfs merge=lfs -text\n'
    assert not all(p.startswith(MODELS) for p in _lfs_patterns(planted))
