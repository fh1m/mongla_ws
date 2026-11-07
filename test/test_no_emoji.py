"""No emoji in any file this repository owns.

Markers were the house style once, and with 3 500 of them nothing stood out.
The words replace them: **Warning:** / **Caution:** / **Key point:** in docs,
`WARNING:` / `ERROR:` / `ok` / `FAIL` in operator output (CLAUDE.md section 9).
"""
import re
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TEXT = re.compile(r'\.(py|md|html|ya?ml|sh|txt|js|css|json|toml|cfg'
                  r'|action|msg|srv|cpp|hpp|h|rules)$')
# Pictographs, dingbats, misc symbols and the emoji variation selector. Box
# drawing, arrows, maths and the degree sign are typography and stay.
EMOJI = re.compile(r'[\u231A\u231B\u23E9-\u23FA\u2600-\u27BF\u2B00-\u2BFF'
                   r'\U0001F000-\U0001FAFF\uFE0F]')


def _tracked():
    out = subprocess.run(['git', 'ls-files'], cwd=ROOT, capture_output=True, text=True).stdout
    return [ROOT / f for f in out.split() if TEXT.search(f)]


def test_no_tracked_file_holds_an_emoji():
    found = []
    for f in _tracked():
        try:
            text = f.read_text(encoding='utf-8')
        except (OSError, UnicodeDecodeError):
            continue
        for n, line in enumerate(text.splitlines(), 1):
            if EMOJI.search(line):
                found.append(f'{f.relative_to(ROOT)}:{n}: {line.strip()[:80]}')
    assert not found, 'emoji found -- use a word instead:\n' + '\n'.join(found[:30])
