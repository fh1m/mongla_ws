"""The site has two reds, and the figures use the same two.

`#ff0000` is the CAD fairing colour: structure, never text. `--red-lit` is the
one red for text and chart series. Until 2026-11-06 there were three: the site
used `#ff5747`, the banners `#ff2a14`, the diagrams a copy of the site's. The
site's one sat at OKLCH L 0.682, outside the dark chart band (0.48-0.67), so
every figure that used it as a series failed the palette check. `#ff2a14` is
inside the band (L 0.641) and 5.6:1 on black, so it is the lit red now.

Read as text: the files are the artefact under test.
"""
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def _css_red_lit() -> str:
    css = (ROOT / 'docs/assets/site.css').read_text()
    return re.search(r'--red-lit:\s*(#[0-9a-fA-F]{6})', css).group(1).lower()


def test_figures_use_the_sites_lit_red():
    lit = _css_red_lit()
    diagrams = (ROOT / 'tools/make_diagrams.py').read_text()
    banners = (ROOT / 'tools/make_banner.py').read_text() + \
        (ROOT / 'tools/section_banners.py').read_text()
    assert re.search(rf"RED_L = '#ff0000', '{lit}'", diagrams), 'make_diagrams RED_L != --red-lit'
    assert lit in banners.lower(), 'the banners draw a red the site does not use'


def test_the_retired_red_is_gone():
    hits = [str(p.relative_to(ROOT)) for d in ('docs', 'tools') for p in (ROOT / d).rglob('*')
            if p.is_file() and p.suffix in {'.css', '.html', '.svg', '.py', '.md'}
            and '#ff5747' in p.read_text(errors='ignore').lower()]
    assert not hits, f'#ff5747 sits outside the dark chart band; use --red-lit: {hits}'
