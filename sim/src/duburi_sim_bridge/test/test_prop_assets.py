"""The props must carry the artwork and the surface detail the competition has.

Three things this pins, each of which shipped wrong at least once:

  * the task images come from VENDORED artwork, not from a font. The emoji font
    on this machine is a CBDT bitmap with a single 109 px strike, so every glyph
    was drawn at 109 px and resampled up -- soft by construction, and most of
    why the props read as cartoonish beside a photograph;
  * `droplet` is the DROP OF BLOOD, not the water droplet. Rendered once with
    Fluent's blue "Droplet" and it was obvious. A blue drop would also make the
    sim task EASIER than the pool task, since blue-vs-orange separates far more
    cleanly than magenta-vs-orange;
  * normal maps exist at all. Nothing in this tree used one until now, and a
    perfectly smooth surface with its detail painted on is both the clearest
    tell of a CG render and the worst case for feature matching.
"""
import os

import pytest

yaml = pytest.importorskip('yaml')
PIL = pytest.importorskip('PIL.Image')

ROOT = os.path.join(os.path.dirname(__file__), '..', '..', 'duburi_sim_worlds')
TEX = os.path.join(ROOT, 'models', 'robosub_textures')
EMOJI = os.path.join(TEX, 'emoji')

NEEDED = ('fire', 'droplet', 'ambulance', 'fire_engine',
          'compass', 'hammer_pick', 'ring_buoy', 'sos')


@pytest.mark.parametrize('name', NEEDED)
def test_task_artwork_is_vendored(name):
    path = os.path.join(EMOJI, f'{name}.png')
    assert os.path.isfile(path), (
        f'{name}: run scripts/fetch_emoji.py. Without it the generator falls '
        'back to a 109 px font strike.')
    img = PIL.open(path)
    assert img.mode == 'RGBA', f'{name} has no alpha; it would paste as a box'
    assert min(img.size) >= 128, f'{name} is {img.size}, too small to print'


def test_the_blood_drop_is_magenta_not_blue():
    """Fluent has both a blue `Droplet` and a magenta `Drop of blood`; the
    rulebook says :drop_of_blood:. Check the artwork, not the filename."""
    img = PIL.open(os.path.join(EMOJI, 'droplet.png')).convert('RGBA')
    px = [p for p in img.getdata() if p[3] > 200]
    assert px, 'droplet is fully transparent'
    r = sum(p[0] for p in px) / len(px)
    b = sum(p[2] for p in px) / len(px)
    assert r > b, f'droplet reads blue (mean R={r:.0f} B={b:.0f}) -- that is ' \
                  'the water Droplet, not the Drop of blood'


def test_emoji_licence_is_recorded():
    """MIT-licensed third-party artwork vendored into the repo needs its notice."""
    assert os.path.isfile(os.path.join(EMOJI, 'LICENSE'))


@pytest.mark.parametrize('nm', ('norm_pvc.png', 'norm_plastic.png',
                                'norm_fabric.png'))
def test_normal_maps_exist(nm):
    path = os.path.join(TEX, nm)
    if not os.path.isfile(path):
        pytest.skip('textures not generated')
    img = PIL.open(path).convert('RGB')
    # A flat surface encodes to (128,128,255). If every pixel is that, the map
    # carries no relief and the feature is a no-op that still costs a fetch.
    px = list(img.getdata())
    flat = sum(1 for p in px if abs(p[0] - 128) < 2 and abs(p[1] - 128) < 2)
    assert flat < 0.9 * len(px), f'{nm} is essentially flat'


def test_board_layout_pairs_images_with_openings():
    """Four openings and four image slots -- the CAD's two rows of four."""
    with open(os.path.join(ROOT, 'spec', 'robosub.yaml')) as fh:
        cfg = yaml.safe_load(fh)['props']['torpedo_board']
    assert len(cfg['cells']) == 4
    assert len(cfg['images']) == 4
    # no image slot may sit on top of an opening
    for img in cfg['images']:
        for cell in cfg['cells']:
            assert (img['y'], img['z']) != (cell['y'], cell['z'])
