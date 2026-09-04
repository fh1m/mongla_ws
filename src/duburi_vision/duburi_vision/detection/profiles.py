"""ONE WORD instead of five knobs, because competition day is not the time.

THE PROBLEM THIS SOLVES. The vision path now has four independent settings
that interact -- `conf`, `preprocess`, `preprocess_clip`, `range_crop` --
and each was added because a measurement justified it. Individually correct;
collectively a configuration puzzle to solve while a run clock is going.

    ros2 launch ... conf:=0.10 preprocess:=clahe preprocess_clip:=3.0 \
                    range_crop:=true

Nobody gets that right under pressure, and a wrong combination is worse than
the default: CLAHE on clear water cost 64 points of recall, and a fixed crop
costs 31 points on a close prop.

So: `vision:=<profile>`. One word, from what the water looks like.

    murky      green, low visibility -- CLAHE on, crop on, conf 0.10
    clear      good visibility       -- no CLAHE, crop on, conf 0.10
    close      manipulation, docking -- nothing on, conf 0.15
    fast       the pipeline at full rate, everything off

EVERY VALUE HERE IS MEASURED, and the measurement is named beside it. A
profile that bundles a guess is worse than four honest knobs, because it
hides the guess.

The knobs remain, and a profile only supplies DEFAULTS -- an explicit
`preprocess:=off` still wins. Poolside you want one word; while debugging you
want the knob, and neither should require the other.
"""
from __future__ import annotations

from typing import Dict

# name -> (settings, why). The `why` is printed at startup, so an operator
# reading the log can see what was chosen and on what evidence.
PROFILES: Dict[str, tuple] = {
    'murky': (
        dict(conf=0.10, preprocess='clahe', preprocess_clip=3.0,
             range_crop=True),
        'green/low-visibility water. CLAHE measured +42 points of target '
        'presence on blurry saturated footage (10.7 -> 56.2 %); conf 0.10 '
        'adds 8.5 points with jitter flat; the crop recovers 66 -> 100 % '
        'recall at 4x range.'),
    'clear': (
        dict(conf=0.10, preprocess='off', preprocess_clip=3.0,
             range_crop=True),
        'good visibility. CLAHE is OFF because it COST 64 points of recall '
        'on sharp desaturated footage -- it is a contrast fix, and clear '
        'water has contrast. Crop and conf still help at range.'),
    'close': (
        dict(conf=0.15, preprocess='off', preprocess_clip=3.0,
             range_crop=False),
        'close work -- alignment, docking, firing. The crop is OFF because '
        'it LOSES 31 points on a target that fills the frame, and conf is '
        'back to 0.15 because a false positive matters more than a missed '
        'distant one when the prop is right there.'),
    'fast': (
        dict(conf=0.15, preprocess='off', preprocess_clip=3.0,
             range_crop=False),
        'maximum pipeline rate. CLAHE costs 79 -> 50 Hz on the Pi; this is '
        'the arm to use when the loop rate is the binding constraint and '
        'the target is easy.'),
}

DEFAULT = 'fast'


def resolve(name: str) -> tuple:
    """(settings, why) for a profile name. Unknown names RAISE.

    Never silently fall back to a default: a mistyped profile that quietly
    runs `fast` in murky water is a mission lost to a typo, and this repo has
    shipped the silent-fallback failure five times already.
    """
    key = (name or '').strip().lower()
    if not key:
        key = DEFAULT
    if key not in PROFILES:
        raise ValueError(
            f'unknown vision profile {name!r}. Choose one of '
            f'{sorted(PROFILES)} -- a wrong profile is recoverable, a '
            f'silently-ignored one is not.')
    settings, why = PROFILES[key]
    return dict(settings), why
