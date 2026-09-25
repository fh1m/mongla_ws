"""One deck file for the ladder's switches, read by the node itself.

⛔ WHY NOT LAUNCH ARGUMENTS. A switch threaded through launch is only read by
the launch files that DECLARE it, and this package has shipped capabilities
reachable from one launch path and not the one `bringup` includes -- place
recognition was wired into `vision_pi.launch.py` while `bringup` included
`vision.launch.py`, the fifth instance of the same defect. Every new launch
argument is a fresh chance to make that mistake, and a second source of truth
for a value the node already has a default for.

⭐ `anchor/loop_closure.py` already solved this, and carries a guard test that
fails if launch plumbing comes back. This is that pattern generalised, so the
rest of the ladder's switches gain the same property: **the node behaves
identically whichever launch started it, and an operator changes one file on
the deck.**

    ~/.mongla/ladder.yaml
        act_conf: 0.60            # raise the acting bar for a murky venue
        anchor_semi_dense: true   # try the 2.1-2.6x inlier mode

⚠ EVERY FAILURE DEGRADES TO THE DEFAULTS *AND SAYS SO*. A malformed file must
not take the vision stack down on a pool deck -- but silently ignoring a file
the operator just edited is the worse failure, because they would then watch
for a behaviour change that was never loaded.

⚠ AN UNKNOWN KEY IS REPORTED, NOT IGNORED. A typo'd switch is indistinguishable
from a switch that does nothing, and the operator cannot tell which they are
looking at.
"""
from __future__ import annotations

import os
from typing import Any, Callable, Dict

CONFIG_PATH = '~/.mongla/ladder.yaml'


def _as_bool(v: Any) -> bool:
    """YAML gives real bools, but a hand-edited file often carries a string."""
    if isinstance(v, bool):
        return v
    return str(v).strip().lower() in ('1', 'true', 'yes', 'on')


# name -> (coercion, default). The default MUST equal the node's declared
# default: two copies of a default is how they drift, and a test compares them
# rather than trusting this comment.
_KEYS: Dict[str, tuple] = {
    # The B-59 acting bar. Below this a detection may be ASSOCIATED by the
    # tracker but is not ACTED ON by the ladder. Measured knee 0.60; 0.45
    # ships because 0.60 came from one model, one class and one venue.
    'act_conf': (float, 0.45),
    # B-62: XFeat on the Hailo-8 (10.89 ms) rather than the Pi CPU (32.9 ms).
    'anchor_xfeat_hef': (_as_bool, True),
    # Semi-dense matching: 2.1-2.6x inliers for +37 % match cost. OFF until
    # frame-to-reference behaviour and inlier correctness are measured.
    'anchor_semi_dense': (_as_bool, False),
    # Measured TRUE width of the target in metres; 0 = use the rulebook
    # nominal. A measured prop beats a nominal, which is what SAUVC's +/-5 %
    # tolerance exists to allow for.
    'target_width_m': (float, 0.0),
}


def defaults() -> Dict[str, Any]:
    return {k: d for k, (_c, d) in _KEYS.items()}


def load_config(path: str = CONFIG_PATH, log=None) -> Dict[str, Any]:
    """Deck settings merged over the defaults. Never raises."""
    out = defaults()
    full = os.path.expanduser(path)
    if not os.path.isfile(full):
        return out
    try:
        import yaml
        with open(full) as fh:
            raw = yaml.safe_load(fh) or {}
        if not isinstance(raw, dict):
            raise ValueError(f'expected a mapping, got {type(raw).__name__}')
        unknown = [k for k in raw if k not in _KEYS]
        for k, v in raw.items():
            if k in _KEYS:
                coerce: Callable = _KEYS[k][0]
                out[k] = coerce(v)
        if log is not None:
            named = ', '.join(f'{k}={out[k]}' for k in sorted(raw)
                              if k in _KEYS)
            if named:
                log.info(f'[LOCK ] ladder config from {full}: {named}')
            if unknown:
                log.warn(
                    f'[LOCK ] {full} has unknown key(s) {sorted(unknown)} -- '
                    f'ignored. A typo\'d switch looks exactly like a switch '
                    f'that does nothing. Known: {sorted(_KEYS)}')
    except Exception as exc:                                     # noqa: BLE001
        if log is not None:
            log.warn(f'[LOCK ] {full} could not be read '
                     f'({type(exc).__name__}: {exc}) -- using defaults. The '
                     f'ladder is running its SHIPPED behaviour, not what that '
                     f'file says.')
        return defaults()
    return out
