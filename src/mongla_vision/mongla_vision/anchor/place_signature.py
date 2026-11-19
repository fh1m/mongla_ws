"""The bank's place signature: VLAD over the descriptors it already holds.

The checkpoint bank ranks its references by a cheap signature and matches only
the top few (`bank.py`). The signature used to be the L2-normalised MEAN of a
reference's 1024 XFeat descriptors, 64 numbers. VLAD keeps what a mean throws
away: each descriptor's residual to its nearest of K = 16 visual words, summed
per word, normalised per word, square-rooted and normalised again (Jegou et
al., 2010). K x 64 = 1024 numbers, from the descriptors the backend already
computed: no second network, one (1024 x 64) @ (64 x 16) product per call.

Measured on the archive clips with ground truth taken the expensive way
(`tools/studies/bank_shortlist_accuracy.py`, measured-bars section 20.2), the
vocabulary learnt on the OTHER clips only: Recall@1 forward 52.5 % -> 65.8 %
(+13.3 points, 95 % +5.8 to +20.8), downward 47.9 % -> 77.1 % (+29.2, 95 %
+14.6 to +43.8), and the inlier yield lost when the shortlist misses falls.

The words are `config/vlad16.npz`, written by that same study with
`--write-vocab` from every clip. A missing file raises: a bank that silently
fell back to another signature would rank by a different measure than the one
that was measured, and nothing downstream would say so.
"""
from __future__ import annotations

import os
import threading

import numpy as np

K_WORDS = 16
DIM = 64
SIG_DIM = K_WORDS * DIM

_VOCAB_FILE = 'vlad16.npz'
_lock = threading.Lock()
_words: np.ndarray | None = None


def vlad(desc: np.ndarray | None, words: np.ndarray) -> np.ndarray:
    """Residuals to the nearest word, summed per word; intra-, power- and
    L2-normalised. Zeros when there are no descriptors."""
    v = np.zeros_like(words)
    if desc is not None and len(desc):
        d = np.asarray(desc, np.float32)
        a = np.argmax(d @ words.T - 0.5 * (words * words).sum(1), axis=1)
        np.add.at(v, a, d - words[a])
    n = np.linalg.norm(v, axis=1, keepdims=True)
    v = v / np.where(n > 0, n, 1.0)
    v = np.sign(v) * np.sqrt(np.abs(v))
    v = v.ravel()
    n = float(np.linalg.norm(v))
    return (v / n) if n > 0.0 else v


def _find() -> str:
    here = os.path.dirname(os.path.abspath(__file__))
    cands = [os.path.join(here, '..', '..', 'config', _VOCAB_FILE)]
    try:
        from ament_index_python.packages import get_package_share_directory
        cands.append(os.path.join(get_package_share_directory('mongla_vision'),
                                  'config', _VOCAB_FILE))
    except Exception:                                       # noqa: BLE001
        pass
    for c in cands:
        if os.path.isfile(c):
            return os.path.abspath(c)
    raise FileNotFoundError(
        f'{_VOCAB_FILE} not found (looked in {[os.path.abspath(c) for c in cands]}); '
        f'write it with tools/studies/bank_shortlist_accuracy.py --write-vocab')


def load_vocabulary(path: str | None = None) -> np.ndarray:
    """The K x 64 visual words, read once and shared. Refuses a wrong shape."""
    global _words
    if path is None and _words is not None:
        return _words
    with _lock:
        if path is None and _words is not None:
            return _words
        with np.load(path or _find(), allow_pickle=False) as z:
            w = np.asarray(z['words'], np.float32)
        if w.shape != (K_WORDS, DIM):
            raise ValueError(f'vocabulary is {w.shape}, expected {(K_WORDS, DIM)}')
        if path is None:
            _words = w
        return w


def signature(desc: np.ndarray | None) -> np.ndarray:
    """The 1024-D unit signature of one reference's (or one query's) descriptors."""
    return vlad(desc, load_vocabulary())
