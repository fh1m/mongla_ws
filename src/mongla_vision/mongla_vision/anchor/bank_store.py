"""Saving the bank to disk and loading it back (a practice bank, or the last run).

Mixed into `CheckpointBank`.
"""
from __future__ import annotations
import json
import os
import numpy as np
from .anchor import Anchor


class StoreMixin:
    """Methods of `CheckpointBank`; see `bank.py` for the node."""

    def save(self, path: str) -> int:
        """Write the bank to a .npz. Returns the number of references saved."""
        if not self._refs:
            raise ValueError('refusing to save an empty bank')
        out = {'version': np.array([self.SAVE_VERSION]),
               'count': np.array([len(self._refs)]),
               'backend_wh': np.array([self._be.w, self._be.h]),
               'labels': np.array(self._labels, dtype=object),
               'yields': np.array(self._yields, np.int32),
               'att': np.array([(np.nan, np.nan, np.nan) if a is None else a
                                for a in self._att], np.float32),
               'depth': np.array(self._depth, np.float32),
               'pos': np.array([(np.nan, np.nan) if q is None else q
                                for q in self._pos], np.float32),
               # JSON per reference: a dict of named points is not an array,
               # and a ragged object array would not survive allow_pickle=False
               # for anyone who loads this more carefully than we do.
               'ann': np.array([json.dumps(a or {}) for a in self._ann],
                               dtype=object)}
        for i, a in enumerate(self._refs):
            out[f'k{i}'] = np.asarray(a._ref_kpts, np.float32)
            out[f'd{i}'] = np.asarray(a._ref_desc, np.float32)
            out[f's{i}'] = np.asarray(self._sigs[i], np.float32)
            roi = a.reference_roi
            out[f'r{i}'] = (np.array([np.nan] * 4, np.float32) if roi is None
                            else np.asarray(roi, np.float32))
        os.makedirs(os.path.dirname(os.path.abspath(path)) or '.', exist_ok=True)
        np.savez_compressed(path, **out)
        return len(self._refs)

    def load(self, path: str, *, append: bool = False) -> int:
        """Read a bank back. Returns how many references were added.

        Refuses a bank built at another resolution. Keypoints are stored in
        BACKEND pixels, so loading 640x480 references into a 320x240 backend
        would silently halve every coordinate -- no error, a plausible
        homography, and every pose wrong by a factor of two. That is the exact
        class of failure this codebase keeps finding, so it is a hard refusal
        rather than a rescale.
        """
        z = np.load(path, allow_pickle=True)
        ver = int(z['version'][0])
        if ver != self.SAVE_VERSION:
            raise ValueError(f'bank version {ver} != {self.SAVE_VERSION}')
        w, h = (int(v) for v in z['backend_wh'])
        if (w, h) != (self._be.w, self._be.h):
            raise ValueError(
                f'bank was built at {w}x{h}, backend is '
                f'{self._be.w}x{self._be.h} -- keypoints are in backend pixels, '
                f'so loading it would scale every pose silently')
        if not append:
            self.clear()
        labels = [str(v) for v in z['labels']]
        yields = [int(v) for v in z['yields']]
        n = int(z['count'][0])
        added = 0
        for i in range(n):
            if len(self._refs) >= self._cap:
                self._evict()
            a = Anchor(self._be, min_inliers=self._min_inliers,
                       min_cossim=self._min_cossim, k=self._k)
            a._ref_kpts = z[f'k{i}']
            a._ref_desc = z[f'd{i}']
            a._ref_shape = (self._be.h, self._be.w)
            roi = z[f'r{i}']
            a._ref_roi = None if not np.isfinite(roi).all() else tuple(
                float(v) for v in roi)
            # No stored image: a preloaded reference can still MATCH, it simply
            # cannot be displayed. Better than refusing to load, and better
            # than inventing a picture.
            a._ref_gray = None
            self._refs.append(a)
            self._yields.append(yields[i] if i < len(yields) else len(a._ref_kpts))
            self._sigs.append(z[f's{i}'])
            self._labels.append(labels[i] if i < len(labels) else '')
            att = z['att'][i] if 'att' in z.files and i < len(z['att']) else None
            self._att.append(None if att is None or not np.isfinite(att).all()
                             else tuple(float(v) for v in att))
            self._depth.append(float(z['depth'][i])
                               if 'depth' in z.files and i < len(z['depth'])
                               else float('nan'))
            q = z['pos'][i] if 'pos' in z.files and i < len(z['pos']) else None
            self._pos.append(None if q is None or not np.isfinite(q).all()
                             else (float(q[0]), float(q[1])))
            raw = (json.loads(str(z['ann'][i]))
                   if 'ann' in z.files and i < len(z['ann']) else {})
            # Already in BACKEND pixels when saved, so no rescale here -- and
            # `load` refuses a foreign resolution outright, which is what makes
            # that safe.
            self._ann.append({k: (float(v[0]), float(v[1]))
                              for k, v in raw.items()} or None)
            added += 1
        return added
