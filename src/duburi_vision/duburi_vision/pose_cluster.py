"""Moved to `duburi_localization` (2026-09-11). Re-exported so importers keep working.

None of this module was ever about pixels: it takes measurements in and
produces a pose, which is the estimation layer's job. It lives in
`duburi_localization.pose_cluster` now. This shim exists because the
alternative -- editing every importer in one commit -- is how a move becomes a
breakage, and `flow_math` set the same precedent when `RefractiveRectifier`
went to `optics`.
"""
from duburi_localization.pose_cluster import *          # noqa: F401,F403
from duburi_localization import pose_cluster as _moved

__all__ = getattr(_moved, '__all__', [n for n in dir(_moved)
                                      if not n.startswith('_')])
