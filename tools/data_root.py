"""Where training artifacts go, and why it is never `/tmp`.

⛔ `/tmp` ON THIS BOX IS A RAM DISK. `df -h /tmp` reports **tmpfs, 7.8 G**, so
everything written there competes with the training process for the same
memory and none of it survives a reboot. It has cost this project twice:

  * two training runs were silently OOM-killed with the dataset staged there,
    which reads as a crash in the loader rather than as a full disk;
  * Round 7's entire XFeat fine-tune -- the dataset, the inventory and every
    checkpoint -- was written to `/tmp/xfeat_ourwater` and
    `/tmp/footage_inventory.json` and was **gone after the next reboot**,
    along with the ability to score the run through the shipped ONNX path.

The second is the expensive kind of mistake, because nothing failed. The run
completed, the numbers were printed, and the artifacts evaporated hours later
with no error anywhere.

⚠ A MEMORY NOTE IS NOT A FIX. "use mongla_data/" was already written down and
the tools still defaulted to `/tmp`, so the default kept winning. The default
is the fix; this module is that default, in one place, so the next tool that
needs a data root cannot quietly reintroduce a sixth copy.

Override with `$MONGLA_DATA` when a run belongs somewhere else.
"""
from __future__ import annotations

import os
import pathlib

ENV_VAR = 'MONGLA_DATA'

# The workspace's persistent data directory, beside `Ros_workspaces/`.
_DEFAULT = (pathlib.Path(__file__).resolve().parents[3] / 'mongla_data')


def _is_ram_disk(path: pathlib.Path) -> bool:
    """True when `path`'s nearest existing parent is tmpfs/ramfs.

    Checks a PARENT because the directory usually does not exist yet, and
    asking `df` about a path that has not been created returns nothing useful.
    """
    try:
        import subprocess
        probe = path
        while not probe.exists() and probe != probe.parent:
            probe = probe.parent
        out = subprocess.run(['df', '--output=fstype', str(probe)],
                             capture_output=True, text=True, timeout=5).stdout
        return any(line.strip() in ('tmpfs', 'ramfs')
                   for line in out.splitlines()[1:])
    except Exception:                                            # noqa: BLE001
        # Best effort: a path we cannot classify is better used than refused.
        return False


def data_root(*parts: str, create: bool = True) -> str:
    """A writable, PERSISTENT directory for datasets and checkpoints.

    `data_root('xfeat', 'run7')` yields `<root>/xfeat/run7`. Refuses a root on
    a RAM disk rather than accepting one that will evaporate.
    """
    root = pathlib.Path(os.environ.get(ENV_VAR) or _DEFAULT)
    if _is_ram_disk(root):
        raise SystemExit(
            f'REFUSING: {root} is on a RAM disk. Training artifacts there '
            f'compete with the run for memory and do not survive a reboot -- '
            f'that is how Round 7 was lost. Set {ENV_VAR} to a real disk.')
    p = root.joinpath(*parts) if parts else root
    if create:
        p.mkdir(parents=True, exist_ok=True)
    return str(p)
