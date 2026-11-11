"""The Lichtblick build Batighor serves: one pinned version, one hash."""
from __future__ import annotations

import json
from pathlib import Path

HOME = Path.home() / '.mongla' / 'batighor'


def share() -> Path:
    """The installed share directory, or the source tree when run from it."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory('mongla_batighor'))
    except Exception:                                   # noqa: BLE001
        return Path(__file__).resolve().parents[1]


def pin(root: Path | None = None) -> dict:
    return json.loads(((root or share()) / 'LICHTBLICK.json').read_text())


def web_dir(version: str, home: Path = HOME) -> Path:
    return home / 'web' / version
