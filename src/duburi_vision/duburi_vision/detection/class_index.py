"""class_index -- load a {id: name} mapping from a YAML file alongside a model.

The YAML lives at ``<model_stem>.yaml`` in the same directory as the weights.
Format mirrors Ultralytics data.yaml::

    names:
      0: person
      1: bicycle
      2: car
      ...

Returns None when no YAML exists (model carries its own names table).
"""

from __future__ import annotations

from pathlib import Path


def load_class_index(model_path: str) -> dict[int, str] | None:
    yaml_path = Path(model_path).with_suffix('.yaml')
    if not yaml_path.exists():
        return None
    try:
        import yaml
        data = yaml.safe_load(yaml_path.read_text())
        names = data.get('names', {}) if isinstance(data, dict) else {}
        return {int(k): str(v) for k, v in names.items()}
    except Exception:
        return None
