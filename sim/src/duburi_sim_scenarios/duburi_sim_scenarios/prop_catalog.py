#!/usr/bin/env python3

"""Reach duburi_sim_worlds' prop library from this package.

The prop emitters live in duburi_sim_worlds, which is an ament_cmake package, so
its scripts are installed to share/ rather than onto the Python path. Importing
them here rather than duplicating the geometry is the point: a prop spawned at
runtime has to be identical to the same prop baked into a world, and the only way
to guarantee that is for both to come from one definition.
"""

import importlib.util
import os
import sys


def _scripts_dir():
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(
            get_package_share_directory('duburi_sim_worlds'), 'scripts'
        )
    except Exception:
        return None


def load_prop_library():
    """Import duburi_sim_worlds' prop_library, or return None if unavailable."""
    scripts = _scripts_dir()
    if not scripts:
        return None

    module_path = os.path.join(scripts, 'prop_library.py')
    if not os.path.exists(module_path):
        return None

    # prop_library reads spec/arena.yaml relative to its own location, so the
    # directory has to be importable as well as readable.
    if scripts not in sys.path:
        sys.path.insert(0, scripts)

    spec = importlib.util.spec_from_file_location('prop_library', module_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def prop_names(library):
    if library is None:
        return []
    return sorted(library.PROPS)


def render_prop(library, name):
    """Return complete SDF for a registered prop, or None if it is not one."""
    if library is None or name not in library.PROPS:
        return None
    return library.standalone_sdf(name, library.load_spec())


def prop_anchor(library, name):
    """Where the prop's origin sits: 'floor' or 'surface'."""
    if library is None or name not in library.PROPS:
        return None
    return library.PROPS[name]['anchor']


def pool_depth(library):
    if library is None:
        return None
    return library.load_spec()['pool']['depth']
