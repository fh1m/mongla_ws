"""Dependabot never proposes what the vehicle's numpy cannot run.

While kutum/constraints.txt keeps numpy below 2 (cv_bridge and the system cv2
are built against 1.x), scipy must stay below 1.18, the first release that
requires numpy>=2.0, and no major version arrives as a weekly PR. Dependabot
opened exactly that on 2026-11-09, and the checks passed, because CI does not
run the vision nodes against the vehicle's cv2.
"""
from __future__ import annotations

import re
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]


def numpy_one():
    return bool(re.search(r'^numpy\s*<\s*2\b', (ROOT / 'kutum/constraints.txt').read_text(), re.M))


def pip_updates():
    doc = yaml.safe_load((ROOT / '.github/dependabot.yml').read_text())
    return [u for u in doc['updates'] if u['package-ecosystem'] == 'pip']


def test_scipy_pin_needs_no_numpy_two():
    pin = re.search(r'^scipy==(\d+)\.(\d+)', (ROOT / 'kutum/requirements.txt').read_text(), re.M)
    assert pin, 'kutum/requirements.txt no longer pins scipy; update this test'
    if numpy_one():
        assert (int(pin[1]), int(pin[2])) < (1, 18)


def test_every_pip_update_holds_the_numpy_one_ceiling():
    if not numpy_one():
        return
    ups = pip_updates()
    assert {u['directory'] for u in ups} == {'/', '/kutum'}
    for u in ups:
        ign = u.get('ignore', [])
        names = {i['dependency-name'] for i in ign if set(i) == {'dependency-name'}}
        assert 'numpy' in names, u['directory']
        assert any(i['dependency-name'] == 'scipy' and i.get('versions') == ['>=1.18']
                   for i in ign), u['directory']
        assert any(i['dependency-name'] == '*'
                   and i.get('update-types') == ['version-update:semver-major']
                   for i in ign), u['directory']


def test_each_requirements_file_has_exactly_one_scanner():
    """The root `pip` entry scans subdirectories: without an exclusion it found
    kutum/requirements.txt as well, and every vehicle bump arrived twice
    (#115, #116) -- two PRs to verify on Kutum for one change."""
    doc = yaml.safe_load((ROOT / '.github/dependabot.yml').read_text())
    pips = [u for u in doc['updates'] if u['package-ecosystem'] == 'pip']
    root = next(u for u in pips if u['directory'] == '/')
    assert any(u['directory'] == '/kutum' for u in pips)
    assert 'kutum/**' in root.get('exclude-paths', [])


def test_pytest_stays_where_ros_launch_testing_loads():
    """requirements.txt caps pytest below 9 and Dependabot must not lift it.

    Measured 2026-11-20 on Humble: under pytest 9.2.0, launch_testing 1.0.14
    fails to load (`pytest_pycollect_makemodule(path, parent)`: the `path`
    argument is gone), so no ROS test runs. Dependabot's semver-major rule
    did not stop #178, which widened the range to <10.
    """
    cap = re.search(r'^pytest\b.*<\s*(\d+)', (ROOT / 'requirements.txt').read_text(), re.M)
    assert cap and int(cap.group(1)) <= 9, 'pytest is no longer capped below 9'
    root = [u for u in pip_updates() if u['directory'] == '/'][0]
    assert {'dependency-name': 'pytest', 'versions': ['>=9']} in root.get('ignore', []), \
        'dependabot.yml must ignore pytest>=9 while requirements.txt caps it'
