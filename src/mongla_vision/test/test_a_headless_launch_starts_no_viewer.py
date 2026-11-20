"""With no screen, no vision launch starts the viewer by default.

`vision_dual` wires the viewer's exit to shut the whole launch (on a desk, `q`
quits everything). On Kutum, which has no screen, the viewer aborted and took
cameras, detectors and trackers down 5.5 s after start, measured 2026-11-20:
every bench run of recorded footage there ended before a frame was detected.
"""
import importlib.util
import pathlib

import pytest

from mongla_runtime import launch_defaults

_LAUNCH = pathlib.Path(__file__).resolve().parents[1] / 'launch'


def _viewer_default(name, monkeypatch, env):
    for k in ('DISPLAY', 'WAYLAND_DISPLAY'):
        monkeypatch.delenv(k, raising=False)
    for k, v in env.items():
        monkeypatch.setenv(k, v)
    monkeypatch.setenv('MONGLA_BRINGUP', '/nonexistent/bringup.yaml')
    spec = importlib.util.spec_from_file_location(name, _LAUNCH / f'{name}.launch.py')
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    for e in mod.generate_launch_description().entities:
        if getattr(e, 'name', None) == 'viewer':
            return ''.join(str(getattr(p, 'text', p)) for p in e.default_value)
    raise AssertionError(f'{name}: no viewer argument')


@pytest.mark.parametrize('name', ['vision', 'vision_dual'])
def test_no_screen_means_no_viewer(name, monkeypatch):
    assert _viewer_default(name, monkeypatch, {}) == 'false'


@pytest.mark.parametrize('name', ['vision', 'vision_dual'])
def test_a_screen_still_gets_the_viewer(name, monkeypatch):
    assert _viewer_default(name, monkeypatch, {'DISPLAY': ':0'}) == 'true'


def test_wayland_counts_as_a_screen():
    assert launch_defaults.has_screen({'WAYLAND_DISPLAY': 'wayland-0'}) == 'true'
    assert launch_defaults.has_screen({}) == 'false'
