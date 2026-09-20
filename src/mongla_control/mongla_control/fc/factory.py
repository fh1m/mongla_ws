"""Single dispatch point: name -> FlightController instance.

Mirrors `mongla_sensors.factory.make_yaw_source`: a `BUILDERS` dict + a fail-loud
`make_flight_controller(name, ...)`. Adding a backend = one line + the class.

Backends are imported lazily inside their builders so that importing this module
(or `srot_protocol`) does not pull `pymavlink` until a backend is actually built.
"""

from __future__ import annotations


def _build_pixhawk(*, master, log=None, **_):
    from .pixhawk_fc import PixhawkFC
    return PixhawkFC(master, log=log)


def _build_srot(*, master, log=None, **_):
    from .srot_fc import SrotFC
    return SrotFC(master, log=log)


BUILDERS = {
    'pixhawk': _build_pixhawk,
    'srot':    _build_srot,
}


def make_flight_controller(name, *, master, log=None, **kwargs):
    """Return a configured FlightController for `name` ('pixhawk' | 'srot').

    `master` is a pre-opened mavutil connection (built in the manager /
    connection_config), exactly as `Pixhawk` has always been constructed.

    Raises ValueError on an unknown name -- no silent fallback, so a typo in the
    `flight_controller` param fails at startup rather than flying the wrong stack.
    """
    key = (name or '').strip().lower()
    if key not in BUILDERS:
        known = ', '.join(sorted(BUILDERS))
        raise ValueError(f"unknown flight_controller '{name}'. known: {known}")
    return BUILDERS[key](master=master, log=log, **kwargs)
