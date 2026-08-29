"""Single dispatch point: name -> YawSource instance.

Used by `auv_manager_node` and `sensors_node` at startup. Adding a new
source = one line in `BUILDERS` plus the source class itself; nothing
else changes.
"""

from .sources.mavlink_ahrs import MavlinkAhrsSource


def _build_mavlink_ahrs(*, pixhawk, **_):
    if pixhawk is None:
        raise ValueError("yaw_source='mavlink_ahrs' requires pixhawk=<Pixhawk>")
    return MavlinkAhrsSource(pixhawk)


def _build_bno085(*, port, baud, logger=None, pixhawk=None, calibrate=None, **_):
    """Build a BNO085Source.

    Calibration policy:
      * `calibrate=None`  -> default: calibrate iff `pixhawk` is provided.
      * `calibrate=True`  -> require pixhawk; raise if missing.
      * `calibrate=False` -> raw mode (no Earth reference). Diag only.
    """
    from .sources.bno085 import BNO085Source   # lazy: keep pyserial optional

    do_calibrate = (pixhawk is not None) if calibrate is None else bool(calibrate)
    if do_calibrate and pixhawk is None:
        raise ValueError("bno085 calibrate=True requires pixhawk=<Pixhawk>")

    def _read_pixhawk_yaw():
        attitude = pixhawk.get_attitude()
        return None if attitude is None else attitude['yaw']

    return BNO085Source(
        port=port,
        baud=baud,
        logger=logger,
        reference_yaw_provider=_read_pixhawk_yaw if do_calibrate else None,
    )


def _build_nucleus_dvl(*, nucleus_dvl_host='192.168.2.201',
                       nucleus_dvl_port=9000,
                       nucleus_dvl_password='nortek',
                       logger=None, **_):
    """Nortek Nucleus 1000 DVL over TCP -- heading from AHRS + body position.

    Starts DISCONNECTED. Call `duburi.dvl_connect()` (or the `dvl_connect`
    Move.action verb) at pool-side to open TCP and begin streaming.
    """
    from .sources.nucleus_dvl import NucleusDVLSource
    return NucleusDVLSource(
        host=nucleus_dvl_host,
        port=int(nucleus_dvl_port),
        password=nucleus_dvl_password,
        logger=logger,
    )


def _build_witmotion_stub(**_):
    """Future WitMotion HWT905 / WT901C serial yaw. Same fail-loud
    contract as the DVL stub."""
    from .sources.witmotion_stub import WitMotionSource
    return WitMotionSource()


def _build_bno085_dvl(*, port, baud, logger=None, pixhawk=None,
                      nucleus_dvl_host='192.168.2.201',
                      nucleus_dvl_port=9000,
                      nucleus_dvl_password='nortek', **_):
    """BNO085 heading + Nucleus DVL position.

    Heading (for yaw turns and heading lock) comes from the BNO085 IMU.
    Position (for DVL distance commands) comes from the Nucleus DVL.
    DVL starts DISCONNECTED; call dvl_connect (or use dvl_auto_connect).
    """
    from .sources.bno085           import BNO085Source
    from .sources.nucleus_dvl      import NucleusDVLSource
    from .sources.composite_bno_dvl import CompositeBnoDvlSource

    do_calibrate = pixhawk is not None

    def _read_pixhawk_yaw():
        if pixhawk is None:
            return None
        attitude = pixhawk.get_attitude()
        return None if attitude is None else attitude['yaw']

    bno = BNO085Source(
        port=port,
        baud=baud,
        logger=logger,
        reference_yaw_provider=_read_pixhawk_yaw if do_calibrate else None,
    )
    dvl = NucleusDVLSource(
        host=nucleus_dvl_host,
        port=int(nucleus_dvl_port),
        password=nucleus_dvl_password,
        logger=logger,
    )
    return CompositeBnoDvlSource(bno, dvl, logger=logger)


def _build_sim_dvl(*, pixhawk, logger=None, sim_dvl_topic='/dvl/velocity', **_):
    """Gazebo DVL position + MAVLink AHRS heading, for the simulator.

    The DVL is position-only, so it must be paired with a heading source or
    every yaw verb breaks. `stack.launch.py` otherwise pins the sim to
    `mavlink_ahrs`; this keeps that heading and only adds bottom-track position,
    which is what turns the `*_dist` verbs from dead reckoning into a real
    closed loop in sim.

    Connects eagerly: unlike the Nucleus (a TCP link that may not be plugged
    in), the gz subscription is local and cheap, and a mission that forgets
    `dvl_connect` would otherwise silently have no position.
    """
    if pixhawk is None:
        raise ValueError("yaw_source='sim_dvl' requires pixhawk=<Pixhawk> for heading")
    from .sources.sim_dvl import SimDvlSource
    from .sources.composite_bno_dvl import CompositeBnoDvlSource

    ahrs = MavlinkAhrsSource(pixhawk)
    dvl = SimDvlSource(topic=sim_dvl_topic, logger=logger)
    dvl.connect()
    return CompositeBnoDvlSource(ahrs, dvl, logger=logger, name='sim_dvl')


def _build_bno085_sim_dvl(*, port, baud, logger=None, pixhawk=None,
                          sim_dvl_topic='/dvl/velocity', **_):
    """BNO085 heading + Gazebo DVL position -- the SIM twin of `bno085_dvl`.

    The pool configuration is `bno085_dvl`: BNO for heading, Nucleus for
    position. In simulation the Nucleus is not reachable, so that source comes
    up with heading only and every `*_dist` verb is dead -- which left the
    pool's actual configuration with no sim equivalent at all. This is the same
    composite with the Gazebo DVL in the Nucleus's place, so a mission can be
    rehearsed on the sensor pair it will fly.

    Requires the virtual BNO board (`bno085_sim`), or an explicit `port` to
    real hardware.
    """
    from .sources.bno085 import BNO085Source
    from .sources.sim_dvl import SimDvlSource
    from .sources.composite_bno_dvl import CompositeBnoDvlSource

    def _read_pixhawk_yaw():
        if pixhawk is None:
            return None
        attitude = pixhawk.get_attitude()
        return None if attitude is None else attitude['yaw']

    bno = BNO085Source(
        port=port,
        baud=baud,
        logger=logger,
        reference_yaw_provider=_read_pixhawk_yaw if pixhawk is not None else None,
    )
    dvl = SimDvlSource(topic=sim_dvl_topic, logger=logger)
    # Eager, like _build_sim_dvl: the gz subscription is local and cheap, and a
    # mission that forgets dvl_connect would silently have no position.
    dvl.connect()
    return CompositeBnoDvlSource(bno, dvl, logger=logger, name='bno085_sim_dvl')


# Registered sources. witmotion is wired up as a fail-loud stub so users
# hitting `yaw_source='witmotion'` get a clear "not implemented" message
# instead of a generic "unknown source" error from make_yaw_source.
BUILDERS = {
    'mavlink_ahrs': _build_mavlink_ahrs,
    'bno085':       _build_bno085,
    'dvl':          _build_nucleus_dvl,
    'nucleus_dvl':  _build_nucleus_dvl,
    'bno085_dvl':   _build_bno085_dvl,   # BNO heading + DVL position
    # Sim twin of bno085_dvl: BNO heading + Gazebo DVL position.
    'bno085_sim_dvl': _build_bno085_sim_dvl,
    'dvl_bno':      _build_bno085_dvl,   # alias
    'sim_dvl':      _build_sim_dvl,     # Gazebo DVL position + AHRS heading
    'witmotion':    _build_witmotion_stub,
}


def make_yaw_source(name, **kwargs):
    """Return a configured YawSource for the given name.

    Raises
    ------
    ValueError
        If `name` is unknown or required kwargs are missing.
    Exception
        Hardware-specific errors propagate (e.g. SerialException for
        bno085 with the wrong port). Caller is responsible for failing
        loudly at startup -- no silent fallback.
    """
    key = (name or '').strip().lower()
    if key not in BUILDERS:
        known = ', '.join(sorted(BUILDERS))
        raise ValueError(f"unknown yaw_source '{name}'. known: {known}")
    return BUILDERS[key](**kwargs)
