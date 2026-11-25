"""localization_check -- is localization working, stage by stage, in water.

Each stage is a yes or no with its reason, printed as it happens and as a
table at the end; the first failing stage is the one to fix. Read only the
verdict lines: every number in them came from the vehicle.

    1  live       the localiser publishes (before arming)
    2  aided      horizontal velocity observed while hovering (ZUPT or flow)
    3  anchored   anchor_on(ANCHOR_PROP) puts the estimate in the pool frame
    4  out        go_to a point LEG_M ahead: the filter says it arrived
    5  back       go_to the start
    6  truth      a range-and-bearing fix off ANCHOR_PROP, an observation the
                  filter did not make, agrees with the filter's position
                  to within TRUTH_M

Stage 6 is the one that ranks the filter. Stages 4 and 5 are the filter
agreeing with itself, which is necessary and is not evidence: a drifting
estimate arrives at a drifted goal. The fix is a second, independent
observation.

Fill the tunables, then:

    ros2 run mongla_planner mission localization_check
"""

COURSE = None          # course name, the file stem under ~/mongla/deck/courses
ANCHOR_PROP = None     # a prop with a taped position and bearing, in view from the start
MODEL = None           # a detector model that emits ANCHOR_PROP's class

DEPTH_M = -0.5         # negative = below the surface
LEG_M = 1.5            # the out-and-back leg, along the anchored heading
TOLERANCE_M = 0.5      # go_to arrival, and the sigma it must be known to
TRUTH_M = 0.5          # filter vs independent fix, stage 6
GAIN = 25.0            # % thrust cap for go_to
AIDED_WAIT_S = 10.0    # how long stage 2 waits for aiding


def _stage(table, log, name, ok, why):
    table.append((name, bool(ok), why))
    log(f'localization_check: {name:<8} {"PASS" if ok else "FAIL"}  {why}')
    return bool(ok)


def run(mongla, log):
    import math
    import time
    missing = [n for n, v in (('COURSE', COURSE), ('ANCHOR_PROP', ANCHOR_PROP),
                              ('MODEL', MODEL)) if not v]
    if missing:
        log(f'localization_check: fill {", ".join(missing)} first -- nothing run')
        return
    table = []
    mongla.mission_reset()
    course = mongla.use_course(COURSE)
    mongla.set_model(MODEL)
    s = mongla.localization(timeout=3.0)
    if not _stage(table, log, 'live', s is not None,
                  'publishing' if s is not None
                  else 'no /mongla/localization/state (localization:=false?)'):
        return _report(table, log)
    mongla.arm()
    mongla.set_depth(DEPTH_M, timeout=30.0)
    try:
        deadline = time.monotonic() + AIDED_WAIT_S
        s = mongla.localization()
        while (s is None or not s.aiding.startswith('aided')) and time.monotonic() < deadline:
            mongla.pause(0.5)
            s = mongla.localization()
        if not _stage(table, log, 'aided', s is not None and s.aiding.startswith('aided'),
                      f'aiding={s.aiding if s else "?"}, flow {s.flow_fused if s else 0} '
                      f'fused / {s.flow_refused if s else 0} refused'):
            return
        anchor = mongla.anchor_on(ANCHOR_PROP)
        s = mongla.localization()
        if not _stage(table, log, 'anchored',
                      anchor.ok and s is not None and s.frame == 'pool',
                      anchor.reason if not anchor.ok
                      else f'frame={s.frame if s else "?"}'):
            return
        x0, y0, psi = s.x_m, s.y_m, math.radians(s.yaw_deg)
        gx, gy = x0 + LEG_M * math.cos(psi), y0 + LEG_M * math.sin(psi)
        r = mongla.go_to(gx, gy, tolerance=TOLERANCE_M, gain=GAIN)
        _stage(table, log, 'out', r.success, r.message)
        r = mongla.go_to(x0, y0, tolerance=TOLERANCE_M, gain=GAIN)
        _stage(table, log, 'back', r.success, r.message)
        cls = course.prop(ANCHOR_PROP).detect_class or ANCHOR_PROP
        mongla.vision.align(cls, yaw=0, duration=15.0)
        est = mongla.localization()           # the filter, before the fix reaches it
        fix = mongla.fix_from_prop(ANCHOR_PROP)
        if not fix.ok or est is None:
            _stage(table, log, 'truth', False,
                   fix.reason if not fix.ok else 'no estimate to compare')
        else:
            err = math.hypot(fix.x_m - est.x_m, fix.y_m - est.y_m)
            _stage(table, log, 'truth', err <= TRUTH_M,
                   f'filter ({est.x_m:+.2f}, {est.y_m:+.2f}) vs fix '
                   f'({fix.x_m:+.2f}, {fix.y_m:+.2f}): {err:.2f} m apart, '
                   f'filter sigma {est.sigma_xy_m:.2f} m')
    finally:
        _report(table, log)
        # Disarm even if surface raises: a hull left armed after a
        # failed surface is the worse outcome (bead 2s8).
        try:
            mongla.surface()
        finally:
            mongla.disarm()


def _report(table, log):
    log('localization_check: ---- verdict ----')
    for name, ok, why in table:
        log(f'localization_check: {name:<8} {"PASS" if ok else "FAIL"}  {why}')
    passed = sum(ok for _, ok, _ in table)
    log(f'localization_check: {passed} of {len(table)} stages passed')
