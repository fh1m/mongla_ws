"""survey_course -- put the props where the vehicle actually sees them.

A course file ships with every position unset or nominal, and the tape-measure
survey (`course_survey`) needs someone on the deck with a tape. This mission is
the other way: from one station, it turns to each prop in turn, centres it,
and places it from several sightings with `place_prop(commit=True)`. The
placement goes into the deck course file, so the next run dead-reckons to
where the prop stood today, not where a rulebook said it might.

It trusts the hull, so the hull must be trustworthy. Run it from a known
station, right after `anchor_on`, before the pose has had time to drift. A
surveyed (`measured: true`) prop is only ever checked, never overwritten, so
a run from a bad pose cannot rewrite a good survey -- it logs a disagreement
instead.

It never translates: arm, one depth, then turns and alignments in place.
Fill the four tunables, then:

    ros2 run mongla_planner mission survey_course
"""

COURSE = None          # course name, e.g. the file stem under ~/.mongla/courses
ANCHOR_PROP = None     # a prop with a measured bearing, visible from the station
MODEL = None           # a detector model that emits every surveyed prop's class
SURVEY_PROPS = ()      # prop names in the order to turn to them

DEPTH_M = -0.5         # negative = below the surface
SAMPLES = 8            # sightings per prop
ALIGN_S = 15.0         # seconds to centre each prop before sighting it


def run(mongla, log):
    missing = [n for n, v in (('COURSE', COURSE), ('ANCHOR_PROP', ANCHOR_PROP),
                              ('MODEL', MODEL)) if not v]
    if missing or not SURVEY_PROPS:
        log(f'survey_course: fill {", ".join(missing or ["SURVEY_PROPS"])} '
            f'first -- nothing to survey')
        return
    mongla.mission_reset()
    course = mongla.use_course(COURSE)
    mongla.set_model(MODEL)
    mongla.arm()
    mongla.set_depth(DEPTH_M, timeout=30.0)
    try:
        mongla.anchor_on(ANCHOR_PROP)
        placed = []
        for name in SURVEY_PROPS:
            cls = course.prop(name).detect_class or name
            seen = mongla.vision.align(cls, yaw=0, duration=ALIGN_S)
            if not seen.saw_target:
                log(f'survey_course: {name} not seen -- skipped')
                continue
            got = mongla.place_prop(name, samples=SAMPLES, commit=True)
            log(f'survey_course: {name}: '
                + (f'({got.x_m:+.2f}, {got.y_m:+.2f}) m, sigma {got.sigma_m:.2f} m'
                   if got.ok else got.reason))
            if got.ok:
                placed.append(name)
        log(f'survey_course: placed {len(placed)} of {len(SURVEY_PROPS)}')
    finally:
        mongla.surface()
        mongla.disarm()
