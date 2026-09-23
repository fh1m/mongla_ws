// =============================================================================
//  A host shim for Arduino.h, so the board's control code compiles natively.
//
//  ⛔ WHY THIS EXISTS. Until now a Hengla control change was testable NOWHERE
//  but the vehicle: `sim/` runs ArduSub SITL by design, so it exercises verbs and
//  host behaviour and never the board's own control laws. Every control question
//  therefore cost a pool session, and you do not out-tune a serious competitor at
//  one experiment per pool day.
//
//  It is possible because the firmware's control maths is PURE. Measured across
//  `mixer.cpp`, `attitude_control.cpp`, `feedforward.cpp`, `pid.h`: zero
//  `Serial`, `digitalWrite`, `Wire`, RTOS or `millis()` calls. Their only real
//  dependencies are their own headers, a struct of floats (`params.h`), a header
//  of defines (`config.h`, which includes nothing at all), and the seven symbols
//  below.
//
//  ⚠ THIS FILE IS NOT A REIMPLEMENTATION AND MUST NEVER BECOME ONE. The whole
//  value of the bench is that it runs THEIR code, unmodified, straight out of
//  the firmware checkout. The moment we copy a control law in here to "make it
//  build", the bench stops being evidence about the board. If a firmware file
//  needs something this shim cannot honestly provide, that is a finding to
//  report upstream -- not a thing to paper over here.
// =============================================================================
#pragma once

#include <math.h>
#include <stdint.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Arduino defines these; `attitude_control.cpp` wraps angles with PI.
#ifndef PI
#define PI 3.1415926535897932384626433832795f
#endif
#ifndef TWO_PI
#define TWO_PI 6.283185307179586476925286766559f
#endif
#ifndef DEG_TO_RAD
#define DEG_TO_RAD 0.017453292519943295769236907684886f
#endif
#ifndef RAD_TO_DEG
#define RAD_TO_DEG 57.295779513082320876798154814105f
#endif

// Arduino's constrain is a MACRO, and that matters: the firmware relies on it
// working for both float and int without overload resolution. A templated
// function here would change which type the expression produces in mixed
// arithmetic, which is exactly the kind of silent divergence this bench exists
// to rule out.
#ifndef constrain
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#endif

#ifndef min
#define min(a, b) ((a) < (b) ? (a) : (b))
#endif

#ifndef max
#define max(a, b) ((a) > (b) ? (a) : (b))
#endif

// ⚠ TIME IS SETTABLE, NOT WALL-CLOCK. `depth_control.cpp` calls millis() to
// stamp its last run and to age its output. Wiring that to the host clock would
// make every bench run depend on how fast the machine happens to be, and a
// freshness check would pass or fail by luck. The bench ADVANCES time
// deliberately instead, so a scenario is reproducible to the millisecond.
#ifdef __cplusplus
extern "C" {
#endif
extern unsigned long bench_millis_value;
static inline unsigned long millis(void) { return bench_millis_value; }
#ifdef __cplusplus
}
#endif

// `isfinite` comes from <math.h> as a macro in C++; nothing to add.
// `fabsf` and `sqrtf` likewise.
