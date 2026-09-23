// =============================================================================
//  bench_api — a flat C ABI over the board's OWN control code.
//
//  Every function here is a thin forwarder. There is no control logic in this
//  file and there must never be any: the point of the bench is that the maths
//  executing is the firmware's, compiled from the firmware checkout, not a
//  Python or C++ retelling of it that can drift.
//
//  What is linked in (all unmodified, from srot-control-board):
//      control/mixer.cpp        the allocation, thrust curve, spin floor, DShot
//      control/thrust_trim.cpp  mixer.cpp calls thrust_trim::gain()
//
//  `g_params` is DEFINED here rather than linked from `comms/params.cpp`,
//  because that translation unit drags in MAVLink, NVS and the whole comms
//  stack. The struct itself is a bag of floats in a header, so defining it is
//  honest -- but it starts ZEROED, and a zeroed MOT_THST_EXPO is not the
//  firmware default. `bench_params_defaults()` writes the config.h defaults,
//  and Python calls it before anything else.
// =============================================================================
#include <Arduino.h>

#include "config.h"
#include "comms/params.h"
#include "control/mixer.h"

// The one symbol `comms/params.cpp` would have given us.
Params g_params;

extern "C" {

int bench_num_thrusters(void) { return NUM_THRUSTERS; }

// ---- parameters ---------------------------------------------------------- //

// The config.h defaults for everything the linked code reads. Explicit rather
// than a memset, because a silently-zero MOT_THST_EXPO would linearise the
// thrust curve and the bench would disagree with the board for a reason nobody
// could see.
//
// ⚠ DEF_MOT_BAT_V_MAX is 0.0, which means battery compensation is OFF by
// default in the firmware. That is not an oversight to "fix" here -- it is the
// shipped configuration, and it matches the board's measured behaviour.
void bench_params_defaults(void) {
    memset(&g_params, 0, sizeof(g_params));
    g_params.mot_thst_expo = DEF_MOT_THST_EXPO;
    g_params.mot_spin_min  = DEF_MOT_SPIN_MIN;
    g_params.mot_spin_arm  = DEF_MOT_SPIN_ARM;
    g_params.mot_bat_v_max = DEF_MOT_BAT_V_MAX;
    g_params.mot_bat_v_min = DEF_MOT_BAT_V_MIN;
    g_params.thr_trim_en   = DEF_THR_TRIM_EN;
}

void  bench_set_thst_expo(float v) { g_params.mot_thst_expo = v; }
void  bench_set_spin_min(float v)  { g_params.mot_spin_min  = v; }
void  bench_set_spin_arm(float v)  { g_params.mot_spin_arm  = v; }
void  bench_set_bat_v_max(float v) { g_params.mot_bat_v_max = v; }
void  bench_set_bat_v_min(float v) { g_params.mot_bat_v_min = v; }
float bench_get_thst_expo(void)    { return g_params.mot_thst_expo; }
float bench_get_spin_min(void)     { return g_params.mot_spin_min; }

// ---- the mixer, verbatim ------------------------------------------------- //

void bench_mix(float roll, float pitch, float yaw,
               float throttle, float forward, float lateral,
               float* out) {
    mixer::mix(roll, pitch, yaw, throttle, forward, lateral, out);
}

void bench_to_dshot(const float* norm, const int8_t* dir, int armed,
                    int16_t* dshot) {
    mixer::toDshot(norm, dir, armed != 0, dshot);
}

int16_t bench_one_to_dshot(float norm, int8_t dir) {
    return mixer::oneToDshot(norm, dir);
}

void bench_set_battery(float volts, float dt_s) {
    mixer::setBatteryVoltage(volts, dt_s);
}

float bench_battery_scale(void) { return mixer::batteryScale(); }

void bench_motor_angular(int m, float* roll, float* pitch, float* yaw) {
    mixer::motorAngular(m, *roll, *pitch, *yaw);
}

}  // extern "C"
