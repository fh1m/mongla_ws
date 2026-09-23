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
#include "control/attitude_control.h"
#include "control/feedforward.h"
#include "control/depth_control.h"

// The one symbol `comms/params.cpp` would have given us.
Params g_params;

// The bench's clock. See the note in shim/Arduino.h.
extern "C" unsigned long bench_millis_value = 0;

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
    // Attitude cascade: outer angle P, inner rate PID.
    g_params.ang_rll_p = DEF_ANG_RLL_P;
    g_params.ang_pit_p = DEF_ANG_PIT_P;
    g_params.ang_yaw_p = DEF_ANG_YAW_P;
    g_params.rat_rll_p = DEF_RAT_RLL_P; g_params.rat_rll_i = DEF_RAT_RLL_I;
    g_params.rat_rll_d = DEF_RAT_RLL_D; g_params.rat_rll_ff = DEF_RAT_RLL_FF;
    g_params.rat_pit_p = DEF_RAT_PIT_P; g_params.rat_pit_i = DEF_RAT_PIT_I;
    g_params.rat_pit_d = DEF_RAT_PIT_D; g_params.rat_pit_ff = DEF_RAT_PIT_FF;
    g_params.rat_yaw_p = DEF_RAT_YAW_P; g_params.rat_yaw_i = DEF_RAT_YAW_I;
    g_params.rat_yaw_d = DEF_RAT_YAW_D; g_params.rat_yaw_ff = DEF_RAT_YAW_FF;
    g_params.pilot_expo = DEF_PILOT_EXPO;
    g_params.pilot_yaw_rate = DEF_PILOT_YAW_RATE;
    // ⚠ The five feedforward gains stay at ZERO, because that is what the
    // vehicle ships and what the live board reads back. Seeding them here would
    // make the bench quietly disagree with the hardware.
    g_params.atc_drag_rll = DEF_ATC_DRAG_RLL;
    g_params.atc_drag_pit = DEF_ATC_DRAG_PIT;
    g_params.atc_drag_yaw = DEF_ATC_DRAG_YAW;
    g_params.xc_yaw2rll = DEF_XC_YAW2RLL;
    g_params.trim_en = DEF_TRIM_EN;
    g_params.trim_leak = DEF_TRIM_LEAK;
    g_params.trim_max = DEF_TRIM_MAX;
}

void bench_set_millis(unsigned long ms) { bench_millis_value = ms; }

// ---- the attitude cascade, verbatim -------------------------------------- //

void bench_attitude_reset(void) { attitude::reset(); }
void bench_hold_yaw(float yaw_rad) { attitude::holdYaw(yaw_rad); }

void bench_stabilize(float s_roll, float s_pitch, float s_yaw,
                     float roll, float pitch, float yaw,
                     float gx, float gy, float gz, float dt,
                     float* out_roll, float* out_pitch, float* out_yaw) {
    attitude::stabilize(s_roll, s_pitch, s_yaw, roll, pitch, yaw,
                        gx, gy, gz, dt, *out_roll, *out_pitch, *out_yaw);
}

void bench_acro(float s_roll, float s_pitch, float s_yaw,
                float gx, float gy, float gz, float dt,
                float* out_roll, float* out_pitch, float* out_yaw) {
    attitude::acro(s_roll, s_pitch, s_yaw, gx, gy, gz, dt,
                   *out_roll, *out_pitch, *out_yaw);
}

float bench_rate_integral(int axis) { return attitude::rateIntegral(axis); }

// ---- the hydrodynamic feedforward ---------------------------------------- //

void bench_feedforward_reset(void) { feedforward::reset(); }

void bench_feedforward(float* roll, float* pitch, float* yaw, float* throttle,
                       float gx, float gy, float gz, int mode, int learn) {
    feedforward::apply(*roll, *pitch, *yaw, *throttle, gx, gy, gz,
                       (FlightMode)mode, learn != 0);
}

void  bench_set_drag(float rll, float pit, float yaw) {
    g_params.atc_drag_rll = rll;
    g_params.atc_drag_pit = pit;
    g_params.atc_drag_yaw = yaw;
}
void  bench_set_xc_yaw2rll(float v) { g_params.xc_yaw2rll = v; }
void  bench_set_trim(float en, float leak, float max_) {
    g_params.trim_en = en; g_params.trim_leak = leak; g_params.trim_max = max_;
}
float bench_get_drag_yaw(void) { return g_params.atc_drag_yaw; }

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
