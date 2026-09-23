// A host stub for FreeRTOS, so the board's PURE control code can compile.
//
// ⚠ READ THIS BEFORE ASSUMING IT IS A CHEAT. `feedforward.cpp` contains no
// RTOS call of any kind -- it is arithmetic on six floats. It needs FreeRTOS
// only TRANSITIVELY: it includes `control/feedforward.h`, which includes
// `state_types.h` for the `FlightMode` enum, and `state_types.h` includes
// <freertos/FreeRTOS.h> and <freertos/semphr.h> because the SHARED-STATE
// STRUCTS further down that header are each guarded by a mutex.
//
// `FlightMode` is `enum class : uint8_t`. It needs none of that.
//
// So this stub satisfies an include the control code does not use, rather than
// standing in for behaviour it does. That is the line the shim's own header
// draws: providing a missing declaration is fine, reimplementing a control law
// is not. Nothing here is ever called -- if any of these symbols is invoked,
// the linked code is doing something the bench has no business modelling, and
// the right response is to stop rather than to flesh this out.
//
// ⭐ AND IT IS A FINDING WORTH SENDING UPSTREAM. A pure, side-effect-free
// control module cannot be compiled or unit-tested on a host because a types
// header drags in the RTOS. Splitting the enums out of `state_types.h` would
// make `mixer`, `attitude_control` and `feedforward` testable off-target with
// no stub at all.
#pragma once

#include <stdint.h>

typedef uint32_t TickType_t;
typedef int32_t BaseType_t;

#define pdTRUE  1
#define pdFALSE 0
#define portMAX_DELAY ((TickType_t)0xFFFFFFFFU)
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
