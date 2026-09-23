// Host stub for FreeRTOS semaphores. See `FreeRTOS.h` in this directory for why
// this exists and why it is not a cheat: `state_types.h` guards its shared-state
// structs with mutexes, and the control code reaches that header only for the
// `FlightMode` enum.
//
// ⛔ THESE ARE DELIBERATELY NOT FUNCTIONAL. `xSemaphoreTake` always succeeds and
// takes nothing. That is safe here ONLY because the bench is single-threaded and
// the linked control modules never call it -- they are pure functions over their
// arguments. If a future bench links a module that really does take a lock, this
// stub would hide a race rather than model one, and the correct move is to stop
// and reconsider what is being linked, not to make these real.
#pragma once

#include <freertos/FreeRTOS.h>

typedef void* SemaphoreHandle_t;

#define xSemaphoreCreateMutex()        ((SemaphoreHandle_t)1)
#define xSemaphoreTake(handle, ticks)  (pdTRUE)
#define xSemaphoreGive(handle)         (pdTRUE)
