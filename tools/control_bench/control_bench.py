"""The board's own control code, running on this machine.

⛔ WHY THIS EXISTS. A Hengla control change was testable NOWHERE but the vehicle.
`sim/` runs ArduSub SITL by design, so it exercises verbs and host behaviour and
never the board's control laws. Every control question therefore cost a pool
session -- and you cannot run a months-long tuning campaign at one experiment per
pool day.

WHAT IS ACTUALLY EXECUTING. `mixer.cpp` and `thrust_trim.cpp`, compiled from the
firmware checkout by `build.sh`, unmodified, linked into a shared library. Not a
Python port. Not a description. The same source the board runs.

⚠ WHAT THIS BENCH DOES **NOT** COVER, stated first because the boundary is the
whole point:

  * `task_control_loop.cpp` is NOT linked -- it pulls in FreeRTOS. So the
    PILOT_EXPO stick shaping, which happens on the board before the mixer, is
    applied here in Python (`pilot_expo`). The firmware half of the chain is
    genuine; the one host-applied stage is named so nobody has to guess which
    is which.
  * It models CONTROL CODE, not water. There is no plant here. A result from
    this bench is never a statement about how the vehicle moves.
  * It is bound to one firmware revision, recorded in `.firmware_rev` and
    reported by `firmware_rev()`. A bench that cannot say which commit it
    represents is not evidence after the firmware moves.
"""
from __future__ import annotations

import ctypes
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
LIB = HERE / 'libcontrolbench.so'
NUM_THRUSTERS = 8

# ⛔ THIS HULL'S CONFIGURATION, read live off the board on 2026-09-22. The mixer
# alone does NOT reproduce the vehicle: two board-level facts sit around it, and
# omitting them made the first falsifier run miss by a mean of 712 counts of 999
# -- not a small error, a structurally different answer.
#
#   FRAME_REVERSE = 1   negates all six axis demands BEFORE the mixer. Shipped 1
#                       on this hull (firmware rev 7+). `CLAUDE.md` warns that
#                       the [-1]x8 motor directions "restored on 2026-08-06" are
#                       no longer the intended configuration precisely because
#                       they would cancel it.
#   MOTOR_DIRS          the product CAL_MDIR{n} x MOT_{n}_DIRECTION, applied per
#                       motor inside `toDshot`. Measured:
#                           CAL_MDIR       [ 1  1  1  1 -1 -1 -1 -1]
#                           MOT_DIRECTION  [-1  1  1  1 -1 -1 -1  1]
#                           product        [-1  1  1  1  1  1  1 -1]
#                       No display anywhere shows that product, which is why
#                       `SrotFC.motor_directions()` exists to read it live.
#
# Read them off the vehicle before trusting any bench result that depends on
# them: a params reset silently restores defaults and this file cannot tell.
FRAME_REVERSE = True
MOTOR_DIRS = (-1, 1, 1, 1, 1, 1, 1, -1)


def _load() -> ctypes.CDLL:
    if not LIB.exists():
        # Build on first use rather than failing with a missing-file error the
        # caller then has to interpret.
        subprocess.run([str(HERE / 'build.sh')], check=True,
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    lib = ctypes.CDLL(str(LIB))
    f32, f32p = ctypes.c_float, ctypes.POINTER(ctypes.c_float)
    lib.bench_num_thrusters.restype = ctypes.c_int
    lib.bench_params_defaults.restype = None
    lib.bench_mix.argtypes = [f32] * 6 + [f32p]
    lib.bench_mix.restype = None
    lib.bench_to_dshot.argtypes = [f32p, ctypes.POINTER(ctypes.c_int8),
                                   ctypes.c_int, ctypes.POINTER(ctypes.c_int16)]
    lib.bench_to_dshot.restype = None
    lib.bench_one_to_dshot.argtypes = [f32, ctypes.c_int8]
    lib.bench_one_to_dshot.restype = ctypes.c_int16
    lib.bench_set_battery.argtypes = [f32, f32]
    lib.bench_battery_scale.restype = f32
    for name in ('bench_set_thst_expo', 'bench_set_spin_min', 'bench_set_spin_arm',
                 'bench_set_bat_v_max', 'bench_set_bat_v_min'):
        getattr(lib, name).argtypes = [f32]
        getattr(lib, name).restype = None
    for name in ('bench_get_thst_expo', 'bench_get_spin_min'):
        getattr(lib, name).restype = f32
    lib.bench_motor_angular.argtypes = [ctypes.c_int, f32p, f32p, f32p]
    return lib


class Board:
    """The board's mixer and output stage, at your fingertips.

    Every method forwards into the compiled firmware. The parameters start at
    the firmware's own `config.h` defaults; override them exactly as you would
    over MAVLink, and read the live values off the vehicle before trusting a
    bench result that depends on one.
    """

    def __init__(self):
        self._lib = _load()
        self._lib.bench_params_defaults()
        assert self._lib.bench_num_thrusters() == NUM_THRUSTERS

    # ---- parameters ------------------------------------------------------ #

    def set_params(self, *, thst_expo=None, spin_min=None, spin_arm=None,
                   bat_v_max=None, bat_v_min=None) -> 'Board':
        if thst_expo is not None:
            self._lib.bench_set_thst_expo(thst_expo)
        if spin_min is not None:
            self._lib.bench_set_spin_min(spin_min)
        if spin_arm is not None:
            self._lib.bench_set_spin_arm(spin_arm)
        if bat_v_max is not None:
            self._lib.bench_set_bat_v_max(bat_v_max)
        if bat_v_min is not None:
            self._lib.bench_set_bat_v_min(bat_v_min)
        return self

    def defaults(self) -> 'Board':
        self._lib.bench_params_defaults()
        return self

    @property
    def thst_expo(self) -> float:
        return float(self._lib.bench_get_thst_expo())

    @property
    def spin_min(self) -> float:
        return float(self._lib.bench_get_spin_min())

    # ---- the mixer ------------------------------------------------------- #

    def mix(self, *, roll=0.0, pitch=0.0, yaw=0.0, throttle=0.0,
            forward=0.0, lateral=0.0) -> list[float]:
        """Six body demands -> eight normalised motor outputs. `mixer::mix`."""
        out = (ctypes.c_float * NUM_THRUSTERS)()
        self._lib.bench_mix(roll, pitch, yaw, throttle, forward, lateral, out)
        return list(out)

    def to_dshot(self, norm, dirs=None, armed=True) -> list[int]:
        """Normalised outputs -> DShot values. `mixer::toDshot`."""
        n = (ctypes.c_float * NUM_THRUSTERS)(*norm)
        d = (ctypes.c_int8 * NUM_THRUSTERS)(*(dirs or [1] * NUM_THRUSTERS))
        out = (ctypes.c_int16 * NUM_THRUSTERS)()
        self._lib.bench_to_dshot(n, d, 1 if armed else 0, out)
        return list(out)

    def one_to_dshot(self, norm: float, direction: int = 1) -> int:
        """One normalised output -> one DShot value. `mixer::oneToDshot`."""
        return int(self._lib.bench_one_to_dshot(norm, direction))

    def drive(self, *, roll=0.0, pitch=0.0, yaw=0.0, throttle=0.0,
              forward=0.0, lateral=0.0, armed=True,
              frame_reverse=None, dirs=None) -> list[int]:
        """Body demands -> the DShot values THIS HULL would actually clock out.

        ⛔ USE THIS, NOT `mix()` + `to_dshot()`, for anything compared against
        the vehicle. The mixer alone is not the vehicle: `FRAME_REVERSE` negates
        every axis before it and the per-motor direction product is applied
        after it. Driving the bare mixer and calling the result "the board" was
        wrong by a mean of 712 counts of 999 on the first falsifier run -- the
        answer was not close, it was a different answer.

        `forward` and `lateral` are expected PRE-expo; apply `pilot_expo` first,
        as the board does. `throttle` is raw and `yaw` is a controller output,
        so neither is shaped.
        """
        fr = FRAME_REVERSE if frame_reverse is None else frame_reverse
        s = -1.0 if fr else 1.0
        norm = self.mix(roll=s * roll, pitch=s * pitch, yaw=s * yaw,
                        throttle=s * throttle, forward=s * forward,
                        lateral=s * lateral)
        return self.to_dshot(norm, dirs=list(dirs or MOTOR_DIRS), armed=armed)

    def set_battery(self, volts: float, dt_s: float = 0.1) -> None:
        """Feed the thruster-pack voltage. ⚠ `DEF_MOT_BAT_V_MAX` is 0.0, so
        compensation is OFF in the shipped configuration and this does nothing
        until `bat_v_max` is also set."""
        self._lib.bench_set_battery(volts, dt_s)

    @property
    def battery_scale(self) -> float:
        return float(self._lib.bench_battery_scale())

    def motor_angular(self, motor: int) -> tuple[float, float, float]:
        """The roll/pitch/yaw a positive drive on `motor` should produce."""
        r, p, y = (ctypes.c_float() for _ in range(3))
        self._lib.bench_motor_angular(motor, ctypes.byref(r), ctypes.byref(p),
                                      ctypes.byref(y))
        return r.value, p.value, y.value


def pilot_expo(d: float, expo: float = 0.30) -> float:
    """⚠ THE ONE STAGE THAT IS NOT THE FIRMWARE'S COMPILED CODE.

    `task_control_loop.cpp:161` applies this before the mixer, but that
    translation unit needs FreeRTOS, so it is reproduced here. It is three
    operations and it is pinned by `test_actuation_model.py` against the board,
    which is why reproducing it is acceptable where reproducing a control law
    would not be.

    Applies to FORWARD and LATERAL only. Throttle is raw; yaw is a controller
    output. See `actuation_model.SHAPED_AXES`.
    """
    expo = min(max(expo, 0.0), 1.0)
    return (1.0 - expo) * d + expo * d ** 3


def firmware_rev() -> str:
    """Which firmware commit this bench was built from."""
    f = HERE / '.firmware_rev'
    return f.read_text().strip() if f.exists() else 'unknown'


if __name__ == '__main__':
    b = Board()
    print(f'firmware {firmware_rev()}  thst_expo={b.thst_expo}  '
          f'spin_min={b.spin_min}')
    for d in (0.0, 0.02, 0.30, 1.00):
        norm = b.mix(forward=pilot_expo(d))
        print(f'  fwd {d:4.2f} -> norm {[round(v, 3) for v in norm[:4]]} '
              f'-> dshot {b.to_dshot(norm)[:4]}')
    sys.exit(0)
