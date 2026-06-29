"""Move.Result/Feedback CDR round-trip guard for the vision end-state contract.

The DSL derives ``saw_target = not isnan(x_px)`` from a NaN written into a
float32 and shipped across the action/DDS boundary. If NaN were ever coerced to
0.0 by the serialization layer, ``saw_target`` would become always-True and a
never-detected target would report "ended dead-centre" -- a silent, dangerous
inversion that drives a bad autonomous decision. Pin the round-trip so a future
interface/runtime change can't break it unnoticed.
"""

import math

from rclpy.serialization import serialize_message, deserialize_message

from duburi_interfaces.action import Move


def _round_trip(msg, cls):
    return deserialize_message(serialize_message(msg), cls)


def test_result_nan_survives_roundtrip():
    r = Move.Result()
    r.end_x_px = float('nan')
    r.end_y_px = float('nan')
    r2 = _round_trip(r, Move.Result)
    assert math.isnan(r2.end_x_px) and math.isnan(r2.end_y_px)


def test_result_signed_end_position_survives_roundtrip():
    r = Move.Result()
    r.end_x_px = -42.0
    r.end_y_px = 8.0
    r.fill_frac = 0.6
    r.elapsed_s = 3.1
    r2 = _round_trip(r, Move.Result)
    assert r2.end_x_px == -42.0 and r2.end_y_px == 8.0
    assert abs(r2.fill_frac - 0.6) < 1e-5 and abs(r2.elapsed_s - 3.1) < 1e-4


def test_feedback_nan_survives_roundtrip():
    f = Move.Feedback()
    f.err_x_px = float('nan')
    f.err_y_px = float('nan')
    f2 = _round_trip(f, Move.Feedback)
    assert math.isnan(f2.err_x_px) and math.isnan(f2.err_y_px)
