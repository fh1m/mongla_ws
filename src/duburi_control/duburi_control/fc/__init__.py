"""FlightController HAL -- the autopilot-backend boundary (pixhawk | srot).

Pure modules (`base`, `srot_protocol`) import eagerly; the pymavlink-dependent
backends are built lazily by the factory, so `from duburi_control.fc import
srot_protocol` stays import-light.
"""

from .base import (FlightController, Telemetry, MoveResult,
                   SUCCEEDED, PREEMPTED, FAILED, DENIED, TIMEOUT, ABORTED)
from .factory import make_flight_controller, BUILDERS

__all__ = [
    'FlightController', 'Telemetry', 'MoveResult',
    'SUCCEEDED', 'PREEMPTED', 'FAILED', 'DENIED', 'TIMEOUT', 'ABORTED',
    'make_flight_controller', 'BUILDERS',
]
