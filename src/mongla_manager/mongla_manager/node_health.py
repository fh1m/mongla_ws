"""Is the node alive, and is the mission: the health report and the three watchdogs (executor, mission, kill switch).

Mixed into `AUVManagerNode` (auv_manager_node.py), which owns the board,
the publishers and the state these methods read.
"""
from __future__ import annotations

import time

from . import health as _health, health_reporters as _hr
from .node_support import (EXECUTOR_STALE_S, MISSION_SILENT_S, _load_sensor_offsets,
                           kill_watch, mission_watchdog)


class HealthMixin:
    # ── issue #13: who keeps the vehicle honest when a process dies ──────────
    def _stamp_executor_alive(self) -> None:
        self._executor_alive_t = time.monotonic()

    def _executor_alive(self) -> bool:
        """True until the executor has been stuck EXECUTOR_STALE_S.

        Before the first stamp -- bring-up, before spin() -- it is True: the
        heartbeat thread exists precisely to cover that window.
        """
        t = getattr(self, '_executor_alive_t', None)
        return t is None or (time.monotonic() - t) <= EXECUTOR_STALE_S

    def _on_mission_alive(self, msg) -> None:
        name = str(msg.data)
        if name:
            if not self._mission['attached']:
                self.get_logger().info(f'[MISSN] mission {name!r} attached -- '
                                       f'keep-alive watched ({MISSION_SILENT_S:.0f}s)')
            self._mission.update(attached=True, name=name, seen=time.monotonic())
        else:
            if self._mission['attached']:
                self.get_logger().info(f'[MISSN] mission {self._mission["name"]!r} '
                                       f'detached cleanly')
            self._mission.update(attached=False, name='')

    def _mission_watchdog_tick(self) -> None:
        m = self._mission
        try:
            armed = bool(self.fc.is_armed())
        except Exception:                          # noqa: BLE001 -- unknown = act
            armed = True
        self._kill_watch_tick(armed)
        action = mission_watchdog(m['attached'], m['seen'], time.monotonic(), armed)
        if action == 'none':
            return
        name = m['name']
        m.update(attached=False, name='')
        if action == 'detach':
            self.get_logger().warning(f'[MISSN] mission {name!r} went silent on a '
                                      f'DISARMED vehicle -- detached, nothing to do')
            return
        self.get_logger().error(
            f'[MISSN] !! mission {name!r} SILENT {MISSION_SILENT_S:.0f}s+ with the '
            f'vehicle ARMED -- the mission process is gone. Abort, brake, SURFACE.')
        try:
            self.mongla.request_abort()
        except Exception as exc:                   # noqa: BLE001
            self.get_logger().error(f'[MISSN] abort raised {exc!r}')
        try:
            res = self.mongla.surface(wait=False)
            self.get_logger().error(f'[MISSN] surface: {getattr(res, "message", res)}')
        except Exception as exc:                   # noqa: BLE001
            self.get_logger().error(f'[MISSN] SURFACE FAILED: {exc!r} -- the '
                                    f"board's own failsafe is the backstop")

    def _kill_watch_tick(self, armed: bool) -> None:
        """Disarm on the edge to ENGAGED; see `node_support.kill_watch`."""
        try:
            killed = getattr(self.fc.telemetry(), 'kill_switch', None)
        except Exception:                          # noqa: BLE001 -- no reading, no edge
            return
        was = getattr(self, '_kill_was', None)
        self._kill_was = killed
        if not kill_watch(was, killed, armed):
            return
        self.get_logger().error('[KILL ] kill switch ENGAGED while armed -- thruster '
                                'power is cut. Abort the mission and DISARM, so '
                                'restoring power cannot resume a move.')
        try:
            res = self.mongla.disarm(timeout=2.0)
            self.get_logger().error(f'[KILL ] {getattr(res, "message", res)}')
        except Exception as exc:                   # noqa: BLE001
            self.get_logger().error(f'[KILL ] DISARM FAILED: {exc!r}')

    def _register_health(self) -> None:
        """Translate what each subsystem already knows into one vocabulary.

        Every reporter is wrapped so a missing method is UNKNOWN rather than an
        AttributeError: a
        health board that crashes the node it is watching is worse than no
        board at all.
        """
        fc = self.fc

        def named(name):
            fn = getattr(fc, '_named_value', None)
            return fn(name) if fn else None

        self._health.register('board_link', lambda: _hr.board_link(fc))
        # Registered right beside board_link ON PURPOSE (B40): when the reader
        # thread dies, board_link says "no heartbeat" and looks exactly like a
        # dead cable. This is the line that names the real side.
        self._health.register('mavlink_reader', lambda: _hr.mavlink_reader(
            lambda: bool(getattr(self, 'reader_thread', None) is not None
                         and self.reader_thread.is_alive()),
            lambda: int(getattr(self, '_reader_faults', 0)),
            lambda: str(getattr(self, '_reader_last_fault', ''))))
        self._health.register('barometer', lambda: _hr.barometer(named))
        self._health.register('heading_ref', lambda: _hr.heading_reference(named))
        # Thrust-allocator saturation. Reads UNKNOWN until srot-control-board#20
        # merges -- `mixer::mix()` computes the per-group scale-down and drops
        # it, so today a saturating allocator and a quiet one are the same from
        # here. Registered NOW so the day the names appear the line is already
        # watching, rather than the capability arriving with no consumer.
        self._health.register('allocator', lambda: _hr.allocator(
            getattr(fc, 'allocator_saturation', lambda: None)()))
        self._health.register('thrusters', lambda: _hr.thrusters(
            getattr(fc, 'thruster_health', lambda: None)()))
        self._health.register('thruster_power', lambda: _hr.thruster_power(
            getattr(fc.telemetry(), 'kill_switch', None)))
        # Pilot gain. The board boots at 0.5, the parameter write cannot reach
        # the live value, and at 0.5 every MANUAL_CONTROL demand -- which is how
        # both vision verbs actuate -- is halved with nothing to indicate it.
        # The firmware's own source says a pool session was lost to exactly
        # that. `fc.read_gain()` existed for this and was called by nothing.
        self._health.register('pilot_gain', lambda: _hr.pilot_gain(
            getattr(fc, 'read_gain', lambda: None)()))
        self._health.register('sensor_offsets', lambda: _hr.sensor_offsets(
            _load_sensor_offsets()))
        self._health.register('detector', lambda: _hr.detector(
            self._detection_rate_hz()))
        # LEAK. Nine reporters were defined here and six were registered; this
        # was one of the three that never ran, and it is the one written for a
        # flooding hull. It returns FAILED for `LEAK_EN = 0` -- the state this
        # board was MEASURED in -- because a disabled failsafe still reads dry,
        # so the vehicle looks safe precisely when nothing is watching.
        # It needed `fc.leak_state()`, which splits "enabled" from "leaking";
        # `sys_status_leak()` collapses both into None and cannot express the trap.
        self._health.register('leak', lambda: _hr.leak_sensor(
            *(getattr(fc, 'leak_state', lambda: (None, None))())))
        self._health.register('target_pose', lambda: _hr.target_pose(
            (lambda vs: vs.target_pose() if vs is not None else None)(
                self._live_vision_state())))
        # `target_lock` stays UNREGISTERED, deliberately. It wants the ladder's
        # decayed `authority` (0..1); `/lock` carries the rung name and a
        # CONFIDENCE in class_id/score, which is a different quantity. Deriving
        # one from the other would publish an invented number in a health line --
        # the exact failure this board exists to catch. It needs a real accessor
        # on VisionState first. Tracked in DEFERRED_FUNCTIONS
        # (mongla_vision/test/test_no_function_is_built_and_uncalled.py).

    def _health_tick(self) -> None:
        self._health.poll()
        worst = self._health.worst()
        if worst is self._health_last:
            return
        self._health_last = worst
        if worst is _health.State.OK:
            self.get_logger().info('[HLTH ] all subsystems OK')
            return
        bad = '; '.join(str(h) for h in self._health.not_ok())
        log = (self.get_logger().warn
               if worst is _health.State.DEGRADED else self.get_logger().error)
        log(f'[HLTH ] {worst.name}: {bad}')
