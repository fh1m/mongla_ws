"""Is the node alive, and is the mission: the health report and the three watchdogs (executor, mission, kill switch).

Mixed into `AUVManagerNode` (auv_manager_node.py), which owns the board,
the publishers and the state these methods read.
"""
from __future__ import annotations

import time

from . import health as _health, health_reporters as _hr
from .node_support import (EXECUTOR_STALE_S, KILL_MAX_TRIES, MISSION_SILENT_S, boot_latch,
                           SURFACE_MAX_TRIES, _load_sensor_offsets, kill_latch,
                           mission_recovery, mission_watchdog)


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

    def _armed_now(self):
        """True / False from a fresh vehicle heartbeat; None when unknown."""
        try:
            fn = getattr(self.fc, 'armed_state', None)
            return fn() if fn is not None else bool(self.fc.is_armed())
        except Exception:                          # noqa: BLE001 -- unknown
            return None

    def _heard_since(self, t: float):
        """(armed, mode) from a heartbeat received after `t`; (None, None) if none."""
        try:
            return self.fc.armed_heard_since(t), self.fc.mode_heard_since(t)
        except Exception:                          # noqa: BLE001 -- unknown
            return None, None

    def _heartbeat_allowed(self) -> bool:
        """The heartbeat thread's gate: a stuck executor, an unconfirmed
        mission recovery or an unconfirmed kill disarm each stop it, so the
        board's own failsafe fires."""
        rec = getattr(self, '_recovery', None)
        kill = getattr(self, '_kill_latch', None)
        return (self._executor_alive() and not (rec and rec.get('withholding'))
                and not (kill and kill.get('withholding')))

    def safety_latch_reason(self) -> str:
        """Why ordinary verbs are refused right now; '' when they are not.

        Every latch clears only when the board itself reports the safe state.
        A manager restart does not clear them: a board heard ARMED at start-up
        latches the new manager too (#98), so the way out is a confirmed
        DISARM, never a reboot.
        """
        rec = getattr(self, '_recovery', None)
        if rec is not None:
            return (f'mission {rec["name"]!r} died with the vehicle armed; SURFACE '
                    f'is not yet confirmed by the board')
        if getattr(self, '_kill_latch', None) is not None:
            return ('the kill switch was engaged while armed and the board has '
                    'not yet confirmed DISARMED')
        # Decide the boot latch now, not on the next watchdog tick: an arm
        # sent before that tick would otherwise be read as a board that was
        # armed before this manager started.
        if getattr(self, '_boot_latch', None) == 'pending':
            self._boot_tick(self._armed_now())
        boot = getattr(self, '_boot_latch', None)
        if boot == 'armed':
            return ('the board was ARMED when this manager started, so a move '
                    'from before may still be running; disarm it first')
        if boot == 'pending':
            return ('no heartbeat from the board yet, so this manager cannot '
                    'tell whether a move from before is still running')
        return ''

    def _mission_watchdog_tick(self) -> None:
        now = time.monotonic()
        m = self._mission
        armed = self._armed_now()
        self._boot_tick(armed)
        self._kill_watch_tick(armed, now)
        self._recovery_tick(now)
        # Unknown is treated as armed: acting on a disarmed vehicle is noise,
        # not acting on an armed one is the failure.
        action = mission_watchdog(m['attached'], m['seen'], now, armed is not False)
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
        self._recovery = {'name': name, 'first': now, 'last': now, 'tries': 1}
        self._request_surface()

    def _request_surface(self) -> None:
        try:
            self.mongla.request_abort()
        except Exception as exc:                   # noqa: BLE001
            self.get_logger().error(f'[MISSN] abort raised {exc!r}')
        try:
            res = self.mongla.surface(wait=False)
            self.get_logger().error(f'[MISSN] surface requested: '
                                    f'{getattr(res, "message", res)}')
        except Exception as exc:                   # noqa: BLE001
            self.get_logger().error(f'[MISSN] SURFACE request FAILED: {exc!r} -- '
                                    f'will retry')

    def _recovery_tick(self, now: float) -> None:
        """Hold the mission-loss recovery until the board confirms it (issue #72)."""
        rec = getattr(self, '_recovery', None)
        if rec is None:
            return
        armed, mode = self._heard_since(rec['first'])
        self._recovery, action = mission_recovery(rec, now, armed, mode)
        log = self.get_logger()
        if action == 'resolved':
            what = 'DISARMED' if armed is False else 'in SURFACE'
            log.warning(f'[MISSN] recovery confirmed: the board reports {what}'
                        + (' -- heartbeat resumed' if rec.get('withholding') else ''))
        elif action == 'surface':
            log.error(f'[MISSN] SURFACE not confirmed -- request '
                      f'{self._recovery["tries"]}/{SURFACE_MAX_TRIES}')
            self._request_surface()
        elif action == 'withhold':
            log.error(f'[MISSN] !! SURFACE still unconfirmed after '
                      f'{SURFACE_MAX_TRIES} requests -- heartbeat WITHHELD so the '
                      f"board's own link-loss failsafe surfaces the vehicle")

    def _boot_tick(self, armed) -> None:
        """Latch a board heard ARMED at start-up (see `node_support.boot_latch`)."""
        self._boot_latch, action = boot_latch(getattr(self, '_boot_latch', None),
                                              armed)
        if action == 'latched':
            self.get_logger().error(
                '[BOOT ] the board is ARMED and this manager just started: a move '
                'from before may still be running. Ordinary verbs are refused '
                'until the board reports DISARMED (disarm, stop and surface work).')
        elif action == 'resolved':
            self.get_logger().warning('[BOOT ] the board reports DISARMED -- '
                                      'ordinary verbs allowed')

    def _kill_watch_tick(self, armed, now: float = None) -> None:
        """Disarm on the kill switch and keep at it until the board confirms.

        See `node_support.kill_latch` (issue #71).
        """
        now = time.monotonic() if now is None else now
        try:
            killed = getattr(self.fc.telemetry(), 'kill_switch', None)
        except Exception:                          # noqa: BLE001 -- unknown
            killed = None
        latch = getattr(self, '_kill_latch', None)
        self._kill_latch, action = kill_latch(latch, killed, armed, now)
        log = self.get_logger()
        if action == 'resolved':
            log.warning('[KILL ] the board reports DISARMED. Arming again is a '
                        'deliberate act, and refused while the switch is engaged.')
            return
        if action not in ('disarm', 'withhold'):
            return
        if action == 'withhold':
            log.error(f'[KILL ] !! DISARM still unconfirmed after {KILL_MAX_TRIES} '
                      f"attempts -- heartbeat WITHHELD so the board's own link-loss "
                      f'failsafe ends the move; still re-sending DISARM')
        elif latch is None:
            log.error('[KILL ] kill switch ENGAGED while armed -- thruster power is '
                      'cut. Abort the mission and DISARM, so restoring power '
                      'cannot resume a move.')
        else:
            log.error(f'[KILL ] disarm not yet confirmed -- attempt '
                      f'{self._kill_latch["tries"]}')
        try:
            self.mongla.request_abort()
        except Exception:                          # noqa: BLE001
            pass
        try:
            res = self.mongla.disarm(timeout=2.0)
            ok = getattr(res, 'success', None)
            log.error(f'[KILL ] {getattr(res, "message", res)}'
                      + ('' if ok else ' -- will retry until the board confirms'))
        except Exception as exc:                   # noqa: BLE001
            log.error(f'[KILL ] DISARM FAILED: {exc!r} -- will retry')

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
        # The two latches (issues #71, #72): FAILED until the board confirms.
        self._health.register('safety_latch', lambda: (
            _health.failed('safety_latch', self.safety_latch_reason())
            if self.safety_latch_reason()
            else _health.ok('safety_latch', 'no unconfirmed recovery')))
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
