"""Bench verbs: motor test, motor detect, autotune and motor tune.

None of these runs on a mission. Each spins thrusters on the board's own
routine and reports what the board said while it ran, so each is a
propellers-clear, human-on-the-kill-switch operation.

Mixed into `SrotFC`.
"""
from __future__ import annotations

import math
import time

from ._srot_base import _POLL_S
from . import srot_protocol as sp


class SrotBench:
    """The bench part of `SrotFC` (see the module docstring)."""

    def motor_test(self, motor_1based: int, throttle_pct: float,
                   seconds: float = 2.0, *, abort_fn=None) -> tuple:
        """Spin ONE thruster, holding the board's keep-alive for `seconds`.

        `MAV_CMD_DO_MOTOR_TEST` (209) has been fully implemented on the board
        the whole time and had no driver method here, so the one procedure that
        can identify a thruster by making it turn was reachable only from
        Bondor.

        The keep-alive is the safety mechanism, which is why this owns it.
        The board expires the test when the resends stop, and expiry
        auto-disarms -- so a caller that spins a motor and then wedges must not
        be able to hold it spinning. This blocks for the duration and pumps from
        inside, and any exit path (a raise, an abort, the deadline) stops
        resending. The window is clamped to 600..3000 ms by the firmware
        whatever we ask, so silence for one clamped window is the longest a
        thruster can run past us.

        Returns (ok, reason). ARMED is required and refused messages come back
        as the board's own text.

        Props off, or the vehicle restrained. This turns a thruster.
        """
        n = int(motor_1based)
        if not 1 <= n <= 8:
            return False, f'motor {n} out of range 1..8'
        pct = float(throttle_pct)
        if not math.isfinite(pct) or not -100.0 <= pct <= 100.0:
            return False, f'throttle {throttle_pct} must be finite and -100..100'
        if not self.is_armed():
            # The board says this too, but saying it here means the caller does
            # not have to spin a motor to find out.
            return False, ('motor test requires ARMED (the board refuses with '
                           '"Arm motors before testing motors.")')

        window_s = max(sp.MOTOR_TEST_WINDOW_MIN_MS,
                       min(sp.MOTOR_TEST_WINDOW_MAX_MS, 1000.0)) / 1000.0
        period = 1.0 / sp.MOTOR_TEST_KEEPALIVE_HZ
        deadline = time.monotonic() + max(0.0, float(seconds))
        sent = 0
        try:
            while time.monotonic() < deadline:
                if abort_fn is not None and abort_fn():
                    return False, f'motor test aborted after {sent} keep-alives'
                self._clear_ack()
                self._command_long(sp.CMD_DO_MOTOR_TEST,
                                   p1=float(n),
                                   p2=float(sp.MOTOR_TEST_THROTTLE_PERCENT),
                                   p3=pct,
                                   p4=window_s)
                sent += 1
                time.sleep(period)
                # Check for a refusal AFTER the sleep, not by blocking on one.
                # A refusal is a property of the request rather than of any
                # single keep-alive, so it only has to be noticed once -- and
                # blocking here would make the CADENCE depend on the link,
                # which is precisely what the keep-alive exists to survive. It
                # would also eat the caller's requested duration: waiting a
                # second for an ACK that never comes turned a 1 s test into one
                # keep-alive, which the board expires.
                ack = self._cache('COMMAND_ACK')
                if ack is not None and int(getattr(ack, 'command', 0)) == \
                        sp.CMD_DO_MOTOR_TEST:
                    res = int(ack.result)
                    if res != sp.ACK_ACCEPTED:
                        st = self._statustext()
                        return False, (f'motor test refused: '
                                       f'{sp.ACK_NAMES.get(res, res)}'
                                       + (f' ({st})' if st else ''))
        finally:
            # Stop resending, deliberately and on EVERY path. The board expires
            # the test within one clamped window and disarms, which is the
            # designed end state -- there is no "stop" command to send.
            pass
        return True, (f'motor {n} tested at {pct:.0f}% for {seconds:.1f}s '
                      f'({sent} keep-alives); the board expires and disarms '
                      f'within {sp.MOTOR_TEST_WINDOW_MAX_MS} ms of the last one')

    # ------------------------------------------------------------------ #
    #  MOTOR_DETECT -- GATE 0, behind an operator confirmation            #
    # ------------------------------------------------------------------ #
    def motor_detect_briefing(self, timeout: float = 2.0) -> str:
        """What this hull's direction configuration is, RIGHT NOW, read live.

        Three values decide what a detect run does, they are stored in three
        different places, and no display anywhere shows the product that
        actually reaches the mixer:

          CAL_MDIRn        what MOTOR_DETECT writes. It COMPOSES (fw rev 6:
                           `c' = c * agree`), so re-running is idempotent and a
                           wrong thruster converges in one pass -- but only
                           because it multiplies, which is the next line.
          MOT_n_DIRECTION  a separate parameter that MULTIPLIES with CAL_MDIRn.
                           On this hull M1 and M8 are -1, and they are set
                           runtime-only and unsaved. Flipping one for a motor
                           detect already reversed cancels the fix.
          FRAME_REVERSE    negates all six axis demands AFTER the mix. It is 1
                           here. A detect run that corrects the per-motor signs
                           does not know about it, so a successful detect leaves
                           this wrong and the whole vehicle inverted.

        Read live rather than assumed: a params-reset boot silently restores
        defaults, and the operator's only clue is one line in a burst of
        thirteen.
        """
        def _p(name):
            v = self.get_param(name, timeout=timeout)
            return None if v is None else int(round(v))

        cal = [_p(n) for n in sp.CAL_MDIR_PARAMS]
        mot = [_p(n) for n in sp.MOT_DIRECTION_PARAMS]
        fr = _p('FRAME_REVERSE')

        def _row(vals):
            return ' '.join('?' if v is None else ('-' if v < 0 else '+')
                            for v in vals)

        eff = []
        for c, m in zip(cal, mot):
            eff.append(None if (c is None or m is None) else c * m)
        # FRAME_REVERSE is applied to the AXIS demand, not per motor, so it is
        # reported beside the product rather than folded into it -- folding it
        # in would suggest a per-motor fix for a whole-vehicle inversion.
        lines = [
            'MOTOR DETECT -- this hull, read live:',
            f'  CAL_MDIR1..8     {_row(cal)}   (what detect writes)',
            f'  MOT_n_DIRECTION  {_row(mot)}   (multiplies with the above)',
            f'  effective mix    {_row(eff)}',
            f'  FRAME_REVERSE    {"?" if fr is None else fr}'
            + ('   <-- negates ALL six axes after the mix; a successful '
               'detect leaves this wrong' if fr else ''),
            '',
            'It pulses all eight thrusters at 30 % and then rewrites the signs',
            'the attitude controllers use. It needs WATER and a hull free to',
            'rotate: in air the gyro response is below the 0.05 rad/s gate, the',
            'run reports FAIL and writes nothing (fw rev 6 -- before that it',
            'reset every thruster to +1 and reported SUCCESS).',
            '',
            f'To run it: motor_detect({sp.MOTOR_DETECT_TOKEN!r})',
        ]
        return '\n'.join(lines)

    def motor_detect(self, confirm=None, *,
                     timeout: float = sp.MOTOR_DETECT_TIMEOUT_S,
                     abort_fn=None) -> tuple:
        """Run the board's MOTOR_DETECT (mode 20). Returns (ok, reason).

        Confirmation is the point of this method. Called without the token it
        does not run -- it reads the three live values and returns them as the
        refusal text, so the only way to reach the token is to have been shown
        what it will change. A bool `force=True` would have made an accidental
        full-thruster run one keystroke away.

        We do not check for water, and say so rather than pretending to. There
        is no wet sensor on this vehicle; the depth reading at the surface is
        the same whether the hull is in a pool or on a bench. The refusal text
        states the requirement instead of implying it was verified.

        The end state is MANUAL and DISARMED, and that is the DESIGNED one, not
        a fault (fw task_control_loop.cpp:383-406): detect has just rewritten
        the direction signs, and handing an armed hull straight to a closed-loop
        controller after an unverified sign change flipped the vehicle in water
        on 2026-08-07. The operator re-arms to verify.
        """
        if confirm != sp.MOTOR_DETECT_TOKEN:
            return False, ('motor detect NOT run -- confirmation required.\n\n'
                           + self.motor_detect_briefing())
        if not self.is_armed():
            # The board says this too ("Motor detect: arm first (needs water,
            # free to rotate)") and bounces back to STABILIZE, so without this
            # the caller watches a mode that silently reverted.
            return False, ('motor detect requires ARMED -- the board refuses '
                           'and falls back to STABILIZE')

        before = [self.get_param(n) for n in sp.CAL_MDIR_PARAMS]
        mark = time.time()
        ok, why = self.set_mode('MOTOR_DETECT')
        if not ok:
            return False, f'could not enter MOTOR_DETECT: {why}'

        deadline = time.monotonic() + max(1.0, float(timeout))
        while time.monotonic() < deadline:
            if abort_fn is not None and abort_fn():
                # Leaving the mode is what stops it driving motors
                # (task_control_loop.cpp:453). Disarm too, because a mode change
                # alone leaves thrusters live.
                self.set_mode('MANUAL')
                self.disarm()
                return False, 'motor detect aborted -- left the mode and disarmed'
            hb = self._vehicle_hb()
            mode = sp.mode_name(getattr(hb, 'custom_mode', -1))
            if mode == 'MANUAL' and not self.is_armed():
                break                       # the board's own completion signal
            time.sleep(_POLL_S)
        else:
            return False, (f'motor detect did not finish in {timeout:.0f}s '
                           f'(mode {sp.mode_name(getattr(self._vehicle_hb(), "custom_mode", -1))}, '
                           f'armed {self.is_armed()})'
                           + self._detect_texts(mark))

        after = [self.get_param(n) for n in sp.CAL_MDIR_PARAMS]
        changed = [i + 1 for i, (a, b) in enumerate(zip(before, after))
                   if a is not None and b is not None
                   and (a < 0) != (b < 0)]
        # A detect that changed NOTHING is ambiguous on purpose: it is either
        # idempotent success (already correct -- agree is +1 for every motor) or
        # a FAIL that correctly declined to write. The board's own text is what
        # separates them, so it is carried through rather than summarised away.
        return True, ('motor detect finished -- DISARMED in MANUAL (designed; '
                      're-arm to verify axes). CAL_MDIR changed on: '
                      + (', '.join(f'M{n}' for n in changed) if changed
                         else 'nothing -- already correct, or the run was '
                              'INCONCLUSIVE and declined to write')
                      + self._detect_texts(mark))

    def _detect_texts(self, since: float) -> str:
        """The board's own MotorDetect lines, verbatim.

        It prints DETECTED and EFFECTIVE on two lines because both together
        exceed the 50-char STATUSTEXT budget and a truncated direction list is
        actively misleading. Relaying them unedited is the point -- our summary
        above is derived, theirs is the source.
        """
        lines = [t for _, _, t in self.statustext_log(since=since)
                 if 'otorDetect' in t or 'otor detect' in t]
        return ('\n  ' + '\n  '.join(lines)) if lines else ''

    # ------------------------------------------------------------------ #
    #  AUTOTUNE (21) and MOTOR_TUNE (22)                                   #
    # ------------------------------------------------------------------ #

    def tune_progress(self):
        """Live tune telemetry: (percent, detail) -- or (None, '') when silent.

        The board publishes `STUNT_PRG` (reused for the autotune percentage,
        fw task_control_loop.cpp:847) and, during an autotune, the live
        limit-cycle measurement `AT_N` / `AT_AMP` / `AT_TU` / `AT_OKPCT`. They
        added those for a stated reason:

            "so a run can be watched from the GCS instead of only explained
             after it aborts"                      (fw control/autotune.h:34)

        Nothing on our side read any of it. The first version of `autotune()`
        polled mode and armed for up to 150 s in TOTAL SILENCE -- exactly the
        failure that comment exists to prevent, on a routine that is driving all
        eight thrusters at full authority. `AT_TU` and `AT_OKPCT` read 0 while
        collecting and settle when a phase converges, so "gathering" and "stuck"
        are distinguishable here and nowhere else.
        """
        pct = self._named_value('STUNT_PRG')
        bits = []
        for name, fmt in (('AT_N', '{:.0f} half-cycles'), ('AT_AMP', 'amp {:.3f}'),
                          ('AT_TU', 'Tu {:.2f}s'), ('AT_OKPCT', 'consensus {:.0f}%')):
            v = self._named_value(name)
            if v is not None:
                bits.append(fmt.format(v))
        return pct, ', '.join(bits)

    def _wait_for_tune_end(self, label, timeout, mark, abort_fn):
        """Poll until the board's own completion signal, or refuse.

        BOTH tunes finish DISARMED in STABILIZE -- which is NOT the end state
        MOTOR_DETECT uses (MANUAL, disarmed). Modelling these on detect and
        watching for MANUAL would wait out the full timeout on a run that
        succeeded, then report failure.
          MOTOR_TUNE  fw task_control_loop.cpp:830-833
          AUTOTUNE    fw task_control_loop.cpp:846-853
        """
        deadline = time.monotonic() + max(1.0, float(timeout))
        last_report = 0.0
        while time.monotonic() < deadline:
            # Say what the tune is doing instead of sitting silent for minutes.
            # Throttled: this loop polls at _POLL_S and the values move slowly.
            now_m = time.monotonic()
            if now_m - last_report >= 5.0:
                last_report = now_m
                pct, detail = self.tune_progress()
                if pct is not None or detail:
                    shown = '--' if pct is None else f'{pct:.0f}%'
                    self._log_info(f'[TUNE ] {label} {shown}'
                                   + (f' | {detail}' if detail else ''))
            if abort_fn is not None and abort_fn():
                # Leaving the mode is what stops the tuner -- the firmware calls
                # abort() on the falling edge (task_control_loop.cpp:532,540).
                # Disarm too: a mode change alone leaves thrusters live.
                self.set_mode('STABILIZE')
                self.disarm()
                return False, f'{label} aborted -- left the mode and disarmed'
            hb = self._vehicle_hb()
            mode = sp.mode_name(getattr(hb, 'custom_mode', -1))
            if mode == 'STABILIZE' and not self.is_armed():
                return True, ''
            time.sleep(_POLL_S)
        return False, (f'{label} did not finish in {timeout:.0f}s '
                       f'(mode {sp.mode_name(getattr(self._vehicle_hb(), "custom_mode", -1))}, '
                       f'armed {self.is_armed()})' + self._tune_texts(mark))

    def _tune_texts(self, since: float) -> str:
        """The board's own tune lines, verbatim. Ours is derived; theirs is source."""
        want = ('utotune', 'otor tune', 'MTune', 'Disarmed:')
        lines = [t for _, _, t in self.statustext_log(since=since)
                 if any(w in t for w in want)]
        return ('\n  ' + '\n  '.join(lines)) if lines else ''

    def autotune(self, confirm=None, *, timeout: float = sp.AUTOTUNE_TIMEOUT_S,
                 abort_fn=None) -> tuple:
        """Run the board's relay AUTOTUNE (mode 21). Returns (ok, reason).

        Astrom-Hagglund relay tuning of the rate loops, then the angle-P loops,
        then depth-hold -- and it WRITES AND PERSISTS EVERY ONE OF THOSE PIDs.
        The firmware's own warning is 'AUTOTUNE started - thrusters WILL drive'.

        We enter by SET_MODE, deliberately, and not by the `ATUNE` param.
        There are three trigger paths -- SET_MODE 21, `MAV_CMD_USER_5`, and
        `PARAM_SET ATUNE >= 1` -- and the latter two set `autotune_active`
        with no reference to arming. It LATCHES: the firmware comment at
        task_control_loop.cpp:513-519 records that setting ATUNE while disarmed
        left it set, and 'the next arm started a full-authority relay tune in
        whatever mode the pilot happened to be in, with the GCS and OLED still
        showing MANUAL'. They mitigated it by reflecting the trigger into the
        mode; we avoid arming the latch at all.

        We do not check for water, and say so rather than pretending to. There
        is no wet sensor. Run in air and the relay never establishes a limit
        cycle -- the hull cannot rotate freely -- so it consumes its phases and
        writes gains fitted to nothing.

        The end state is DISARMED in STABILIZE and that is DESIGNED, not a
        fault (fw task_control_loop.cpp:846-853).
        """
        if confirm != sp.AUTOTUNE_TOKEN:
            return False, ('autotune NOT run -- confirmation required.\n\n'
                           + self.autotune_briefing())
        if not self.is_armed():
            # The firmware gate is `(in.autotune || mode == AUTOTUNE) && in.armed`
            # (task_control_loop.cpp:508). Disarmed, the mode is accepted and the
            # tuner never starts -- a silent no-op.
            return False, ('autotune requires ARMED -- the board accepts the mode '
                           'and never starts the tuner while disarmed')

        before = {n: self.get_param(n) for n in sp.AUTOTUNE_PID_PARAMS}
        mark = time.time()
        ok, why = self.set_mode('AUTOTUNE')
        if not ok:
            return False, f'could not enter AUTOTUNE: {why}'

        ok, why = self._wait_for_tune_end('autotune', timeout, mark, abort_fn)
        if not ok:
            return False, why

        after = {n: self.get_param(n) for n in sp.AUTOTUNE_PID_PARAMS}
        moved = [n for n in sp.AUTOTUNE_PID_PARAMS
                 if before.get(n) is not None and after.get(n) is not None
                 and abs(after[n] - before[n]) > 1e-6]
        # A tune that moved NOTHING is not success. The depth phase is SKIPPED
        # without a healthy depth sensor (autotune.h:20-23), and a relay that
        # never established a limit cycle writes nothing either -- so an empty
        # list is reported as the ambiguity it is, with the board's own lines.
        return True, ('autotune finished -- DISARMED in STABILIZE (designed). '
                      + (f'PIDs changed: {", ".join(moved)}' if moved else
                         'NO PID CHANGED -- either the depth phase was skipped '
                         '(no healthy baro) or no phase established a limit '
                         'cycle. Read the board lines below before trusting it.')
                      + self._tune_texts(mark))

    def autotune_briefing(self, timeout: float = 2.0) -> str:
        """What an autotune run would overwrite on this hull, read LIVE."""
        rows = []
        for n in sp.AUTOTUNE_PID_PARAMS:
            v = self.get_param(n, timeout=timeout)
            rows.append(f'  {n:<12} {"--" if v is None else f"{v:.4f}"}')
        return ('AUTOTUNE will drive all thrusters at full authority for up to\n'
                f'{sp.AUTOTUNE_TIMEOUT_S:.0f}s and then OVERWRITE and PERSIST these:\n'
                + '\n'.join(rows)
                + '\n\nRequires: ARMED, IN WATER, free to rotate. There is no wet\n'
                  'sensor on this vehicle -- nothing verified that for you.\n'
                  'Without a healthy barometer the depth phase is SKIPPED.\n'
                  f'\nTo proceed pass confirm={sp.AUTOTUNE_TOKEN!r}')

    def motor_tune(self, confirm=None, *, timeout: float = sp.MOTOR_TUNE_TIMEOUT_S,
                   abort_fn=None) -> tuple:
        """Run the board's MOTOR_TUNE (mode 22). Returns (ok, reason).

        Per-thruster RPM-controller tuning: ramp to find idle, hold levels to fit
        the feedforward slope FF_A, then relay-tune the PI on the throttle->RPM
        plant. Averaged across motors, written to RPM_KP/KI/FF_A/IDLE +
        MOT_SPIN_MIN, persisted, and pushed to the Pico.

        Two preconditions that each fail silently.

        1. `MTUNE_EN` must be > 0.5 and it DEFAULTS TO ZERO (fw config.h:588
           `DEF_MTUNE_EN = 0.0f`). The gate is
           `(mode == MOTOR_TUNE) && (mtune_en > 0.5f) && armed`
           (task_control_loop.cpp:509) -- so SET_MODE alone enters the mode and
           the tuner never runs. This method reads the param and REFUSES rather
           than sitting in a mode that does nothing. It does not set it for you:
           enabling a motor-spinning mode is an operator decision.

        2. It fits on PER-MOTOR RPM TELEMETRY, which on this vehicle has never
           been non-zero -- 958 recorded ESC_STATUS frames, every rpm exactly 0,
           because telemetry needs Bluejay-flashed ESCs with bidirectional
           DShot. With no RPM the plant fit has no plant. We WARN rather than
           refuse on UNKNOWN (the same asymmetry as the pre-fire gate: `None`
           is this hull's permanent state and refusing on it would make the
           method unreachable forever), and REFUSE on a known-bad thruster.

        The end state is DISARMED in STABILIZE and that is DESIGNED
        (fw task_control_loop.cpp:830-833).
        """
        if confirm != sp.MOTOR_TUNE_TOKEN:
            return False, ('motor tune NOT run -- confirmation required.\n\n'
                           + self.motor_tune_briefing())

        en = self.get_param(sp.MOTOR_TUNE_ENABLE_PARAM)
        if en is None:
            return False, (f'could not read {sp.MOTOR_TUNE_ENABLE_PARAM} -- refusing '
                           'rather than entering a mode that may silently do nothing')
        if en <= 0.5:
            return False, (f'{sp.MOTOR_TUNE_ENABLE_PARAM} = {en:.1f}: MOTOR_TUNE is '
                           'DISABLED on this board, and the mode would be accepted '
                           'while the tuner never started. Set it to 1 deliberately '
                           '-- it is the interlock on a mode that spins motors.')
        if not self.is_armed():
            return False, ('motor tune requires ARMED -- the board accepts the mode '
                           'and never starts the tuner while disarmed')

        health, why = self.thruster_health()
        if health is False:
            return False, f'motor tune refused -- {why}'
        warn = ''
        if health is None:
            warn = ('\n  WARNING: thruster telemetry UNKNOWN -- this tune FITS on per-motor '
                    'RPM. If the ESCs are not Bluejay-flashed it will spin motors '
                    'and fit nothing.')

        before = {n: self.get_param(n) for n in sp.MOTOR_TUNE_PARAMS}
        mark = time.time()
        ok, why = self.set_mode('MOTOR_TUNE')
        if not ok:
            return False, f'could not enter MOTOR_TUNE: {why}'

        ok, why = self._wait_for_tune_end('motor tune', timeout, mark, abort_fn)
        if not ok:
            return False, why + warn

        after = {n: self.get_param(n) for n in sp.MOTOR_TUNE_PARAMS}
        moved = [n for n in sp.MOTOR_TUNE_PARAMS
                 if before.get(n) is not None and after.get(n) is not None
                 and abs(after[n] - before[n]) > 1e-6]
        return True, ('motor tune finished -- DISARMED in STABILIZE (designed). '
                      + (f'changed: {", ".join(moved)}' if moved else
                         'NOTHING CHANGED -- with zero RPM telemetry that is the '
                         'expected outcome, not a success')
                      + warn + self._tune_texts(mark))

    def motor_tune_briefing(self, timeout: float = 2.0) -> str:
        """The two silent preconditions and the live RPM, quoted back."""
        en = self.get_param(sp.MOTOR_TUNE_ENABLE_PARAM, timeout=timeout)
        rpm = self.esc_status_rpm()
        health, hwhy = self.thruster_health()
        rows = []
        for n in sp.MOTOR_TUNE_PARAMS:
            v = self.get_param(n, timeout=timeout)
            rows.append(f'  {n:<14} {"--" if v is None else f"{v:.4f}"}')
        return ('MOTOR_TUNE spins ONE THRUSTER AT A TIME through a throttle ramp\n'
                'and then OVERWRITES and PERSISTS these:\n'
                + '\n'.join(rows)
                + f'\n\n  {sp.MOTOR_TUNE_ENABLE_PARAM:<14} '
                + ('--  (unreadable)' if en is None else
                   f'{en:.1f}  ' + ('OK' if en > 0.5 else
                                    'ERROR: DISABLED -- the mode would do NOTHING'))
                + '\n  live signed RPM  '
                + ('-- (no ESC telemetry at all)' if rpm is None else str(list(rpm)))
                + f'\n  thruster health  {hwhy}'
                + ('\n  WARNING: this tune FITS ON RPM. All-zero means it has no plant.'
                   if health is not True else '')
                + '\n\nRequires: ARMED, IN WATER, props on. There is no wet sensor\n'
                  'on this vehicle -- nothing verified that for you.\n'
                  f'\nTo proceed pass confirm={sp.MOTOR_TUNE_TOKEN!r}')
