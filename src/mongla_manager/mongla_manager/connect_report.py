"""What `connect` prints: the vehicle, one section per subsystem.

`srot_connect.py` opens the link and collects a `Snapshot`; this turns it into
lines. Each section is one function (`_board`, `_power`, ... `_link`) so a
change to one subsystem's display is found and tested on its own. Absence
renders `--`, never `0.0` -- see the reading rule in srot_connect.py.
"""
from __future__ import annotations

import statistics

from .srot_connect import (_NAMED_GROUPS, Snapshot, _armed, _baro_healthy, _f,
                           _mode, baro_verdict, depth_loop_verdict,
                           read_safety_gates, sfmt, sp)

BOLD, DIM, RESET = sfmt.BOLD, sfmt.DIM, sfmt.RESET
RED, YEL, GRN = sfmt.RED, sfmt.YEL, sfmt.GRN


def render(snap: Snapshot, conn) -> list[str]:
    """The report, one section per function. Every absent value renders `--`, never 0."""
    return (_board(snap) + _power(snap) + _attitude(snap) + _depth(snap)
            + _thrusters(snap) + _payload(snap) + _firmware_health(snap)
            + _safety_config(conn) + _link(snap))


def _board(snap: Snapshot) -> list[str]:
    """Firmware, behaviour revision, armed state and mode."""
    av = snap.get('AUTOPILOT_VERSION')
    armed = _armed(snap.get('HEARTBEAT'))
    mode = _mode(snap.get('HEARTBEAT'))
    L: list[str] = []
    rev = int(getattr(av, 'middleware_sw_version', 0)) if av else None
    # This line used to render `Hengla v0.0.0` when the board had not
    # ANSWERED. AUTOPILOT_VERSION is one-shot and must be asked for, so an
    # unanswered request left `fsw = 0`, and the bit-shifts turned that into a
    # version number that looks like a real reading of a very old board. That
    # is the exact defect the board's own value suppression exists to prevent,
    # recreated on our side. `sfmt.fw_version` returns `--` for 0; it was
    # written for this line and never called from it.
    fsw = getattr(av, 'flight_sw_version', 0) if av else 0

    L.append(f'{BOLD}== SROT board =={RESET}')
    L.append(f'  firmware        Hengla {sfmt.fw_version(fsw)}   '
             f'behaviour rev {_f(rev, "{:.0f}")}')
    L.append(f'  state           {RED + "ARMED" + RESET if armed else "disarmed"}   '
             f'mode {mode}')
    if rev is not None and sp is not None and rev < sp.FW_BEHAVIOUR_REV_REQUIRED:
        L.append(f'  {RED}!! behaviour rev {rev} < {sp.FW_BEHAVIOUR_REV_REQUIRED} required '
                 f'-- MOVE_STOP COASTS, the host brake is gone{RESET}')
    return L


def _power(snap: Snapshot) -> list[str]:
    """The two batteries, the rail, and the pilot gain."""
    n = snap.named
    pwr = snap.get('POWER_STATUS')
    L: list[str] = []
    L.append(f'\n{BOLD}== power =={RESET}')
    labels = {0: 'PM1 electronics', 1: 'PM2 thruster pack'}
    for bid in sorted(snap.batteries) or []:
        v, cur = snap.batteries[bid]
        L.append(f'  battery {bid}       {_f(v, "{:6.3f}", " V"):>10}   '
                 f'{_f(cur, "{:.2f}", " A"):>8}   {DIM}{labels.get(bid, "")}{RESET}')
    if not snap.batteries:
        L.append('  battery         --   (no BATTERY_STATUS)')
    if pwr is not None:
        # Vcc is a HARDCODED 5000 in the firmware (mav_stream.cpp:328), not a
        # measurement -- labelled so nobody debugs a 5 V rail off a constant.
        # Vservo carries PM2 (the thruster pack, via ESP-NOW from the 2nd board), so
        # it is ABSENT rather than 0 V when that board is not fitted.
        L.append(f'  rail            Vcc {_f(pwr.Vcc / 1000.0, "{:.2f}", " V")} '
                 f'{DIM}(nominal, not measured){RESET}   '
                 f'Vservo {_f(sfmt.mv_to_volts(pwr.Vservo), "{:.2f}", " V")} '
                 f'{DIM}(= PM2 thruster pack){RESET}')
    L.append(f'  current (CURR)  {_f(n.get("CURR"), "{:.2f}", " A")}       '
             f'pilot GAIN {_f(n.get("GAIN"), "{:.2f}")}'
             + (f'  {YEL}<- halves MANUAL_CONTROL until 1.0{RESET}'
                if (n.get('GAIN') or 1.0) < 0.99 else ''))
    return L


def _attitude(snap: Snapshot) -> list[str]:
    """Roll, pitch, heading, and whether the board hears the companion."""
    n = snap.named
    att = snap.get('ATTITUDE')
    hud = snap.get('VFR_HUD')
    L: list[str] = []
    L.append(f'\n{BOLD}== attitude =={RESET}   {DIM}(heading is ABSOLUTE magnetic from rev 4){RESET}')
    if att is not None:
        # HEADING IS 0..360, matching the board's OLED, VFR_HUD.heading and
        # /mongla/state. Rendering the raw signed ATTITUDE.yaw here printed -162 next
        # to the board's 197 -- the same angle, disagreeing by exactly 360.
        # Roll and pitch stay SIGNED: a +3 deg list is not a 357 deg list.
        L.append(f'  roll {_f(sfmt.signed_deg(att.roll), "{:+7.2f}", "°")}   '
                 f'pitch {_f(sfmt.signed_deg(att.pitch), "{:+7.2f}", "°")}   '
                 f'heading {_f(sfmt.heading_deg(att.yaw), "{:7.2f}", "°")}')
    else:
        L.append('  --   (no ATTITUDE)')
    # VFR_HUD.heading is the board's own integer copy of the same angle. Shown beside
    # ours as a cross-check: these two must AGREE. If they ever differ by more than
    # rounding, one side's convention has drifted and that is worth seeing immediately.
    L.append(f'  board heading   {_f(getattr(hud, "heading", None), "{:.0f}", "°")}'
             f'   {DIM}(VFR_HUD -- must match above){RESET}'
             f'      MAGACC {_f(n.get("MAGACC"), "{:.0f}")}')

    # COMP_SEEN answers the one question a pre-dive check cannot answer any other
    # way: is the board's GCS failsafe actually WATCHING US? It is scoped by
    # FS_GCS_SYSID/FS_GCS_COMPID (default 255/191 = the companion), and if our
    # heartbeat does not match, the failsafe is satisfied by nobody -- or by Bondor
    # alone, which means a dead companion never surfaces the hull. 0 while we are
    # connected and heartbeating is a real finding, not a cosmetic one.
    _cs = n.get('COMP_SEEN')
    if _cs is None:
        _cs_txt = f'{DIM}not reported (firmware predates COMP_SEEN){RESET}'
    elif _cs >= 0.5:
        _cs_txt = f'{GRN}YES{RESET}  {DIM}-- GCS failsafe is scoped to us and satisfied{RESET}'
    else:
        _cs_txt = (f'{RED}NO{RESET}  -- the board has NEVER heard the companion it is '
                   f'scoped to. Check FS_GCS_SYSID/FS_GCS_COMPID against our '
                   f'sysid/compid; the GCS failsafe is not watching us.')
    L.append(f'  companion seen  {_cs_txt}')
    return L


def _depth(snap: Snapshot) -> list[str]:
    """Barometer, depth, water temperature, leak, kill switch, depth loop."""
    n = snap.named
    hud = snap.get('VFR_HUD')
    sysst = snap.get('SYS_STATUS')
    mode = _mode(snap.get('HEARTBEAT'))
    L: list[str] = []
    L.append(f'\n{BOLD}== depth / environment =={RESET}')
    # WHY the baro is unhealthy, not just that depth withdrew. 0 is the GOOD code,
    # so an absent reading must never be rendered as 0.
    _bh = n.get('BARO_HEALT')
    _bh_col = GRN if _bh == 0 else (DIM if _bh is None else RED)
    L.append(f'  baro health     {_bh_col}{sp.baro_health_text(_bh)}{RESET}'
             f'   {DIM}(BARO_HEALTH; 0=healthy, 3=not initialised){RESET}')
    lvl, msg = baro_verdict(snap.press_samples)
    col = {'OK': GRN, 'WARN': YEL, 'FAIL': RED}[lvl]
    L.append(f'  barometer       {col}{lvl}{RESET}  {msg}')
    # VFR_HUD.alt is NOT gated on baro health in the firmware (mav_stream.cpp:258),
    # unlike SCALED_PRESSURE2/WTEMP which ARE suppressed. So the board keeps streaming a
    # depth -- `-0.000` -- with no Bar30 fitted at all, and that number is meaningless.
    # Cross-check against the health bit rather than trusting the value's presence.
    _baro_ok = _baro_healthy(sysst)
    _depth_raw = getattr(hud, 'alt', None)
    if _baro_ok is False or (_baro_ok is None and not snap.press_samples):
        L.append(f'  depth           {sfmt.ABSENT}   {DIM}(VFR_HUD streams a value even '
                 f'with no barometer -- it is not gated on health){RESET}')
    else:
        L.append(f'  depth (VFR_HUD) {_f(_depth_raw, "{:+.3f}", " m")}'
                 f'   {DIM}negative = submerged{RESET}')
    wt = statistics.fmean(snap.wtemp_samples) if snap.wtemp_samples else None
    wspread = (max(snap.wtemp_samples) - min(snap.wtemp_samples)) if len(snap.wtemp_samples) > 1 else None
    L.append(f'  water temp      {_f(wt, "{:.2f}", " °C")}'
             + (f'   {RED}spread {wspread:.1f} °C -- noise{RESET}'
                if wspread and wspread > 2.0 else ''))
    # Tri-state, deliberately: "no leak reported" and "we never heard from the leak
    # sensor" are different facts and only one of them means the hull is dry.
    _leak = n.get('LEAK')
    _kill = n.get('KILL')
    _leak_s = sfmt.bool_word(None if _leak is None else _leak >= 0.5,
                             RED + 'WET' + RESET, 'dry')
    L.append(f'  leak            {_leak_s}'
             f'          kill switch '
             f'{sfmt.bool_word(None if _kill is None else _kill >= 0.5, "ENGAGED", "clear")}')
    if sysst is not None:
        healthy = bool(_baro_ok)
        L.append(f'  board says baro {"healthy" if healthy else "UNHEALTHY"}'
                 + (f'   {YEL}<- but see the barometer line above{RESET}'
                    if healthy and lvl == 'FAIL' else ''))

    lvl, msg = depth_loop_verdict(n, mode)
    col = {'OK': GRN, 'WARN': YEL, 'FAIL': RED}[lvl]
    L.append(f'  depth loop      {col}{lvl}{RESET}  {msg}')
    return L


def _thrusters(snap: Snapshot) -> list[str]:
    """ESC RPM and temperature, and what an all-zero row means."""
    armed = _armed(snap.get('HEARTBEAT'))
    L: list[str] = []
    L.append(f'\n{BOLD}== thrusters =={RESET}')
    rpm, temp = [], []
    for blk in ('ESC_TELEMETRY_1_TO_4', 'ESC_TELEMETRY_5_TO_8'):
        m = snap.get(blk)
        if m is not None:
            rpm += [int(r) for r in getattr(m, 'rpm', ())]
            temp += [int(t) for t in getattr(m, 'temperature', ())]
    if rpm:
        L.append('  rpm             ' + sfmt.rpm_row(rpm))
        L.append('  temp °C         ' + sfmt.esc_temp_row(temp))
        # An all-zero ESC row is ambiguous on the wire: it is what an attached-but-idle
        # ESC sends AND what arrives when no Pico/ESC exists at all. Say which.
        if not any(rpm) and armed:
            L.append(f'  {YEL}all RPM zero while ARMED -- ESCs not reporting, or thruster '
                     f'power off. NOT a healthy idle.{RESET}')
        elif not any(rpm):
            L.append(f'  {DIM}all zero -- disarmed, or no Pico/ESCs attached '
                     f'(bare-board bench){RESET}')
    else:
        L.append(f'  rpm             {sfmt.ABSENT}   {DIM}(no ESC telemetry -- no Pico '
                 f'co-processor, or ESCs not running Bluejay){RESET}')
    return L


def _payload(snap: Snapshot) -> list[str]:
    """Which payload channels mongla_ws may drive; nothing when roles were not read."""
    L: list[str] = []
    if snap.roles:
        L.append(f'\n{BOLD}== payload (PCA9685) =={RESET}   '
                 f'{DIM}role is a FIRMWARE param, set in Bondor{RESET}')
        switch = [c for c, r in sorted(snap.roles.items())
                  if sp and r == sp.PCA_ROLE_SWITCH]
        servo = [c for c, r in sorted(snap.roles.items())
                 if sp and r == sp.PCA_ROLE_SERVO]
        # Read-as-something-else and not-read-at-all are different facts
        # (2026-10-03, on the vehicle). One list said "other/unreadable" and
        # held only channels that WERE read with another role; a channel whose
        # read failed is absent from `roles`, so it appeared in no list at all.
        # Board configuration and a lossy link want different fixes.
        disabled = [c for c, r in sorted(snap.roles.items())
                    if sp and r == sp.PCA_ROLE_DISABLED]
        other = [f'{c}(role {r})' for c, r in sorted(snap.roles.items())
                 if not sp or r not in (sp.PCA_ROLE_SWITCH, sp.PCA_ROLE_SERVO,
                                        sp.PCA_ROLE_DISABLED)]
        n_ch = sp.PCA9685_NUM_CH if sp else 16
        unread = [c for c in range(1, n_ch + 1) if c not in snap.roles]
        L.append(f'  {GRN}SWITCH{RESET} (mongla_ws may fire)   {switch or "--"}')
        L.append(f'  {DIM}SERVO  (on-board arm, ignored){RESET}  {servo or "--"}')
        if disabled:
            L.append(f'  {DIM}DISABLED (role 0, drives nothing){RESET} {disabled}')
        if other:
            L.append(f'  {YEL}other role{RESET}                    {other}')
        if unread:
            L.append(f'  {YEL}UNREAD (no PARAM_VALUE -- link){RESET}  {unread}  '
                     f'{DIM}fire() fails closed on these{RESET}')
        L.append(f'  {DIM}fire(N) uses these numbers directly, e.g. '
                 f'`mongla fire --fire_channel {switch[0] if switch else 9}`{RESET}')
    return L


def _firmware_health(snap: Snapshot) -> list[str]:
    """Free heap, task stacks, load and drop rate."""
    n = snap.named
    sysst = snap.get('SYS_STATUS')
    L: list[str] = []
    L.append(f'\n{BOLD}== firmware health =={RESET}')
    L.append(f'  free heap       {_f(n.get("HEAP"), "{:.0f}", " B")}')
    stacks = {k: v for k, v in n.items() if k.startswith('STK_')}
    if stacks:
        worst = min(stacks.items(), key=lambda kv: kv[1])
        L.append('  task stacks     ' + '  '.join(f'{k[4:]}:{int(v)}' for k, v in sorted(stacks.items())))
        if worst[1] < 512:
            L.append(f'  {RED}{worst[0]} high-water {int(worst[1])} words -- near overflow{RESET}')
    if sysst is not None:
        L.append(f'  load            {sysst.load / 10.0:.1f}%     '
                 f'drop rate {sysst.drop_rate_comm / 100.0:.2f}%')
    return L


def _safety_config(conn) -> list[str]:
    """Board parameters that silently disable a safety behaviour."""
    L: list[str] = []
    # A disabled gate is invisible. Twenty-one board params default to zero and
    # several of them gate safety behaviour; nothing errors when one is off, the
    # feature simply never happens. This section is the only place that looks.
    L.append(f'\n{BOLD}== safety config =={RESET}   '
             f'{DIM}params that silently disable behaviour{RESET}')
    try:
        gates = read_safety_gates(conn)
    except Exception as exc:                     # a probe fault is not a verdict
        gates = {}
        L.append(f'  {YEL}could not read parameters ({exc}){RESET}')
    if gates:
        findings = sp.safety_gate_findings(gates)
        for name, _exp, _sev, _g in sp.SAFETY_GATES:
            v = gates.get(name)
            shown = '--' if v is None else f'{v:g}'
            bad = next((f for f in findings if f[1] == name), None)
            if bad is None:
                L.append(f'  {GRN}OK{RESET}   {name:<14} {shown}')
            else:
                col = RED if bad[0] == 'CRIT' else YEL
                L.append(f'  {col}{bad[0]:<4}{RESET} {name:<14} {shown}   {col}{bad[2]}{RESET}')
        if not findings:
            L.append(f'  {GRN}every gate live{RESET}')
    return L


def _link(snap: Snapshot) -> list[str]:
    """Message rates over the window, named values, and recent STATUSTEXT."""
    n = snap.named
    L: list[str] = []
    L.append(f'\n{BOLD}== link =={RESET}  {DIM}{snap.window_s:.1f}s window{RESET}')
    for mt, c in sorted(snap.msg_counts.items(), key=lambda kv: -kv[1]):
        note = ''
        if mt == 'UNKNOWN_291':
            note = f'  {DIM}<- ESC_STATUS; not in any pymavlink dialect (we use ESC_TELEMETRY){RESET}'
        elif mt == 'BAD_DATA':
            note = f'  {YEL}<- framing errors{RESET}'
        L.append(f'  {mt:<24} {c:5d}   {c / max(snap.window_s, 0.1):6.2f} Hz{note}')
    L.append(f'  {DIM}NAMED_VALUE_FLOAT carries {len(snap.named)} distinct names, '
             f'burst on one msgid{RESET}')

    for grp, keys in _NAMED_GROUPS:
        present = [(k, n[k]) for k in keys if k in n]
        if present:
            L.append(f'  {DIM}{grp:<11}{RESET} ' +
                     '  '.join(f'{k}={v:g}' for k, v in present))

    if snap.statustexts:
        L.append(f'\n{BOLD}== STATUSTEXT =={RESET}')
        for t in snap.statustexts[-8:]:
            L.append(f'  {t}')
    return L
