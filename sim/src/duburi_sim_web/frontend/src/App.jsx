import { useCallback, useEffect, useRef, useState } from 'react'

// What a gain setting actually DELIVERS, which is not what it reads.
//
// The T200 does not begin producing thrust until PWM 1528 (a 28 us deadband
// either side of neutral, then a quadratic curve), so the bottom of the range
// is nearly inert and the middle is weaker than the number suggests: gain 55%
// is 39% of thrust, and 15% is about 4%. Showing both numbers is the honest
// version -- otherwise "half gain" reads like half speed and is not.
const PWM_DEAD = 1528, PWM_MAX = 1900;
function thrustPct(g) {
  const pwm = 1500 + g * 400;
  if (pwm <= PWM_DEAD) return 0;
  const u = (pwm - PWM_DEAD) / (PWM_MAX - PWM_DEAD);
  const full = 25.69 + 27.95 - 0.43;
  return Math.round((25.69 * u * u + 27.95 * u - 0.43) / full * 100);
}

const PAGES = ['operate', 'score', 'world', 'datasets']

async function api(path, opts = {}) {
  const res = await fetch(path, {
    headers: { 'Content-Type': 'application/json', ...(opts.headers || {}) },
    ...opts,
  })
  const text = await res.text()
  let data = null
  try {
    data = text ? JSON.parse(text) : null
  } catch {
    data = { raw: text }
  }
  if (!res.ok) {
    throw new Error((data && (data.detail || data.message)) || res.statusText)
  }
  return data
}

function fmtM(v, digits = 2) {
  if (v == null || Number.isNaN(Number(v))) return '—'
  return `${Number(v).toFixed(digits)} m`
}

function fmtDeg(v) {
  if (v == null || Number.isNaN(Number(v))) return '—'
  return `${Number(v).toFixed(1)}°`
}

function fmtPos(gt) {
  if (!gt?.have) return 'no fix'
  return `x ${Number(gt.x).toFixed(2)} m  ·  y ${Number(gt.y).toFixed(2)} m  ·  z ${Number(gt.z).toFixed(2)} m`
}

function fmtWhen(iso) {
  if (!iso) return '—'
  try {
    const d = new Date(iso)
    return d.toLocaleString(undefined, {
      month: 'short',
      day: '2-digit',
      hour: '2-digit',
      minute: '2-digit',
      second: '2-digit',
    })
  } catch {
    return iso
  }
}

function Crosshair() {
  return (
    <>
      <span className="xh tl" />
      <span className="xh tr" />
      <span className="xh bl" />
      <span className="xh br" />
    </>
  )
}

function Cam({ name }) {
  return (
    <figure className="cam">
      <Crosshair />
      <figcaption>{name}_camera</figcaption>
      <img src={`/api/cameras/${name}/mjpeg`} alt={name} />
    </figure>
  )
}

function CamGrid({ mode }) {
  if (mode === 'front') {
    return (
      <div className="cams single fill">
        <Cam name="front" />
      </div>
    )
  }
  if (mode === 'bottom') {
    return (
      <div className="cams single fill">
        <Cam name="bottom" />
      </div>
    )
  }
  return (
    <div className="cams dual">
      <Cam name="front" />
      <Cam name="bottom" />
    </div>
  )
}

// Deadzone + expo, identical maths to duburi_sim_web/joystick.py so a pad on
// the browser machine and one on the lab machine fly the same.
const JOY_DEADZONE = 0.08
const JOY_EXPO = 0.35
function joyShape(v) {
  const x = Math.max(-1, Math.min(1, v || 0))
  if (Math.abs(x) <= JOY_DEADZONE) return 0
  const sign = x > 0 ? 1 : -1
  const t = (Math.abs(x) - JOY_DEADZONE) / (1 - JOY_DEADZONE)
  return sign * ((1 - JOY_EXPO) * t + JOY_EXPO * t ** 3)
}

// A gamepad plugged into the machine running the BROWSER. This is the QGC
// arrangement -- the ground station reads the stick and sends commands over
// the link -- and it is the one that works when the lab is on another machine.
// A pad on the lab machine is read there instead (/api/vehicle/joystick); both
// end up in the same TeleopStreamer, so there is never a second RC writer.
// Two rounds of each, per the rulebook (p. 64: "up to two markers", "up to
// two torpedoes"). Practising against an unlimited magazine teaches a timing
// that does not exist on the vehicle.
const FIRE_CHANNELS = [1, 2]
const DROP_CHANNELS = [3, 4]

function useGamepad({ enabled, gain, onArm, onDisarm, onPayload }) {
  const [pad, setPad] = useState(null)
  const mag = useRef({ fire: [...FIRE_CHANNELS], drop: [...DROP_CHANNELS] })
  const state = useRef({ axes: [0, 0, 0, 0], buttons: [], sending: false })

  useEffect(() => {
    if (!enabled) return undefined
    let raf = 0
    let last = 0
    const prevButtons = []

    const tick = async (now) => {
      raf = requestAnimationFrame(tick)
      const pads = navigator.getGamepads ? navigator.getGamepads() : []
      const gp = Array.from(pads).find((p) => p && p.connected)
      if (!gp) {
        if (pad) setPad(null)
        return
      }
      // Xbox layout: LS x/y = 0/1, RS x/y = 2/3. Kernel js numbering differs
      // (triggers sit at 2), which is why the two readers do not share a map.
      const lat = joyShape(gp.axes[0])
      const fwd = joyShape(-gp.axes[1])
      const yaw = joyShape(gp.axes[2])
      const up = joyShape(-gp.axes[3])

      gp.buttons.forEach((b, i) => {
        const down = !!b.pressed
        if (down && !prevButtons[i]) {
          if (i === 0) onArm?.()
          if (i === 1) { mag.current = { fire: [...FIRE_CHANNELS], drop: [...DROP_CHANNELS] }; onDisarm?.() }
          if (i === 2 || i === 3) {
            const kind = i === 2 ? 'fire' : 'drop'
            const channel = mag.current[kind].shift()
            if (channel === undefined) onPayload?.(kind, null)
            else onPayload?.(kind, channel)
          }
        }
        prevButtons[i] = down
      })

      setPad({ id: gp.id, index: gp.index, axes: [fwd, lat, up, yaw],
               rounds: { fire: [...mag.current.fire], drop: [...mag.current.drop] } })

      // Fixed 25 Hz, not per-frame: the RC stream runs at 20 Hz, so anything
      // faster is wasted requests, and anything slower shows up as stepping.
      if (now - last < 40 || state.current.sending) return
      last = now
      const active = Math.max(Math.abs(fwd), Math.abs(lat), Math.abs(up), Math.abs(yaw)) > 0
      if (!active && !state.current.wasActive) return
      state.current.wasActive = active
      state.current.sending = true
      try {
        await api('/api/vehicle/teleop', {
          method: 'POST',
          body: JSON.stringify({ fwd, lat, up, yaw, gain }),
        })
      } catch {
        /* ignore */
      } finally {
        state.current.sending = false
      }
    }

    raf = requestAnimationFrame(tick)
    return () => {
      cancelAnimationFrame(raf)
      // Centre on unmount. A released stick that never reaches the server
      // leaves the last value latched in the RC stream.
      api('/api/vehicle/teleop', {
        method: 'POST',
        body: JSON.stringify({ fwd: 0, lat: 0, up: 0, yaw: 0 }),
      }).catch(() => {})
    }
  }, [enabled, gain, onArm, onDisarm, onPayload])

  return pad
}

function AxisBar({ label, value }) {
  const pct = Math.min(100, Math.abs(value) * 100)
  return (
    <div className="joy-axis">
      <span className="joy-axis-label">{label}</span>
      <div className="joy-axis-track">
        <div
          className="joy-axis-fill"
          style={{
            width: `${pct / 2}%`,
            left: value >= 0 ? '50%' : `${50 - pct / 2}%`,
          }}
        />
      </div>
      <span className="joy-axis-value">{value >= 0 ? '+' : ''}{value.toFixed(2)}</span>
    </div>
  )
}

function ControllerPanel({ browserPad, labPad, enabled, setEnabled }) {
  const active = browserPad || (labPad?.connected ? labPad : null)
  const axes = browserPad
    ? browserPad.axes
    : [labPad?.axes?.fwd || 0, labPad?.axes?.lat || 0,
       labPad?.axes?.up || 0, labPad?.axes?.yaw || 0]
  const where = browserPad ? 'browser' : 'lab host'
  const rounds = (browserPad?.rounds) || (labPad?.rounds)
    || { fire: [], drop: [] }
  return (
    <div className="joy-panel">
      <div className="joy-head">
        <span className={`joy-dot ${active ? 'live' : ''}`} />
        <strong>{active ? 'CONTROLLER ACTIVE' : 'no controller'}</strong>
        {active && <span className="muted joy-where">{where}</span>}
        <label className="joy-toggle">
          <input
            type="checkbox"
            checked={enabled}
            onChange={(e) => setEnabled(e.target.checked)}
          />
          browser pad
        </label>
      </div>
      {active ? (
        <>
          <div className="joy-name">
            {browserPad ? browserPad.id : `${labPad.name} (${labPad.device})`}
          </div>
          <AxisBar label="fwd" value={axes[0]} />
          <AxisBar label="lat" value={axes[1]} />
          <AxisBar label="up" value={axes[2]} />
          <AxisBar label="yaw" value={axes[3]} />
          <div className="muted joy-hint">
            {browserPad ? '' : `gain ${labPad.gain} — LB/RB · `}
            X fire {rounds.fire.length}/2 · Y drop {rounds.drop.length}/2
            {rounds.fire.length + rounds.drop.length === 0
              ? ' — empty, disarm to reload' : ''}
          </div>
        </>
      ) : (
        <div className="muted joy-hint">
          plug a pad in and press a button — or run the lab with
          {' '}<code>DUBURI_JOYSTICK=/dev/input/js0</code>
        </div>
      )}
    </div>
  )
}

function Dpad({ armed, onArmToggle, gain, setGain, busy }) {
  const axes = useRef({ fwd: 0, lat: 0, up: 0, yaw: 0 })
  const held = useRef(new Set())
  const timer = useRef(null)

  const push = useCallback(async () => {
    const a = axes.current
    try {
      await api('/api/vehicle/teleop', {
        method: 'POST',
        body: JSON.stringify({ ...a, gain }),
      })
    } catch {
      /* ignore */
    }
  }, [gain])

  const recompute = useCallback(() => {
    const h = held.current
    axes.current = {
      fwd: (h.has('fwd') ? 1 : 0) + (h.has('bk') ? -1 : 0),
      lat: (h.has('rt') ? 1 : 0) + (h.has('lt') ? -1 : 0),
      up: (h.has('up') ? 1 : 0) + (h.has('dn') ? -1 : 0),
      // yaw was hardcoded 0 here while TeleopBody.yaw, teleop.set_axes(yaw=)
      // and CH_YAW were all already wired end to end -- the axis existed, the
      // UI just never sent it.
      yaw: (h.has('yr') ? 1 : 0) + (h.has('yl') ? -1 : 0),
    }
    push()
  }, [push])

  const press = (key) => {
    held.current.add(key)
    recompute()
    if (!timer.current) timer.current = setInterval(push, 100)
  }

  const release = (key) => {
    held.current.delete(key)
    recompute()
    if (held.current.size === 0 && timer.current) {
      clearInterval(timer.current)
      timer.current = null
      axes.current = { fwd: 0, lat: 0, up: 0, yaw: 0 }
      push()
    }
  }

  useEffect(() => {
    const down = (e) => {
      if (e.repeat) return
      const map = {
        w: 'fwd', W: 'fwd', ArrowUp: 'fwd',
        s: 'bk', S: 'bk', ArrowDown: 'bk',
        a: 'lt', A: 'lt', ArrowLeft: 'lt',
        d: 'rt', D: 'rt', ArrowRight: 'rt',
        r: 'up', R: 'up',
        f: 'dn', F: 'dn',
        q: 'yl', Q: 'yl',
        e: 'yr', E: 'yr',
      }
      if (e.code === 'Space') {
        e.preventDefault()
        onArmToggle()
        return
      }
      const k = map[e.key]
      if (k) {
        e.preventDefault()
        press(k)
      }
    }
    const up = (e) => {
      const map = {
        w: 'fwd', W: 'fwd', ArrowUp: 'fwd',
        s: 'bk', S: 'bk', ArrowDown: 'bk',
        a: 'lt', A: 'lt', ArrowLeft: 'lt',
        d: 'rt', D: 'rt', ArrowRight: 'rt',
        r: 'up', R: 'up',
        f: 'dn', F: 'dn',
        q: 'yl', Q: 'yl',
        e: 'yr', E: 'yr',
      }
      const k = map[e.key]
      if (k) {
        e.preventDefault()
        release(k)
      }
    }
    window.addEventListener('keydown', down)
    window.addEventListener('keyup', up)
    return () => {
      window.removeEventListener('keydown', down)
      window.removeEventListener('keyup', up)
      if (timer.current) clearInterval(timer.current)
      api('/api/vehicle/teleop', {
        method: 'POST',
        body: JSON.stringify({ fwd: 0, lat: 0, up: 0, yaw: 0 }),
      }).catch(() => {})
    }
  }, [onArmToggle, push])

  const btn = (key, label, cls = '') => (
    <button
      type="button"
      className={`pad ${cls}`}
      disabled={busy}
      aria-label={key}
      onPointerDown={(e) => {
        e.preventDefault()
        e.currentTarget.setPointerCapture(e.pointerId)
        press(key)
      }}
      onPointerUp={() => release(key)}
      onPointerCancel={() => release(key)}
    >
      {label}
    </button>
  )

  return (
    <div className="dpad-wrap">
      <div className="dpad">
        <span className="pad ghost" />
        {btn('fwd', '▲')}
        <span className="pad ghost" />
        {btn('lt', '◀')}
        <button
          type="button"
          className={`pad arm ${armed ? 'armed' : ''}`}
          disabled={busy}
          onClick={onArmToggle}
          title="space"
        >
          {armed ? '●' : '○'}
          <span>{armed ? 'armed' : 'arm'}</span>
        </button>
        {btn('rt', '▶')}
        <span className="pad ghost" />
        {btn('bk', '▼')}
        <span className="pad ghost" />
      </div>
      <div className="heave">
        {btn('yl', '↺ yaw', 'heave-btn')}
        {btn('yr', 'yaw ↻', 'heave-btn')}
      </div>
      <div className="heave">
        {btn('up', '⬆ up', 'heave-btn')}
        {btn('dn', '⬇ dn', 'heave-btn')}
      </div>
      <label className="gain">
        gain
        <input
          type="range"
          min="10"
          max="100"
          step="5"
          value={Math.round(gain * 100)}
          onChange={(e) => setGain(Number(e.target.value) / 100)}
        />
        <span>{Math.round(gain * 100)}%<em>{thrustPct(gain)}% thrust</em></span>
      </label>
      <p className="hint">wasd move · q/e yaw · r/f depth · space arm</p>
    </div>
  )
}

// What is actually wrong, in words. Five unlabelled dots told you something was
// off but not which layer, and the layers fail in a fixed order -- no Gazebo
// means no SITL means no MAVLink -- so the FIRST broken one is the only one
// worth acting on. Everything downstream of it is a symptom.
function linkDiagnosis(link = {}, teleop) {
  if (!link.gz) return ['gazebo is not running', 'duburi_sim sim']
  if (!link.sitl) return ['ArduSub SITL is not up', 'restart: duburi_sim sim']
  if (!link.mav) return ['no MAVLink from the manager', 'duburi_sim stack']
  if (!link.cams) return ['camera bridge has no frames', 'check duburi_sim sim log']
  if (!(teleop?.connected ?? link.teleop)) return ['teleop link idle', 'arm or nudge the d-pad']
  return [null, null]
}

function LinkDots({ link = {}, teleop }) {
  const items = [
    ['gz', link.gz],
    ['sitl', link.sitl],
    ['mav', link.mav],
    ['cams', link.cams],
    ['teleop', teleop?.connected ?? link.teleop],
  ]
  const [what, how] = linkDiagnosis(link, teleop)
  return (
    <span className="dot-row">
      {items.map(([k, on]) => (
        <span key={k} className="dot-item">
          <span className={`dot ${on ? 'on' : ''}`} />
          {k}
        </span>
      ))}
      {what && <span className="link-why" title={how}>{what}</span>}
    </span>
  )
}

// Pool floor from duburi_sim_worlds/spec/arena.yaml (surface z=0, depth 1.6).
// SAUVC's floor is a shallow V: 1.6 m at the centre, 1.2 m at both ends. A
// constant -1.6 here overstated altitude by up to 0.4 m near either wall, and
// the operator reads this number to decide whether the vehicle is about to
// ground. RoboSub's pool is flat at 2.1 m.
//
// Kept in sync with duburi_sim_worlds/spec/*.yaml by hand -- the lab has no
// route to the arena spec, so a drift test in the autonomy package asserts
// these numbers still match.
const POOL_PROFILE = {
  sauvc: { length: 25.0, depth: 1.6, edge: 1.2 },
  robosub: { length: 20.0, depth: 2.1, edge: null },
}

function floorDepthAt(course, x) {
  const p = (course || '').startsWith('robosub') || (course || '').startsWith('rs_')
    ? POOL_PROFILE.robosub : POOL_PROFILE.sauvc
  if (!p.edge || !Number.isFinite(x)) return -p.depth
  const t = Math.min(Math.abs(x) / (p.length / 2), 1)
  return -(p.depth - (p.depth - p.edge) * t)
}
// Mirrors ALLOWED_MODES in server.py. Kept short on purpose: SURFACE has its
// own verb and POSHOLD needs DVL/EKF params the sim does not set.
const MODES = ['MANUAL', 'STABILIZE', 'ALT_HOLD', 'ACRO', 'DEPTH_HOLD']

function Operate({ status, refresh }) {
  const st = status?.state || {}
  const gt = status?.ground_truth || {}
  const link = status?.link || {}
  const depthDelta =
    gt?.have && st.depth_m != null && !Number.isNaN(Number(st.depth_m))
      ? Number(st.depth_m) - Number(gt.z)
      : null
  const course = status?.active_course || status?.sim?.active_course || 'sauvc26_qualification'
  const [camMode, setCamMode] = useState('both')
  const [gain, setGain] = useState(1.0)
  const [padEnabled, setPadEnabled] = useState(true)
  const [busy, setBusy] = useState(false)
  const [name, setName] = useState('gate_approach')
  const [recCams, setRecCams] = useState({ front: true, bottom: true })
  const [fx, setFx] = useState(true)
  const [frames, setFrames] = useState(true)
  const [labels, setLabels] = useState(true)
  const [recording, setRecording] = useState(false)
  const [turbidity, setTurbidity] = useState(status?.fx?.turbidity ?? 0.45)
  const [msg, setMsg] = useState('')
  const armedRef = useRef(!!st.armed)

  useEffect(() => {
    armedRef.current = !!st.armed
  }, [st.armed])

  useEffect(() => {
    if (status?.fx?.turbidity != null) setTurbidity(status.fx.turbidity)
  }, [status?.fx?.turbidity])

  useEffect(() => {
    const id = setInterval(() => {
      api('/api/record/status').then((s) => setRecording(!!s.running)).catch(() => {})
    }, 1500)
    return () => clearInterval(id)
  }, [])

  const onArmToggle = useCallback(async () => {
    setBusy(true)
    try {
      if (armedRef.current) await api('/api/vehicle/disarm', { method: 'POST' })
      else await api('/api/vehicle/arm', { method: 'POST' })
      await refresh()
    } catch (e) {
      setMsg(e.message)
    } finally {
      setBusy(false)
    }
  }, [refresh])

  // A pad on the BROWSER machine. Arm/disarm go through the same handlers the
  // on-screen buttons use, so a stick cannot reach the vehicle by a path the
  // operator's own clicks do not.
  const onPadArm = useCallback(() => {
    if (!armedRef.current) onArmToggle()
  }, [onArmToggle])
  const onPadDisarm = useCallback(() => {
    if (armedRef.current) onArmToggle()
  }, [onArmToggle])
  const onPadPayload = useCallback(async (kind, channel) => {
    if (channel === null) {
      setMsg(`${kind}: no rounds left — disarm to reload`)
      return
    }
    setMsg(`${kind} -> channel ${channel} ...`)
    try {
      await api('/api/vehicle/cmd', {
        method: 'POST',
        body: JSON.stringify({ cmd: 'fire', fire_channel: channel }),
      })
      setMsg(`${kind} channel ${channel} away`)
    } catch (e) {
      setMsg(e.message)
    }
  }, [])

  const browserPad = useGamepad({
    enabled: padEnabled,
    gain,
    onArm: onPadArm,
    onDisarm: onPadDisarm,
    onPayload: onPadPayload,
  })

  const onSetMode = useCallback(async (mode) => {
    setBusy(true)
    setMsg(`mode -> ${mode} ...`)
    try {
      await api('/api/vehicle/mode', {
        method: 'POST',
        body: JSON.stringify({ cmd: 'set_mode', target_name: mode }),
      })
      // Read it back rather than trusting the POST: ArduSub can refuse a mode
      // (ALT_HOLD needs a healthy depth source), and the refusal is the thing
      // worth showing.
      const after = await refresh()
      const now = after?.state?.mode
      setMsg(now === mode ? `mode ${mode}` : `asked for ${mode}, vehicle reports ${now || 'unknown'}`)
    } catch (e) {
      setMsg(e.message)
    } finally {
      setBusy(false)
    }
  }, [refresh])

  const triggerZipDownload = (runId) => {
    if (!runId) return
    const a = document.createElement('a')
    a.href = `/api/datasets/${encodeURIComponent(runId)}/zip`
    a.download = `${runId}.zip`
    document.body.appendChild(a)
    a.click()
    a.remove()
  }

  const startRec = async () => {
    const cameras = []
    if (recCams.front) cameras.push('front')
    if (recCams.bottom) cameras.push('bottom')
    if (!cameras.length) {
      setMsg('select at least one camera')
      return
    }
    try {
      await api('/api/record/start', {
        method: 'POST',
        body: JSON.stringify({
          name,
          label: name,
          cameras,
          fx,
          frames,
          labels,
          duration: 0,
          course,
        }),
      })
      setRecording(true)
      setMsg(`recording on ${course}…`)
    } catch (e) {
      setMsg(e.message)
    }
  }

  const stopRec = async () => {
    try {
      const r = await api('/api/record/stop', { method: 'POST' })
      setRecording(false)
      let runId = null
      if (r.record_dir) {
        runId = String(r.record_dir).split('/').filter(Boolean).pop()
        const counts = (r.meta && r.meta.counts) || {}
        const total = Object.values(counts).reduce((a, b) => a + Number(b || 0), 0)
        if (total === 0) {
          setMsg(`saved ${runId} but counts=0 (no frames) — check cams / FX topics`)
        } else {
          const fps = r.meta?.fps_actual
            ? ` · ${Object.values(r.meta.fps_actual).map((v) => Number(v).toFixed(1)).join('/')} fps`
            : ''
          const dur = r.meta?.duration_s != null ? `${Number(r.meta.duration_s).toFixed(1)}s` : ''
          setMsg(`saved ${runId}${dur ? ` (${dur}${fps})` : ''} — downloading zip…`)
          triggerZipDownload(runId)
        }
      } else {
        setMsg('stopped (no record_dir)')
      }
      await refresh()
    } catch (e) {
      setMsg(e.message)
    }
  }

  return (
    <section className="page operate">
      <div className="operate-main">
        <div className="cam-toolbar">
          {['front', 'bottom', 'both'].map((m) => (
            <button
              key={m}
              type="button"
              className={camMode === m ? 'active' : ''}
              onClick={() => setCamMode(m)}
            >
              {m}
            </button>
          ))}
          <LinkDots link={link} teleop={status?.teleop} />
        </div>
        <div className="cam-stage">
          <CamGrid mode={camMode} />
        </div>
      </div>

      <aside className="panel">
        <h2>vehicle</h2>
        <dl className="stats dense">
          <div><dt>armed</dt><dd className={st.armed ? 'on' : ''}>{st.armed ? 'yes' : 'no'}</dd></div>
          <div><dt>mav</dt><dd>{link.mav ? 'live' : 'waiting'}</dd></div>
          <div><dt>depth</dt><dd>{fmtM(st.depth_m)}</dd></div>
          <div><dt>yaw</dt><dd>{fmtDeg(st.yaw_deg)}</dd></div>
          <div><dt>battery</dt><dd>{st.battery_voltage != null ? `${Number(st.battery_voltage).toFixed(1)} V` : '—'}</dd></div>
          <div><dt>alt</dt><dd>{fmtM(gt?.have ? Math.abs(floorDepthAt(st?.active_course, gt.x) - gt.z) : null)}</dd></div>
          {/* course is a single 21-char unbreakable token; in a half-width
              track it set the grid's min-content and forced the whole sidebar
              to overflow. Full width is the honest place for it. */}
          <div className="full"><dt>course</dt><dd>{course}</dd></div>
          <div className="full"><dt>ground truth</dt><dd className="pos">{fmtPos(gt)}</dd></div>
          {/* Sim ground truth vs what the stack believes. This is the exact
              bug class the audit is about: depth is read from AHRS2, which is
              offset from truth by ~0.33 m at the surface, and that offset is
              why surface() never confirms and calibrate_depth refuses. Showing
              the delta makes it visible instead of folklore. */}
          <div className="full">
            <dt>depth vs truth</dt>
            <dd className={`pos ${depthDelta != null && Math.abs(depthDelta) > 0.25 ? 'warn-val' : ''}`}>
              {depthDelta == null
                ? '—'
                : `stack ${Number(st.depth_m).toFixed(2)} · truth ${Number(gt.z).toFixed(2)} · Δ ${depthDelta >= 0 ? '+' : ''}${depthDelta.toFixed(2)} m`}
            </dd>
          </div>
        </dl>

        <h2>mode</h2>
        <div className="row mode-row">
          <select
            value={st.mode && MODES.includes(st.mode) ? st.mode : ''}
            disabled={busy || !link.mav}
            onChange={(e) => e.target.value && onSetMode(e.target.value)}
          >
            <option value="" disabled>{st.mode || 'unknown'}</option>
            {MODES.map((m) => <option key={m} value={m}>{m}</option>)}
          </select>
          <span className="muted">{st.mode || 'unknown'}</span>
        </div>

        <h2>teleop</h2>
        <ControllerPanel
          browserPad={browserPad}
          labPad={status?.joystick}
          enabled={padEnabled}
          setEnabled={setPadEnabled}
        />
        <Dpad
          armed={!!st.armed}
          onArmToggle={onArmToggle}
          gain={gain}
          setGain={setGain}
          busy={busy}
        />

        <h2>record</h2>
        <label>
          name
          <input value={name} onChange={(e) => setName(e.target.value)} placeholder="clip_name" />
        </label>
        <div className="row checks">
          <label><input type="checkbox" checked={recCams.front} onChange={(e) => setRecCams({ ...recCams, front: e.target.checked })} /> front</label>
          <label><input type="checkbox" checked={recCams.bottom} onChange={(e) => setRecCams({ ...recCams, bottom: e.target.checked })} /> bottom</label>
          <label><input type="checkbox" checked={fx} onChange={(e) => setFx(e.target.checked)} /> fx</label>
          <label><input type="checkbox" checked={frames} onChange={(e) => setFrames(e.target.checked)} /> frames</label>
          <label><input type="checkbox" checked={labels} onChange={(e) => setLabels(e.target.checked)} /> labels</label>
        </div>
        <div className="row">
          {!recording ? (
            <button type="button" className="solid" onClick={startRec}>● record</button>
          ) : (
            <button type="button" className="warn" onClick={stopRec}>■ stop + download</button>
          )}
        </div>

        <h2>turbidity</h2>
        <input
          type="range"
          min="0"
          max="2"
          step="0.05"
          value={turbidity}
          onChange={async (e) => {
            const v = Number(e.target.value)
            setTurbidity(v)
            try {
              await api('/api/fx', { method: 'POST', body: JSON.stringify({ turbidity: v }) })
            } catch (err) {
              setMsg(err.message)
            }
          }}
        />
        <span className="muted">{turbidity.toFixed(2)} (0 clear · 2 murky)</span>
        {msg && <p className="msg">{msg}</p>}
      </aside>
    </section>
  )
}

function World({ status, refresh }) {
  const sim = status?.sim || status?.restart || {}
  const [courses, setCourses] = useState([])
  const [course, setCourse] = useState('sauvc26_qualification')
  const [gui, setGui] = useState(true)
  const [stack, setStack] = useState(true)
  const [msg, setMsg] = useState('')
  const [models, setModels] = useState([])
  const [propModel, setPropModel] = useState('sauvc_qual_gate')
  const [propName, setPropName] = useState('gate_a')
  const [px, setPx] = useState(0)
  const [py, setPy] = useState(0)
  const [pz, setPz] = useState(-1.5)
  const [pyaw, setPyaw] = useState(0)
  const [instances, setInstances] = useState([])
  const [scripts, setScripts] = useState([])
  const [scriptId, setScriptId] = useState('')
  const [showUtils, setShowUtils] = useState(false)
  const [listOut, setListOut] = useState('')

  const reloadCatalog = () => {
    api('/api/props/catalog').then((d) => {
      const ms = d.models || []
      setModels(ms)
      if (ms.length && !ms.find((m) => m.id === propModel)) {
        setPropModel(ms[0].id)
      }
    }).catch(() => {})
  }

  const reloadInstances = () => {
    api('/api/props/instances').then((d) => setInstances(d.instances || [])).catch(() => {})
  }

  useEffect(() => {
    api('/api/course').then((d) => {
      setCourses(d.courses || [])
      if (d.active_course) setCourse(d.active_course)
    }).catch(() => {})
    reloadCatalog()
    reloadInstances()
    api('/api/scripts').then((d) => {
      setScripts(d.scripts || [])
      if (d.scripts?.[0]) setScriptId(d.scripts[0].id)
    }).catch(() => {})
  }, [])

  useEffect(() => {
    if (status?.active_course) setCourse(status.active_course)
  }, [status?.active_course])

  useEffect(() => {
    if (!sim.running) return undefined
    const id = setInterval(refresh, 2000)
    return () => clearInterval(id)
  }, [sim.running, refresh])

  const runJob = async (path) => {
    setMsg(`${path.split('/').pop()}…`)
    try {
      await api(path, {
        method: 'POST',
        body: JSON.stringify({ course, gui, stack }),
      })
      setMsg(`${path.split('/').pop()} queued — waiting for ready`)
      refresh()
    } catch (e) {
      setMsg(e.message)
    }
  }

  const phase = sim.phase || 'idle'
  const busy = !!sim.running
  const poseBody = () => ({
    model: propModel,
    name: propName,
    x: px,
    y: py,
    z: pz,
    yaw: pyaw,
  })

  return (
    <section className="page single">
      <div className="panel wide">
        <h2>simulation</h2>
        <div className="row">
          <LinkDots link={status?.link || { gz: status?.gz, sitl: status?.ardusub, mav: status?.state?.have_state }} teleop={status?.teleop} />
        </div>
        <p className="muted">
          phase <strong>{phase}</strong>
          {sim.error ? ` · error: ${sim.error}` : ''}
          {sim.active_course ? ` · active ${sim.active_course}` : ''}
        </p>
        <label>
          course
          <select value={course} onChange={(e) => setCourse(e.target.value)} disabled={busy}>
            {courses.map((c) => (
              <option key={c.id} value={c.id}>{c.id}</option>
            ))}
          </select>
        </label>
        <div className="row checks">
          <label className="inline">
            <input type="checkbox" checked={gui} disabled={busy} onChange={(e) => setGui(e.target.checked)} /> gazebo gui
          </label>
          <label className="inline">
            <input type="checkbox" checked={stack} disabled={busy} onChange={(e) => setStack(e.target.checked)} /> start stack
          </label>
        </div>
        <div className="row">
          <button type="button" className="solid" disabled={busy} onClick={() => runJob('/api/sim/start')}>▶ start</button>
          <button type="button" className="solid" disabled={busy} onClick={() => runJob('/api/sim/restart')}>↻ restart / switch</button>
          <button
            type="button"
            disabled={busy}
            onClick={async () => {
              await api('/api/sim/stop', { method: 'POST' })
              setMsg('stopped')
              refresh()
            }}
          >
            ■ stop
          </button>
        </div>
        {sim.log_tail && <pre className="msg log">{sim.log_tail}</pre>}

        <h2>props</h2>
        <div className="grid2">
          <label>
            model
            <select value={propModel} onChange={(e) => setPropModel(e.target.value)}>
              {(models.length ? models : [{ id: propModel }]).map((m) => (
                <option key={m.id} value={m.id}>
                  {m.id}
                  {m.anchor ? ` (${m.anchor})` : ''}
                  {m.source === 'custom' ? ' · custom' : ''}
                </option>
              ))}
            </select>
          </label>
          <label>name<input value={propName} onChange={(e) => setPropName(e.target.value)} /></label>
          <label>x<input type="number" step="0.1" value={px} onChange={(e) => setPx(Number(e.target.value))} /></label>
          <label>y<input type="number" step="0.1" value={py} onChange={(e) => setPy(Number(e.target.value))} /></label>
          <label>z<input type="number" step="0.1" value={pz} onChange={(e) => setPz(Number(e.target.value))} /></label>
          <label>yaw rad<input type="number" step="0.1" value={pyaw} onChange={(e) => setPyaw(Number(e.target.value))} /></label>
        </div>
        <div className="row">
          <button
            type="button"
            onClick={async () => {
              try {
                await api('/api/props/spawn', {
                  method: 'POST',
                  body: JSON.stringify(poseBody()),
                })
                setMsg(`spawned ${propName}`)
                reloadInstances()
              } catch (e) {
                setMsg(e.message)
              }
            }}
          >
            + spawn
          </button>
          <button
            type="button"
            onClick={async () => {
              try {
                await api('/api/props/move', {
                  method: 'POST',
                  body: JSON.stringify({
                    name: propName,
                    x: px,
                    y: py,
                    z: pz,
                    yaw: pyaw,
                  }),
                })
                setMsg(`moved ${propName}`)
                reloadInstances()
              } catch (e) {
                setMsg(e.message)
              }
            }}
          >
            ↻ move
          </button>
          <button
            type="button"
            onClick={async () => {
              try {
                await api(`/api/props/remove/${encodeURIComponent(propName)}`, { method: 'POST' })
                setMsg(`removed ${propName}`)
                reloadInstances()
              } catch (e) {
                setMsg(e.message)
              }
            }}
          >
            − remove
          </button>
          <button type="button" onClick={reloadInstances}>instances</button>
          <button
            type="button"
            onClick={async () => {
              const r = await api('/api/props/list')
              setListOut(r.stdout || r.stderr || '')
            }}
          >
            catalog
          </button>
        </div>
        {instances.length > 0 && (
          <ul className="instance-list">
            {instances.map((it) => (
              <li key={it.name}>
                <button
                  type="button"
                  className="linkish"
                  onClick={() => {
                    setPropName(it.name)
                    if (it.model) setPropModel(it.model)
                    if (it.x != null) setPx(it.x)
                    if (it.y != null) setPy(it.y)
                    if (it.z != null) setPz(it.z)
                    if (it.yaw != null) setPyaw(it.yaw)
                  }}
                >
                  {it.name}
                </button>
                <span className="muted">
                  {' '}
                  {it.model || '?'} · x={it.x} y={it.y} z={it.z ?? '—'}
                </span>
              </li>
            ))}
          </ul>
        )}
        {listOut && <pre className="msg log">{listOut}</pre>}

        <h2>custom assets</h2>
        <p className="muted">Upload a .zip with model.sdf (+ meshes). May need sim restart for Gazebo to see new models.</p>
        <input
          type="file"
          accept=".zip"
          onChange={async (e) => {
            const f = e.target.files?.[0]
            if (!f) return
            const fd = new FormData()
            fd.append('file', f)
            try {
              const r = await fetch('/api/assets/upload', { method: 'POST', body: fd })
              const d = await r.json()
              if (!r.ok) throw new Error(d.detail || r.statusText)
              setMsg(`uploaded model ${d.model_id}`)
              reloadCatalog()
            } catch (err) {
              setMsg(err.message)
            }
            e.target.value = ''
          }}
        />

        <p className="muted">
          timeseries: <code>ros2 run duburi_sim_bringup duburi_sim plotjuggler</code>
        </p>

        <button type="button" className="linkish" onClick={() => setShowUtils((v) => !v)}>
          {showUtils ? '− utils' : '+ utils (legacy scripts)'}
        </button>
        {showUtils && (
          <div className="utils">
            <select value={scriptId} onChange={(e) => setScriptId(e.target.value)}>
              {scripts.map((s) => (
                <option key={s.id} value={s.id}>{s.id}</option>
              ))}
            </select>
            <button
              type="button"
              onClick={async () => {
                try {
                  await api('/api/scripts/run', {
                    method: 'POST',
                    body: JSON.stringify({ script_id: scriptId }),
                  })
                  setMsg(`script ${scriptId} started`)
                } catch (e) {
                  setMsg(e.message)
                }
              }}
            >
              run script
            </button>
          </div>
        )}
        {msg && <pre className="msg">{msg}</pre>}
      </div>
    </section>
  )
}

function Datasets() {
  const [runs, setRuns] = useState([])

  const load = () => {
    api('/api/datasets').then((d) => setRuns(d.datasets || [])).catch(() => {})
  }

  useEffect(() => {
    load()
    const boot = setTimeout(load, 400)
    const id = setInterval(load, 4000)
    return () => {
      clearTimeout(boot)
      clearInterval(id)
    }
  }, [])

  return (
    <section className="page single">
      <div className="panel wide">
        <div className="panel-head">
          <h2>datasets</h2>
          <button type="button" onClick={load}>↻ refresh</button>
        </div>
        <table>
          <thead>
            <tr>
              <th>name</th>
              <th>course</th>
              <th>started</th>
              <th>duration</th>
              <th>cameras</th>
              <th>position start</th>
              <th>position end</th>
              <th>size</th>
              <th>integrity</th>
              <th />
            </tr>
          </thead>
          <tbody>
            {runs.map((r) => {
              const m = r.meta || {}
              return (
                <tr key={r.id}>
                  <td>
                    <div className="name">{m.label || '—'}</div>
                    <div className="muted mono tiny">{r.id}</div>
                  </td>
                  <td>{m.course || '—'}</td>
                  <td>{fmtWhen(m.utc_start)}</td>
                  <td>{m.duration_s != null ? `${Number(m.duration_s).toFixed(1)} s` : '—'}{m.fps_actual ? ` · ${Object.values(m.fps_actual).map((v) => Number(v).toFixed(1)).join('/')} fps` : ''}</td>
                  <td>{(m.cameras || []).join(', ') || '—'}</td>
                  <td className="mono tiny">{fmtPos(m.gt_start ? { ...m.gt_start, have: true } : null)}</td>
                  <td className="mono tiny">{fmtPos(m.gt_end ? { ...m.gt_end, have: true } : null)}</td>
                  <td>{r.size_mb} MB</td>
                  {/* record_cameras drops png/label writes on a full queue
                      without erroring, so a finished-looking directory can have
                      desynced indices. Surfacing it here catches a bad clip
                      before it is trained on. */}
                  <td>
                    <span
                      className={`badge ${r.integrity?.state || 'unknown'}`}
                      title={r.integrity?.detail || ''}
                    >
                      {r.integrity?.state === 'ok' ? 'ok'
                        : r.integrity?.state === 'mismatch' ? 'MISMATCH' : '?'}
                    </span>
                  </td>
                  <td>
                    <a href={`/api/datasets/${encodeURIComponent(r.id)}/zip`}>zip ↓</a>
                  </td>
                </tr>
              )
            })}
          </tbody>
        </table>
        {!runs.length && <p className="muted">no clips yet — record from operate</p>}
      </div>
    </section>
  )
}


// ---------------------------------------------------------------------------
// Score -- what the run was actually worth
// ---------------------------------------------------------------------------
//
// Every item is one line of the published rulebook, in the rulebook's own
// words, in one of three states: earned, still available, or NOT MODELLED --
// scored by the competition but not judgeable here. The third state is the
// reason the header shows the reachable maximum next to the rulebook maximum.
// A total that quietly counts unreachable points reads like a competition
// result and is not one.

function mmss(s) {
  const t = Math.max(0, Math.round(s || 0))
  return `${Math.floor(t / 60)}:${String(t % 60).padStart(2, '0')}`
}

function ScoreItem({ item }) {
  const dead = item.state === 'not_modelled'
  const got = item.count > 0
  const cls = dead ? 'score-item dead' : got ? 'score-item got' : 'score-item'
  return (
    <li className={cls}>
      <span className="tick">{dead ? '—' : got ? '✓' : '·'}</span>
      <span className="what">
        {item.label}
        {item.repeat > 1 && <em> ×{item.repeat}</em>}
        {dead && <em className="why">not modelled: {item.note}</em>}
        {!dead && item.note && <em className="why">{item.note}</em>}
        {item.evidence.map((e, i) => (
          <em key={i} className="evidence">{e}</em>
        ))}
      </span>
      <span className="pts">
        {got ? `+${item.earned}` : dead ? '' : item.points * item.repeat}
      </span>
    </li>
  )
}

function Score() {
  const [card, setCard] = useState(null)
  const [err, setErr] = useState(null)

  useEffect(() => {
    const load = async () => {
      try {
        setCard(await api('/api/score'))
        setErr(null)
      } catch (e) {
        setErr(String(e.message || e))
      }
    }
    load()
    const id = setInterval(load, 2000)
    return () => clearInterval(id)
  }, [])

  if (err) return <main className="page single"><div className="panel wide"><p className="muted">lab offline — {err}</p></div></main>
  if (!card) return <main className="page single"><div className="panel wide"><p className="muted">waiting for the scorer…</p></div></main>
  if (!card.available) {
    return (
      <main className="page single">
        <div className="panel wide">
          <h2>score</h2>
          <p className="muted">{card.hint}</p>
        </div>
      </main>
    )
  }

  const run = card.run || {}
  const c = card.card || {}
  const pct = c.max_reachable ? Math.round((c.total / c.max_reachable) * 100) : 0
  const remaining = []
  for (const t of c.tasks || []) {
    for (const it of t.items || []) {
      if (it.state === 'scored' && it.count < it.repeat) {
        remaining.push(`${it.label}${it.count ? ` (${it.count}/${it.repeat})` : ''}`)
      }
    }
  }

  return (
    <main className="page single">
      <div className="panel wide">
        <div className="score-head">
          <div>
            <h2>{c.name}</h2>
            <p className="muted mono tiny">{card.competition} · {run.running ? 'RUN LIVE' : 'not running'}</p>
          </div>
          <div className="score-total">
            <strong>{c.total}</strong>
            <em>of {c.max_reachable} reachable</em>
            <em className="tiny">rulebook maximum {c.max_rulebook}</em>
          </div>
        </div>

        <div className="score-bar"><i style={{ width: `${Math.min(100, pct)}%` }} /></div>

        <ul className="score-facts">
          <li><span>clock</span>{mmss(run.elapsed_s)} / {mmss(run.limit_s)}</li>
          <li><span>pool contact</span>{(run.contact_s || 0).toFixed(1)} s · {run.touches || 0} touches</li>
          <li><span>penalties</span>{(c.penalties || []).reduce((a, p) => a + p.points, 0)}</li>
        </ul>
        {run.aborted && <p className="score-abort">RUN ABORTED — {run.aborted}</p>}
      </div>

      {(c.tasks || []).map((t) => (
        <div className="panel wide" key={t.key}>
          <h3>{t.label}</h3>
          <ul className="score-list">
            {t.items.map((it) => <ScoreItem key={it.key} item={it} />)}
          </ul>
        </div>
      ))}

      {(c.penalties || []).length > 0 && (
        <div className="panel wide">
          <h3>penalties</h3>
          <ul className="score-list">
            {c.penalties.map((p, i) => (
              <li className="score-item pen" key={i}>
                <span className="tick">!</span>
                <span className="what">{p.label}<em className="evidence">{p.evidence}</em></span>
                <span className="pts">{p.points}</span>
              </li>
            ))}
          </ul>
        </div>
      )}

      <div className="panel wide">
        <h3>what remains</h3>
        {remaining.length === 0
          ? <p className="muted">everything the sim can score has been scored.</p>
          : <ul className="score-remaining">{remaining.map((r, i) => <li key={i}>{r}</li>)}</ul>}
      </div>
    </main>
  )
}

export default function App() {
  const [page, setPage] = useState('operate')
  const [status, setStatus] = useState(null)

  const refresh = useCallback(async () => {
    try {
      const next = await api('/api/sim/status')
      setStatus(next)
      // Returned so callers can read the vehicle's ANSWER instead of assuming
      // their command took (setStatus is async, so reading state after await
      // refresh() would still see the old value).
      return next
    } catch {
      /* offline */
      return null
    }
  }, [])

  useEffect(() => {
    refresh()
    const id = setInterval(refresh, 2000)
    return () => clearInterval(id)
  }, [refresh])

  return (
    <div className="app">
      <header>
        <div className="brand">
          <img src="/ue-logo.png" alt="unauthorized engineering" className="logo" />
          <div>
            <div className="title">unauthorized engineering</div>
            <div className="sub">mongla · duburi sim lab</div>
          </div>
        </div>
        <nav>
          {PAGES.map((p) => (
            <button
              key={p}
              type="button"
              className={page === p ? 'active' : ''}
              onClick={() => setPage(p)}
            >
              {p}
            </button>
          ))}
        </nav>
        <div className="tagline">build. break. learn. repeat.</div>
      </header>
      {page === 'operate' && <Operate status={status} refresh={refresh} />}
      {page === 'score' && <Score />}
      {page === 'world' && <World status={status} refresh={refresh} />}
      {page === 'datasets' && <Datasets />}
      <footer>
        <span>air | water | ground</span>
        <span>autonomy</span>
      </footer>
    </div>
  )
}
