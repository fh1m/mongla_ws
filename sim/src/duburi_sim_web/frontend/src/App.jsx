import { useCallback, useEffect, useRef, useState } from 'react'

const PAGES = ['operate', 'world', 'datasets']

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
      yaw: 0,
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
        {btn('up', '⬆ up', 'heave-btn')}
        {btn('dn', '⬇ dn', 'heave-btn')}
      </div>
      <label className="gain">
        gain
        <input
          type="range"
          min="0.15"
          max="1"
          step="0.05"
          value={gain}
          onChange={(e) => setGain(Number(e.target.value))}
        />
        <span>{gain.toFixed(2)}</span>
      </label>
      <p className="hint">wasd · r/f depth · space arm</p>
    </div>
  )
}

function LinkDots({ link = {}, teleop }) {
  const items = [
    ['gz', link.gz],
    ['sitl', link.sitl],
    ['mav', link.mav],
    ['cams', link.cams],
    ['teleop', teleop?.connected ?? link.teleop],
  ]
  return (
    <span className="dot-row">
      {items.map(([k, on]) => (
        <span key={k} className="dot-item">
          <span className={`dot ${on ? 'on' : ''}`} />
          {k}
        </span>
      ))}
    </span>
  )
}

function Operate({ status, refresh }) {
  const st = status?.state || {}
  const gt = status?.ground_truth || {}
  const link = status?.link || {}
  const course = status?.active_course || status?.sim?.active_course || 'sauvc26_qualification'
  const [camMode, setCamMode] = useState('both')
  const [gain, setGain] = useState(0.55)
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
          <div><dt>mode</dt><dd>{st.mode || '—'}</dd></div>
          <div><dt>depth</dt><dd>{fmtM(st.depth_m)}</dd></div>
          <div><dt>yaw</dt><dd>{fmtDeg(st.yaw_deg)}</dd></div>
          <div><dt>battery</dt><dd>{st.battery_voltage != null ? `${Number(st.battery_voltage).toFixed(1)} V` : '—'}</dd></div>
          <div className="full"><dt>position</dt><dd className="pos">{fmtPos(gt)}</dd></div>
          <div><dt>course</dt><dd>{course}</dd></div>
          <div><dt>mav link</dt><dd>{link.mav ? 'live' : 'waiting'}</dd></div>
        </dl>

        <h2>teleop</h2>
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

export default function App() {
  const [page, setPage] = useState('operate')
  const [status, setStatus] = useState(null)

  const refresh = useCallback(async () => {
    try {
      setStatus(await api('/api/sim/status'))
    } catch {
      /* offline */
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
      {page === 'world' && <World status={status} refresh={refresh} />}
      {page === 'datasets' && <Datasets />}
      <footer>
        <span>air | water | ground</span>
        <span>autonomy</span>
      </footer>
    </div>
  )
}
