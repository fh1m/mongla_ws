/* Mongla hull viewer — WebGL2, no dependencies, no build step.
 *
 * Reads the .mgla container written by tools/pack_hull.py:
 *   'MGLA' | u16 version | u16 indexBytes | u32 verts | u32 tris
 *   f32 centre[3] | f32 scale | int16 pos[3]*verts | uint16|uint32 idx[3]*tris
 *
 * Positions are quantised to the model's bounding sphere, so a vertex costs
 * 6 bytes and no normal is stored — the facet normal is recovered in the
 * fragment shader from screen-space derivatives. On a CAD mesh that is not a
 * compromise: it gives exactly the crisp facet-and-edge reading a machined
 * part should have, and it is the reason the full 0.9 mm tessellation fits.
 */
(function () {
  'use strict';

  const MAX_PARTS = 64;

  const VERT = `#version 300 es
  in vec3 p;
  in vec3 c;            /* the part's own material colour, straight from the CAD */
  in float part;        /* which CAD body this vertex belongs to */
  uniform mat4 mvp, mv;
  uniform vec3 uDir[${MAX_PARTS}];   /* unit explode direction per body */
  uniform float uExplode;                 /* 0 assembled, 1 fully apart */
  uniform float uFocus;                   /* -1 none, else the part to keep lit */
  out vec3 vE;
  out vec3 vO;
  out vec3 vC;
  out float vDim;
  void main(){
    vec3 d = uDir[int(part + 0.5)];
    vec3 q = p + d * uExplode;
    vO = p; vC = c;
    vDim = (uFocus < 0.0 || abs(uFocus - part) < 0.5) ? 1.0 : 0.18;
    vec4 e = mv * vec4(q, 1.0);
    vE = e.xyz;
    gl_Position = mvp * vec4(q, 1.0);
  }`;

  /* A red machine lit by blue water. One key light, one ocean rim, one depth
     cue — and the depth cue is the whole argument: range is what water takes. */
  /* The hull is drawn in the livery it is actually built in — red fairings over a
     grey pressure can — and the water is what lights it. Key light warm, rim and
     shadow-side bounce in ocean blue, and range taken away with depth, because
     losing range to water is the whole problem this vehicle exists inside. */
  const FRAG = `#version 300 es
  precision highp float;
  in vec3 vE; in vec3 vO; in vec3 vC; in float vDim; out vec4 o;
  uniform vec3 uRim, uAmbient;
  uniform float uNear, uFar, uLivery;

  void main(){
    vec3 n = normalize(cross(dFdx(vE), dFdy(vE)));   /* facet normal, free */
    vec3 v = normalize(-vE);
    if (dot(n, v) < 0.0) n = -n;

    vec3  key   = normalize(vec3(0.40, 0.80, 0.45));
    vec3  fill  = normalize(vec3(-0.55, -0.25, 0.32));
    float lam   = max(dot(n, key), 0.0);
    float bounce= max(dot(n, fill), 0.0);

    vec3  h    = normalize(key + v);
    float spec = pow(max(dot(n, h), 0.0), 68.0) * 0.42;
    float fres = pow(1.0 - max(dot(n, v), 0.0), 2.7);

    float depth = clamp((-vE.z - uNear) / max(uFar - uNear, 1e-4), 0.0, 1.0);

    vec3 albedo = mix(vec3(0.62, 0.10, 0.06), vC, uLivery);
    vec3 c = albedo * (uAmbient + 0.90 * lam * lam);
    c += uRim * bounce * 0.26 * albedo;              /* water on the shadow side */
    c += uRim * fres * 0.62;                         /* the silhouette lights up  */
    c += vec3(0.58, 0.74, 1.0) * spec;               /* highlights carry the water */
    c *= mix(1.08, 0.22, depth);
    c *= 1.0 + 0.035 * sin(vO.z * 46.0 + vO.y * 9.0);

    o = vec4(c * vDim, 1.0);
  }`;

  function sh(gl, t, src) {
    const s = gl.createShader(t);
    gl.shaderSource(s, src); gl.compileShader(s);
    if (!gl.getShaderParameter(s, gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(s));
    return s;
  }
  function mul(a, b) {
    const r = new Float32Array(16);
    for (let i = 0; i < 4; i++) for (let j = 0; j < 4; j++) {
      let s = 0; for (let k = 0; k < 4; k++) s += a[k * 4 + j] * b[i * 4 + k];
      r[i * 4 + j] = s;
    }
    return r;
  }
  const ident = () => new Float32Array([1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1]);
  const persp = (fovy, asp, n, f) => {
    const t = 1 / Math.tan(fovy / 2);
    return new Float32Array([t/asp,0,0,0, 0,t,0,0, 0,0,(f+n)/(n-f),-1, 0,0,2*f*n/(n-f),0]);
  };
  const rotY = a => { const c=Math.cos(a), s=Math.sin(a);
    return new Float32Array([c,0,-s,0, 0,1,0,0, s,0,c,0, 0,0,0,1]); };
  const rotX = a => { const c=Math.cos(a), s=Math.sin(a);
    return new Float32Array([1,0,0,0, 0,c,s,0, 0,-s,c,0, 0,0,0,1]); };
  const trans = (x,y,z) => { const m = ident(); m[12]=x; m[13]=y; m[14]=z; return m; };
  const hex = h => [parseInt(h.slice(1,3),16)/255, parseInt(h.slice(3,5),16)/255, parseInt(h.slice(5,7),16)/255];

  function parse(buf) {
    const dv = new DataView(buf);
    const magic = String.fromCharCode(dv.getUint8(0), dv.getUint8(1), dv.getUint8(2), dv.getUint8(3));
    if (magic !== 'MGLA') throw new Error('not a hull container: ' + magic);
    const idxBytes = dv.getUint16(6, true);
    const nv = dv.getUint32(8, true), nt = dv.getUint32(12, true);
    const version = dv.getUint16(4, true);
    let off = 16 + 12 + 4;                       /* header + centre[3] + scale */
    let dirs = null, parts = null, nParts = 0;
    if (version >= 3) {
      nParts = dv.getUint16(off, true); off += 2;
      dirs = new Float32Array(buf.slice(off, off + nParts * 12)); off += nParts * 12;
    }
    const pos = new Int16Array(buf, off, nv * 3); off += nv * 6;
    let col = null;
    if (version >= 2) { col = new Uint8Array(buf, off, nv * 3); off += nv * 3; }
    if (version >= 3) { parts = new Uint8Array(buf, off, nv); off += nv; }
    const idx = idxBytes === 2 ? new Uint16Array(buf, off, nt * 3)
                               : new Uint32Array(buf, off, nt * 3);
    return { pos, col, parts, dirs, nParts, idx, nv, nt, version };
  }

  window.MonglaHull = function (canvas, url, opts) {
    opts = opts || {};
    const gl = canvas.getContext('webgl2', { antialias: true, alpha: false, powerPreference: 'high-performance' });
    if (!gl) return Promise.reject(new Error('WebGL2 unavailable'));

    const prog = gl.createProgram();
    gl.attachShader(prog, sh(gl, gl.VERTEX_SHADER, VERT));
    gl.attachShader(prog, sh(gl, gl.FRAGMENT_SHADER, FRAG));
    gl.linkProgram(prog);
    if (!gl.getProgramParameter(prog, gl.LINK_STATUS)) throw new Error(gl.getProgramInfoLog(prog));
    gl.useProgram(prog);

    const U = n => gl.getUniformLocation(prog, n);
    const uMVP = U('mvp'), uMV = U('mv'), uNear = U('uNear'), uFar = U('uFar');
    gl.uniform3fv(U('uRim'), hex(opts.rim || '#004eff'));
    const uExplode = U('uExplode'), uFocus = U('uFocus'), uDir = U('uDir');
    gl.uniform1f(uExplode, 0); gl.uniform1f(uFocus, -1);
    gl.uniform3fv(U('uAmbient'), opts.ambient || [0.055, 0.075, 0.125]);
    gl.uniform1f(U('uLivery'), opts.livery == null ? 1.0 : opts.livery);

    let yaw = opts.yaw != null ? opts.yaw : -0.7;
    let pitch = opts.pitch != null ? opts.pitch : 0.26;
    let spin = true, visible = true, raf = 0;
    let explodeT = 0, focusId = -1, explodeShown = 0;
    const reduce = matchMedia('(prefers-reduced-motion: reduce)').matches;
    if (reduce) spin = false;

    return fetch(url)
      .then(r => { if (!r.ok) throw new Error('hull ' + r.status); return r.arrayBuffer(); })
      .then(buf => {
        const m = parse(buf);

        gl.bindVertexArray(gl.createVertexArray());
        const vb = gl.createBuffer();
        gl.bindBuffer(gl.ARRAY_BUFFER, vb);
        gl.bufferData(gl.ARRAY_BUFFER, m.pos, gl.STATIC_DRAW);
        const loc = gl.getAttribLocation(prog, 'p');
        gl.enableVertexAttribArray(loc);
        gl.vertexAttribPointer(loc, 3, gl.SHORT, true, 0, 0);   /* normalised int16 */

        if (m.parts) {
          const pb = gl.createBuffer();
          gl.bindBuffer(gl.ARRAY_BUFFER, pb);
          gl.bufferData(gl.ARRAY_BUFFER, m.parts, gl.STATIC_DRAW);
          const ploc = gl.getAttribLocation(prog, 'part');
          gl.enableVertexAttribArray(ploc);
          gl.vertexAttribPointer(ploc, 1, gl.UNSIGNED_BYTE, false, 0, 0);
          /* the direction table is small enough to live in uniforms; indexing a
             uniform array by an attribute is legal in GLSL ES 3.00 */
          const pad = new Float32Array(64 * 3);
          pad.set(m.dirs.subarray(0, Math.min(m.dirs.length, 64 * 3)));
          gl.uniform3fv(uDir, pad);
        } else {
          gl.disableVertexAttribArray(gl.getAttribLocation(prog, 'part'));
          gl.vertexAttrib1f(gl.getAttribLocation(prog, 'part'), 0);
          gl.uniform3fv(uDir, new Float32Array(64 * 3));
        }

        if (m.col) {
          const cb = gl.createBuffer();
          gl.bindBuffer(gl.ARRAY_BUFFER, cb);
          gl.bufferData(gl.ARRAY_BUFFER, m.col, gl.STATIC_DRAW);
          const cloc = gl.getAttribLocation(prog, 'c');
          gl.enableVertexAttribArray(cloc);
          gl.vertexAttribPointer(cloc, 3, gl.UNSIGNED_BYTE, true, 0, 0);
        } else {
          gl.disableVertexAttribArray(gl.getAttribLocation(prog, 'c'));
          gl.vertexAttrib3f(gl.getAttribLocation(prog, 'c'), 0.78, 0.12, 0.08);
        }

        const ib = gl.createBuffer();
        gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, ib);
        gl.bufferData(gl.ELEMENT_ARRAY_BUFFER, m.idx, gl.STATIC_DRAW);
        const idxType = m.idx.BYTES_PER_ELEMENT === 2 ? gl.UNSIGNED_SHORT : gl.UNSIGNED_INT;

        gl.enable(gl.DEPTH_TEST);
        gl.enable(gl.CULL_FACE); gl.cullFace(gl.BACK);
        gl.clearColor(0, 0, 0, 1);

        function size() {
          /* the offline render rig drives the canvas at a fixed resolution;
             leave its backing store alone or it collapses to the CSS size */
          if (opts.fixed) { gl.viewport(0, 0, canvas.width, canvas.height);
                            return canvas.width / canvas.height; }
          const dpr = Math.min(devicePixelRatio || 1, 2);
          const w = Math.max(1, Math.round(canvas.clientWidth * dpr));
          const h = Math.max(1, Math.round(canvas.clientHeight * dpr));
          if (canvas.width !== w || canvas.height !== h) { canvas.width = w; canvas.height = h; }
          gl.viewport(0, 0, canvas.width, canvas.height);
          return w / h;
        }

        const DIST = 2.24;
        /* dolly out as the hull opens, so the parts stay in frame and the whole
           thing reads as one move rather than as pieces escaping */
        const distFor = t => DIST * (1 + 0.58 * t);
        let last = performance.now();
        function drawOnce() {
          /* ease toward the target so a scroll jump does not snap the hull */
          explodeShown += (explodeT - explodeShown) * 0.14;
          gl.uniform1f(uExplode, explodeShown * 0.34);
          gl.uniform1f(uFocus, focusId);
          const asp = size();
          const dist = distFor(explodeShown);
          const mv = mul(trans(0, 0, -dist), mul(rotX(pitch), rotY(yaw)));
          gl.uniformMatrix4fv(uMV, false, mv);
          gl.uniformMatrix4fv(uMVP, false, mul(persp(0.60, asp, 0.1, 18), mv));
          gl.uniform1f(uNear, dist - 0.95);
          gl.uniform1f(uFar,  dist + 1.20 + explodeShown * 0.9);
          gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
          gl.drawElements(gl.TRIANGLES, m.nt * 3, idxType, 0);
        }
        function draw(t) {
          raf = requestAnimationFrame(draw);
          if (!visible) { last = t; return; }
          const dt = Math.min((t - last) / 1000, 0.05); last = t;
          if (spin) yaw += dt * 0.24;
          drawOnce();
        }
        raf = requestAnimationFrame(draw);

        /* a canvas spinning where nobody can see it is just heat */
        if ('IntersectionObserver' in window) {
          new IntersectionObserver(es => { visible = es[0].isIntersecting; },
            { threshold: 0.01 }).observe(canvas);
        }

        let drag = null;
        canvas.addEventListener('pointerdown', e => {
          drag = { x: e.clientX, y: e.clientY }; spin = false;
          canvas.setPointerCapture(e.pointerId);
        });
        canvas.addEventListener('pointermove', e => {
          if (!drag) return;
          yaw   += (e.clientX - drag.x) * 0.0075;
          pitch  = Math.max(-1.25, Math.min(1.25, pitch + (e.clientY - drag.y) * 0.0060));
          drag = { x: e.clientX, y: e.clientY };
        });
        const up = () => { drag = null; };
        canvas.addEventListener('pointerup', up);
        canvas.addEventListener('pointercancel', up);
        canvas.addEventListener('keydown', e => {
          const k = e.key;
          if (k === 'ArrowLeft')  { spin = false; yaw -= 0.14; e.preventDefault(); }
          if (k === 'ArrowRight') { spin = false; yaw += 0.14; e.preventDefault(); }
          if (k === 'ArrowUp')    { spin = false; pitch = Math.max(-1.25, pitch - 0.09); e.preventDefault(); }
          if (k === 'ArrowDown')  { spin = false; pitch = Math.min( 1.25, pitch + 0.09); e.preventDefault(); }
          if (k === ' ')          { spin = !spin; e.preventDefault(); }
        });

        return {
          triangles: m.nt, vertices: m.nv, parts: m.nParts,
          /* point the camera and draw exactly one frame, for the render rig */
          pose(y, p_) { yaw = y; pitch = p_; },
          frame() { drawOnce(); },
          /* 0 assembled, 1 fully apart. Scroll drives this. */
          explode(t) { explodeT = Math.max(0, Math.min(1, t)); },
          /* keep one CAD body lit and dim the rest; -1 restores all */
          focus(id) { focusId = id == null ? -1 : id; },
          spin(on) { spin = !!on; },
          stop() { cancelAnimationFrame(raf); }
        };
      });
  };
})();
