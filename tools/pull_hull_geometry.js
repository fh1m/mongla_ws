// Re-read the hull's thruster geometry from Onshape, in the browser.
//
// ⛔ WHY THIS IS A TOOL AND NOT A ONE-OFF. The CAD is a moving design: custom
// thrusters are being designed, not all of them are fitted yet, and materials
// are not assigned. `hull_geometry.yaml` is therefore a SNAPSHOT, and a
// hand-maintained snapshot of a moving design goes stale silently. Re-run this
// whenever the CAD changes and regenerate the file rather than editing numbers
// by hand.
//
// HOW TO RUN IT. Open the Part Studio in Chrome, signed in, then paste this in
// the console (or run it through the browser automation tool). It uses the
// page's own session: no API key, no OAuth app, and it does NOT consume the
// FeatureScript MCP's API allocation.
//
// ⚠ WHY NOT THE ONSHAPE MCP. That server exposes FeatureScript authoring:
// `test_featurescript` runs in a scratch document and takes no document id, and
// this document holds no Feature Studios. It cannot read part geometry. The
// browser can, because the page is already authenticated.
//
// ⚠ WHY BOUNDING BOXES AND NOT MASS PROPERTIES. `/massproperties` returns
// `hasMass:false`, `massMissingCount:1` and a centroid of exactly [0,0,0] for
// every part, because no part has a material assigned. A zero centroid is not a
// measurement -- it is the absence of one, and it would put the whole vehicle at
// the origin. Bounding boxes are pure geometry and need no material.
// ⭐ WHEN MATERIALS ARE ASSIGNED, switch to /massproperties: it gives true
// centroids, the total mass, and the inertia tensor -- which unblocks the entire
// force domain (BENCH B-5) and the 6x6 mass-inertia matrix INDI needs.

const DOC = {
  d: '7254e77ceff30ba82f72f408',
  w: '1ac100c7c4a0660307476d72',
  e: '290dbcab1f90336f7e9206f6',
};

const j = async (p) =>
  (await fetch(p, { headers: { Accept: 'application/json' }, credentials: 'include' })).json();

const mm = (v) => +(v * 1000).toFixed(1);

async function pull({ d, w, e } = DOC) {
  const parts = await j(`/api/parts/d/${d}/w/${w}/e/${e}`);
  const rows = [];
  for (const p of parts) {
    const b = await j(
      `/api/parts/d/${d}/w/${w}/e/${e}/partid/${encodeURIComponent(p.partId)}/boundingboxes`);
    rows.push({
      name: p.name,
      id: p.partId,
      centre_mm: [mm((b.lowX + b.highX) / 2), mm((b.lowY + b.highY) / 2), mm((b.lowZ + b.highZ) / 2)],
      size_mm: [mm(b.highX - b.lowX), mm(b.highY - b.lowY), mm(b.highZ - b.lowZ)],
    });
  }

  // A tunnel is identified by its BORE, and its axis by its longest extent --
  // a tunnel is a tube, so the long dimension is the direction it pushes.
  const BORE_MM = 84;
  const tunnels = rows.filter((r) => r.size_mm.some((v) => Math.abs(v - BORE_MM) < 3));

  // ⚠ A MOTOR IS IDENTIFIED BY POSITION, NOT BY SHAPE. A brushless can's
  // bounding box includes its shaft and mount, so its longest extent does NOT
  // reliably give its axis: the axial unit's box is [27.8, 40, 43.9], whose
  // largest value points the wrong way. What identifies it is that it sits on
  // the long axis with no tunnel near it. Motors that ARE near a tunnel are
  // that tunnel's drive, not a thruster in their own right.
  const motors = rows.filter((r) => /A2212|motor|thrust/i.test(r.name));
  const nearTunnel = (m) =>
    tunnels.some((t) => Math.hypot(...m.centre_mm.map((v, i) => v - t.centre_mm[i])) < 60);
  const standalone = motors.filter((m) => !nearTunnel(m));

  return {
    // ⚠ Filter non-finite first. At least one part in this document returns a
    // bounding box that does not reduce to a number, and a single NaN
    // propagates through Math.max to make the whole extent null -- which reads
    // as "no hull" rather than as "one bad part".
    hull_extent_mm: [0, 1, 2].map((i) => {
      const hi = rows.map((r) => r.centre_mm[i] + r.size_mm[i] / 2).filter(Number.isFinite);
      const lo = rows.map((r) => r.centre_mm[i] - r.size_mm[i] / 2).filter(Number.isFinite);
      return hi.length ? +(Math.max(...hi) - Math.min(...lo)).toFixed(1) : null;
    }),
    tunnels: tunnels.map((t) => ({
      part: t.name,
      centre_mm: t.centre_mm,
      size_mm: t.size_mm,
      axis: [0, 1, 2].map((i) => (t.size_mm.indexOf(Math.max(...t.size_mm)) === i ? 1 : 0)),
    })),
    motors_standalone: standalone,   // candidates for the axial unit
    motors_on_tunnels: motors.filter(nearTunnel),
    n_parts: rows.length,
    // ⚠ Check this every time. If it is ever true, STOP using bounding boxes and
    // switch to /massproperties -- and re-open BENCH B-5, because mass is then
    // computable and every force-domain row unblocks.
    materials_assigned: await j(
      `/api/partstudios/d/${d}/w/${w}/e/${e}/massproperties?massAsGroup=true`)
      .then((m) => !!m.hasMass).catch(() => null),
    all: rows,
  };
}

// Paste the result into `src/mongla_control/config/hull_geometry.yaml`, keeping
// its provenance header honest: say which microversion it came from and when.
await pull();
