#!/usr/bin/env python3
"""Which tracker closes more gaps on REAL competition footage?

The question this settles, with numbers rather than an argument: the coast
layer exists to bridge detection gaps, and nobody has ever measured how much
of a real gap any tracker actually bridges.

Deterministic by construction. Inference runs ONCE and the detections are
cached; every tracker then sees byte-identical input. No camera, no GIL, no
dropped frames -- so the tracker is the only variable, which is exactly what
a rosbag replay cannot give (`ros2 bag play` is wall-clock paced and the
pipeline drops a different frame subset each run).

    # cache the detections once
    python3 tools/studies/tracker_ab.py --video ... --model ... --class bin_shaw_fish \
        --conf 0.15 --cache /tmp/bin.npz

    # then compare, as many times as you like, on identical input
    python3 tools/studies/tracker_ab.py --cache /tmp/bin.npz --compare

What is reported is the CONTINUITY report from `mongla_vision.continuity` --
the same ruler used for the raw detector -- so "how much did tracking buy" is
answered in the units the lock ladder is written in.
"""
import argparse
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..',
                                'src', 'mongla_vision'))

from mongla_vision.continuity import Obs, analyse    # noqa: E402


def packet_times(video):
    """Each frame's presentation time, from the container's packets.

    OpenCV derives both CAP_PROP_FPS and CAP_PROP_POS_MSEC from the stream's
    nominal rate, and a variable-frame-rate recording (every Cockpit clip)
    declares its 1000 Hz timebase there: gate.mkv is 6142 frames over
    191.8 s, 32 fps, and was analysed as 1000. None when ffprobe is absent.
    """
    import shutil, subprocess
    if not shutil.which('ffprobe'):
        return None
    out = subprocess.run(['ffprobe', '-v', 'error', '-select_streams', 'v:0',
                          '-show_entries', 'packet=pts_time', '-of', 'csv=p=0',
                          video], capture_output=True, text=True).stdout
    t = np.sort(np.array([float(x) for x in out.split() if x not in ('', 'N/A')]))
    # Non-decreasing, not strictly: a recorder glitch repeats a stamp (gate.mkv
    # has 155.07 s four times), which is a zero interval, not a bad file.
    return t if len(t) > 1 and np.all(np.diff(t) >= 0) else None


def cache_detections(a):
    """Run the detector ONCE over the clip; store every box for one class."""
    import cv2
    from ultralytics import YOLO

    model = YOLO(a.model)
    names = model.names if isinstance(model.names, dict) else dict(
        enumerate(model.names))
    cap = cv2.VideoCapture(a.video)
    fps = a.fps or cap.get(cv2.CAP_PROP_FPS) or 30.0
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    if a.start_frame:
        cap.set(cv2.CAP_PROP_POS_FRAMES, a.start_frame)

    rows = []          # frame_idx, x1, y1, x2, y2, score
    # Each frame's own timestamp. A variable-frame-rate recording (every
    # Cockpit clip) reports its 1000 Hz timebase as CAP_PROP_FPS, and dividing
    # by it put every gap and coast window on a timescale 31x wrong.
    idx = a.start_frame
    kept = 0
    t0 = time.monotonic()
    while True:
        ok, frame = cap.read()
        if not ok or (a.max_frames and kept >= a.max_frames):
            break
        r = model.predict(frame, conf=a.conf, imgsz=a.imgsz, verbose=False)[0]
        for b in r.boxes:
            if names.get(int(b.cls[0])) != a.klass:
                continue
            x1, y1, x2, y2 = (float(v) for v in b.xyxy[0])
            rows.append([idx, x1, y1, x2, y2, float(b.conf[0])])
        idx += 1
        kept += 1
        if kept % 1000 == 0:
            print(f'    ... {kept} frames', flush=True)
    cap.release()
    t = None if a.fps else packet_times(a.video)
    if t is not None and len(t) >= a.start_frame + kept:
        t = t[a.start_frame:a.start_frame + kept]
    else:
        if not a.fps:
            print(f'  WARNING: no usable packet times; using {fps:.1f} fps '
                  f'(pass --fps if the container lies)')
        t = np.arange(kept) / fps
    np.savez(a.cache, rows=np.array(rows, dtype=np.float64), t=t - t[0] if len(t) else t,
             meta=np.array([fps, w, h, a.start_frame, kept, a.conf]))
    print(f'  cached {len(rows)} boxes over {kept} frames in '
          f'{time.monotonic() - t0:.0f}s -> {a.cache}')


def _obs_from(frames, w, h, fps, start, n, ts=None):
    """Detector-only observations: largest box per frame, as `bbox_error` picks."""
    out = []
    for i in range(start, start + n):
        best = None
        for (x1, y1, x2, y2, sc) in frames.get(i, ()):
            area = (x2 - x1) * (y2 - y1)
            if best is None or area > best[0]:
                best = (area, (x1 + x2) / 2 / w, (y1 + y2) / 2 / h, sc)
        out.append(Obs(t=(ts[i - start] if ts is not None else i / fps), seen=best is not None,
                       score=best[3] if best else 0.0,
                       cx=best[1] if best else float('nan'),
                       cy=best[2] if best else float('nan')))
    return out


def _track(frames, w, h, fps, start, n, make_tracker, needs_frame=False, ts=None):
    """Feed the cached detections through a tracker; a coasted box counts as
    SEEN, which is the whole point -- that is what the control loop steers on
    when `coast_s > 0`."""
    import supervision as sv
    tr = make_tracker()
    out = []
    locked = -1      # the id the vehicle's align holds (motion_vision/align.py)
    for i in range(start, start + n):
        boxes = frames.get(i, [])
        if boxes:
            xyxy = np.array([[b[0], b[1], b[2], b[3]] for b in boxes])
            conf = np.array([b[4] for b in boxes])
            cid = np.zeros(len(boxes), dtype=int)
        else:
            xyxy = np.empty((0, 4)); conf = np.empty(0); cid = np.empty(0, int)
        dets = sv.Detections(xyxy=xyxy, confidence=conf, class_id=cid)
        if hasattr(tr, 't') or type(tr).__name__ == '_Shim':
            type(tr).t = (ts[i - start] if ts is not None else i / fps)
        try:
            res = tr.update(dets)
        except TypeError:
            # BoT-SORT wants the frame for optical-flow CMC. We have no frame
            # here by design -- the cache is the point -- so it is compared
            # WITHOUT CMC and that is stated rather than silently assumed.
            res = tr.update(dets, frame=np.zeros((h, w, 3), np.uint8))
        best = None
        if isinstance(res, list):          # our wrapper: TrackedDetection[]
            # Selected as the vehicle selects: stay on the locked id while it
            # is published; otherwise the largest box, and lock on it. Taking
            # the largest every frame flips between two live tracks the
            # vehicle never jumps between, and counts that as a switch.
            held = [td for td in res if td.track_id == locked]
            for td in (held or res):
                x1, y1, x2, y2 = td.xyxy
                area = (x2 - x1) * (y2 - y1)
                if best is None or area > best[0]:
                    best = (area, (x1 + x2) / 2 / w, (y1 + y2) / 2 / h,
                            td.score, td.track_id)
            if best is not None and not held:
                locked = best[4]
        else:                              # the bare library: sv.Detections
            for j in range(len(res.xyxy)):
                x1, y1, x2, y2 = res.xyxy[j]
                area = (x2 - x1) * (y2 - y1)
                if best is None or area > best[0]:
                    tid = (int(res.tracker_id[j])
                           if res.tracker_id is not None else -1)
                    sc = (float(res.confidence[j])
                          if res.confidence is not None else 0.0)
                    best = (area, (x1 + x2) / 2 / w, (y1 + y2) / 2 / h, sc, tid)
        out.append(Obs(t=(ts[i - start] if ts is not None else i / fps), seen=best is not None,
                       score=best[3] if best else 0.0,
                       cx=best[1] if best else float('nan'),
                       cy=best[2] if best else float('nan'),
                       track_id=best[4] if best else -1))
    return out


def compare(a):
    z = np.load(a.cache)
    rows, meta = z['rows'], z['meta']
    ts = z['t'] if 't' in z.files else None
    fps, w, h, start, n, conf = (meta[0], int(meta[1]), int(meta[2]),
                                 int(meta[3]), int(meta[4]), meta[5])
    # The vehicle's two bars: a box ACTS (and may start a track) at `act`;
    # the tracker also sees candidates down to `assoc` (vision_pi defaults).
    act, assoc = a.act or float(conf), a.assoc or float(conf)
    every = {}
    for r in rows:
        every.setdefault(int(r[0]), []).append(tuple(r[1:]))
    frames = {i: [b for b in bs if b[4] >= assoc] for i, bs in every.items()}
    acting = {i: [b for b in bs if b[4] >= act] for i, bs in every.items()}
    print(f'  acting bar {act:.2f}, association floor {assoc:.2f}')
    print(f'\n  {n} frames @ {fps:.0f} fps, {len(rows)} boxes cached at '
          f'conf={conf:.2f}\n')

    # The rate the trackers are told is the clip's MEASURED rate, from the
    # frame times, never the container's nominal one (1000 on Cockpit clips).
    hz = (float(1.0 / np.median(np.diff(ts))) if ts is not None and len(ts) > 1
          else float(fps))
    print(f'  measured rate {hz:.1f} Hz')
    # OUR WRAPPER, not the raw library. `RoboflowTracker.update()` adds the
    # COASTED rows out of `tracked_objects` on top of whatever the library
    # returns -- which is the entire coast layer. Comparing the bare library
    # would measure something we do not ship, and would have reported that
    # tracking buys nothing when the wrapper is what does the work.
    from mongla_vision.tracking.roboflow_tracker import RoboflowTracker
    from mongla_vision.detection.detector import Detection

    def _wrapped(ttype, min_hits=3, max_predict_s=None):
        def make():
            tr = RoboflowTracker(tracker_type=ttype, track_buffer=150,
                                 min_hits=min_hits, detector_conf=act,
                                 iou_threshold=0.2,
                                 track_activation_threshold=0.40)

            class _Shim:
                def update(self, dets, frame=None):
                    ds = [Detection(class_id=0, class_name='t',
                                    score=float(dets.confidence[i]),
                                    xyxy=tuple(float(v) for v in dets.xyxy[i]))
                          for i in range(len(dets.xyxy))]
                    out = tr.update(ds, _Shim.t)
                    if max_predict_s is None:
                        return out
                    # What tracker_node publishes: the smoother drops a
                    # predicted box max_predict_s after its last real one.
                    return [td for td in out if not td.predicted
                            or _Shim.t - tr._last_real_t.get(td.track_id, -1e9)
                            <= max_predict_s]
            return _Shim()
        return make

    import trackers as T
    arms = [
        ('detector only (no tracking)', None),
        ('OC-SORT wrapper (SHIPPING)', _wrapped('ocsort')),
        ('OC-SORT wrapper min_hits=1', _wrapped('ocsort', min_hits=1)),
        ('OC-SORT as tracker_node publishes', _wrapped('ocsort', 1, 1.5)),
        ('ByteTrack as tracker_node would', _wrapped('bytetrack', 1, 1.5)),
        ('ByteTrack wrapper', _wrapped('bytetrack')),
        ('ByteTrack wrapper min_hits=1', _wrapped('bytetrack', min_hits=1)),
        ('raw OC-SORT (no coast rows)',
         lambda: T.OCSORTTracker(lost_track_buffer=150, frame_rate=hz,
                                 minimum_consecutive_frames=3,
                                 minimum_iou_threshold=0.2)),
        ('BoT-SORT (CMC off, no frames)',
         lambda: T.BoTSORTTracker(lost_track_buffer=150, frame_rate=hz,
                                  minimum_consecutive_frames=3,
                                  enable_cmc=False)),
    ]
    reports = []
    for name, mk in arms:
        obs = (_obs_from(acting, w, h, fps, start, n, ts=ts) if mk is None
               else _track(frames, w, h, fps, start, n, mk, ts=ts))
        r = analyse(obs, name)
        r.truth = score_truth(obs, a.truth) if a.truth else None
        reports.append(r)
        print(r.text())

    print('\n\n  === SUMMARY: gaps NOT covered by each ladder rung ===')
    print(f'    {"arm":<32} {"presence":>9} {"p50 gap":>9} {"p90 gap":>9}'
          f' {">coast":>7} {">lost":>6} {"switch":>7}')
    for r in reports:
        from mongla_vision.continuity import _pct
        d = [g.duration for g in r.gaps]
        print(f'    {r.label:<32} {100 * r.presence:8.1f}% '
              f'{1000 * _pct(d, 50):8.0f}ms {1000 * _pct(d, 90):8.0f}ms '
              f'{r.over(0.80):7d} {r.over(1.00):6d} {r.switches:7d}')
    if a.truth:
        print(f'\n  === AGAINST TRUTH ({a.truth}) ===')
        print(f'    {"arm":<32} {"recall in view":>15} {"box while absent":>17}')
        for r in reports:
            rec, fp, n_in, n_out = r.truth
            print(f'    {r.label:<32} {100 * rec:14.1f}% {100 * fp:16.1f}%'
                  f'   ({n_in} in view, {n_out} absent frames)')
    print()


def score_truth(obs, path):
    """Recall where the target is truly in view, and boxes where it is not.

    Presence alone ranks nothing: a coasted box counts as presence, so a tracker
    that keeps guessing after the target has left scores higher without being
    more right. The truth file is a hand-labelled JSON
    `{"absent": [[t0, t1], ...], "margin_s": 0.5}` in clip seconds, labelled
    from the frames by eye, independently of any model; frames within
    `margin_s` of a boundary are not scored.
    """
    import json
    with open(path) as fh:
        spec = json.load(fh)
    absent = spec['absent']
    m = float(spec.get('margin_s', 0.5))
    hit = n_in = fp = n_out = 0
    for o in obs:
        if any(t0 - m <= o.t <= t1 + m for t0, t1 in absent) and not any(
                t0 + m <= o.t <= t1 - m for t0, t1 in absent):
            continue                         # too close to a boundary
        if any(t0 <= o.t <= t1 for t0, t1 in absent):
            n_out += 1
            fp += o.seen
        else:
            n_in += 1
            hit += o.seen
    return (hit / n_in if n_in else float('nan'),
            fp / n_out if n_out else float('nan'), n_in, n_out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--video'); ap.add_argument('--model')
    ap.add_argument('--class', dest='klass', default='')
    ap.add_argument('--conf', type=float, default=0.15)
    ap.add_argument('--imgsz', type=int, default=640)
    ap.add_argument('--start-frame', type=int, default=0)
    ap.add_argument('--max-frames', type=int, default=0)
    ap.add_argument('--fps', type=float, default=0.0)
    ap.add_argument('--cache', required=True)
    ap.add_argument('--compare', action='store_true')
    ap.add_argument('--act', type=float, default=0.0,
                    help='acting bar (vision_pi conf, 0.45); default: the cache conf')
    ap.add_argument('--assoc', type=float, default=0.0,
                    help='association floor (vision_pi assoc_conf, 0.10)')
    ap.add_argument('--truth', default='',
                    help='hand-labelled absent intervals (see score_truth)')
    a = ap.parse_args()
    if a.compare:
        compare(a)
    else:
        cache_detections(a)
    return 0


if __name__ == '__main__':
    sys.exit(main())
