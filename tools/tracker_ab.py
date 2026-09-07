#!/usr/bin/env python3
"""Which tracker closes more gaps on REAL competition footage?

The question this settles, with numbers rather than an argument: the coast
layer exists to bridge detection gaps, and nobody has ever measured how much
of a real gap any tracker actually bridges.

DETERMINISTIC BY CONSTRUCTION. Inference runs ONCE and the detections are
cached; every tracker then sees byte-identical input. No camera, no GIL, no
dropped frames -- so the tracker is the only variable, which is exactly what
a rosbag replay cannot give (`ros2 bag play` is wall-clock paced and the
pipeline drops a different frame subset each run).

    # cache the detections once
    python3 tools/tracker_ab.py --video ... --model ... --class bin_shaw_fish \
        --conf 0.15 --cache /tmp/bin.npz

    # then compare, as many times as you like, on identical input
    python3 tools/tracker_ab.py --cache /tmp/bin.npz --compare

What is reported is the CONTINUITY report from `duburi_vision.continuity` --
the same ruler used for the raw detector -- so "how much did tracking buy" is
answered in the units the lock ladder is written in.
"""
import argparse
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..',
                                'src', 'duburi_vision'))

from duburi_vision.continuity import Obs, analyse    # noqa: E402


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
    np.savez(a.cache, rows=np.array(rows, dtype=np.float64),
             meta=np.array([fps, w, h, a.start_frame, kept, a.conf]))
    print(f'  cached {len(rows)} boxes over {kept} frames in '
          f'{time.monotonic() - t0:.0f}s -> {a.cache}')


def _obs_from(frames, w, h, fps, start, n):
    """Detector-only observations: largest box per frame, as `bbox_error` picks."""
    out = []
    for i in range(start, start + n):
        best = None
        for (x1, y1, x2, y2, sc) in frames.get(i, ()):
            area = (x2 - x1) * (y2 - y1)
            if best is None or area > best[0]:
                best = (area, (x1 + x2) / 2 / w, (y1 + y2) / 2 / h, sc)
        out.append(Obs(t=i / fps, seen=best is not None,
                       score=best[3] if best else 0.0,
                       cx=best[1] if best else float('nan'),
                       cy=best[2] if best else float('nan')))
    return out


def _track(frames, w, h, fps, start, n, make_tracker, needs_frame=False):
    """Feed the cached detections through a tracker; a coasted box counts as
    SEEN, which is the whole point -- that is what the control loop steers on
    when `coast_s > 0`."""
    import supervision as sv
    tr = make_tracker()
    out = []
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
            type(tr).t = i / fps
        try:
            res = tr.update(dets)
        except TypeError:
            # BoT-SORT wants the frame for optical-flow CMC. We have no frame
            # here by design -- the cache is the point -- so it is compared
            # WITHOUT CMC and that is stated rather than silently assumed.
            res = tr.update(dets, frame=np.zeros((h, w, 3), np.uint8))
        best = None
        if isinstance(res, list):          # our wrapper: TrackedDetection[]
            for td in res:
                x1, y1, x2, y2 = td.xyxy
                area = (x2 - x1) * (y2 - y1)
                if best is None or area > best[0]:
                    best = (area, (x1 + x2) / 2 / w, (y1 + y2) / 2 / h,
                            td.score, td.track_id)
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
        out.append(Obs(t=i / fps, seen=best is not None,
                       score=best[3] if best else 0.0,
                       cx=best[1] if best else float('nan'),
                       cy=best[2] if best else float('nan'),
                       track_id=best[4] if best else -1))
    return out


def compare(a):
    z = np.load(a.cache)
    rows, meta = z['rows'], z['meta']
    fps, w, h, start, n, conf = (meta[0], int(meta[1]), int(meta[2]),
                                 int(meta[3]), int(meta[4]), meta[5])
    frames = {}
    for r in rows:
        frames.setdefault(int(r[0]), []).append(tuple(r[1:]))
    print(f'\n  {n} frames @ {fps:.0f} fps, {len(rows)} boxes cached at '
          f'conf={conf:.2f}\n')

    hz = float(fps)
    # OUR WRAPPER, not the raw library. `RoboflowTracker.update()` adds the
    # COASTED rows out of `tracked_objects` on top of whatever the library
    # returns -- which is the entire coast layer. Comparing the bare library
    # would measure something we do not ship, and would have reported that
    # tracking buys nothing when the wrapper is what does the work.
    from duburi_vision.tracking.roboflow_tracker import RoboflowTracker
    from duburi_vision.detection.detector import Detection

    def _wrapped(ttype, min_hits=3):
        def make():
            tr = RoboflowTracker(tracker_type=ttype, track_buffer=150,
                                 frame_rate=hz, min_hits=min_hits,
                                 iou_threshold=0.2,
                                 track_activation_threshold=0.40)

            class _Shim:
                def update(self, dets, frame=None):
                    ds = [Detection(class_id=0, class_name='t',
                                    score=float(dets.confidence[i]),
                                    xyxy=tuple(float(v) for v in dets.xyxy[i]))
                          for i in range(len(dets.xyxy))]
                    return tr.update(ds, _Shim.t)
            return _Shim()
        return make

    import trackers as T
    arms = [
        ('detector only (no tracking)', None),
        ('OC-SORT wrapper (SHIPPING)', _wrapped('ocsort')),
        ('OC-SORT wrapper min_hits=1', _wrapped('ocsort', min_hits=1)),
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
        obs = (_obs_from(frames, w, h, fps, start, n) if mk is None
               else _track(frames, w, h, fps, start, n, mk))
        r = analyse(obs, name)
        reports.append(r)
        print(r.text())

    print('\n\n  === SUMMARY: gaps NOT covered by each ladder rung ===')
    print(f'    {"arm":<32} {"presence":>9} {"p50 gap":>9} {"p90 gap":>9}'
          f' {">coast":>7} {">lost":>6} {"switch":>7}')
    for r in reports:
        from duburi_vision.continuity import _pct
        d = [g.duration for g in r.gaps]
        print(f'    {r.label:<32} {100 * r.presence:8.1f}% '
              f'{1000 * _pct(d, 50):8.0f}ms {1000 * _pct(d, 90):8.0f}ms '
              f'{r.over(0.80):7d} {r.over(1.00):6d} {r.switches:7d}')
    print()


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
    a = ap.parse_args()
    if a.compare:
        compare(a)
    else:
        cache_detections(a)
    return 0


if __name__ == '__main__':
    sys.exit(main())
