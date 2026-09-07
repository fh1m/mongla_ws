#!/usr/bin/env python3
"""How often do we lose the target in real water, and for how long?

THE QUESTION NOBODY HAS ANSWERED. Six constants protect a lock -- the
freshness ramp, `coast_s` (0.8), `lost_grace_s` (1.0), `_STALE_LIMIT_S` (1.0),
the Kalman `max_predict_s` (1.5), the tracker's `track_buffer` (5.0 s) -- and
every one is sized from detection RATE and from the others' ordering. None is
sized from the length of a real gap, because nothing has ever measured one.

This runs real competition footage through a real detector and reports the
distribution. Deterministic: the same file yields the same frames every time,
so a tracking change is the only variable in an A/B.

    # gap distribution on the 2025 final run
    python3 tools/detection_continuity.py \\
        --video ~/Work/.../raw_videos/final_run/gate_back.mkv \\
        --model ~/tmp/.../MODELS_USA/GATE_DAY_1/best.pt \\
        --classes shark_gate --conf 0.25

    # what the SAME footage looks like at the tracker's floor
    ... --conf 0.05

A note on honesty: presence here is presence-in-THIS-CLIP. A clip where the
camera pans away from the prop will show gaps that are not detector failures.
That is why `--class` matters and why the leading/trailing absence is excluded
(see `continuity.find_gaps`), and why the report prints the clip's own frame
count so a suspicious result can be checked against the video.
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..',
                                'src', 'duburi_vision'))

from duburi_vision.continuity import Obs, analyse   # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--video', required=True)
    ap.add_argument('--model', required=True)
    ap.add_argument('--classes', default='',
                    help='comma-separated; default = every class the model has')
    ap.add_argument('--conf', type=float, default=0.25)
    ap.add_argument('--imgsz', type=int, default=640)
    ap.add_argument('--start-frame', type=int, default=0,
                    help='skip to here first -- a clip where the prop only '
                         'enters view halfway would otherwise report the '
                         'approach as a detector failure')
    ap.add_argument('--max-frames', type=int, default=0, help='0 = whole clip')
    ap.add_argument('--stride', type=int, default=1,
                    help='process every Nth frame; simulates a slower detector')
    ap.add_argument('--device', default='')
    ap.add_argument('--fps', type=float, default=0.0,
                    help='override the clip fps if its metadata lies')
    ap.add_argument('--save-obs', default='',
                    help='write the per-frame observations as JSON for a '
                         'later A/B without re-running inference')
    a = ap.parse_args()

    import cv2
    from ultralytics import YOLO

    model = YOLO(a.model)
    names = model.names if isinstance(model.names, dict) else dict(
        enumerate(model.names))
    wanted = [c.strip() for c in a.classes.split(',') if c.strip()] or None

    cap = cv2.VideoCapture(a.video)
    if not cap.isOpened():
        print(f'cannot open {a.video}')
        return 1
    fps = a.fps or cap.get(cv2.CAP_PROP_FPS) or 30.0
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f'\n  {os.path.basename(a.video)}  {w}x{h} @ {fps:.1f} fps, '
          f'{total} frames')
    print(f'  {os.path.basename(os.path.dirname(a.model))}  '
          f'classes={list(names.values())}')
    print(f'  conf={a.conf}  imgsz={a.imgsz}  stride={a.stride}')
    if wanted:
        print(f'  measuring: {wanted}')

    if a.start_frame:
        cap.set(cv2.CAP_PROP_POS_FRAMES, a.start_frame)
        print(f'  starting at frame {a.start_frame} '
              f'({a.start_frame / fps:.0f}s)')

    per_class = {}
    idx = a.start_frame
    kept = 0
    t0 = time.monotonic()
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if a.max_frames and kept >= a.max_frames:
            break
        if idx % a.stride:
            idx += 1
            continue
        # The frame's OWN time, from its index -- not wall time. The clip is
        # the ground truth for when things happened; how fast this script
        # runs is irrelevant to the gap lengths.
        t = idx / fps
        r = model.predict(frame, conf=a.conf, imgsz=a.imgsz, verbose=False,
                          device=a.device or None)[0]

        seen_this = {}
        for b in r.boxes:
            cls = names.get(int(b.cls[0]), str(int(b.cls[0])))
            if wanted and cls not in wanted:
                continue
            score = float(b.conf[0])
            x1, y1, x2, y2 = (float(v) for v in b.xyxy[0])
            # Largest-area wins, which is what `bbox_error` does by default.
            area = ((x2 - x1) * (y2 - y1)) / float(w * h)
            prev = seen_this.get(cls)
            if prev is None or area > prev[2]:
                seen_this[cls] = (((x1 + x2) / 2) / w, ((y1 + y2) / 2) / h,
                                  area, score)

        for cls in (wanted or names.values()):
            hit = seen_this.get(cls)
            per_class.setdefault(cls, []).append(
                Obs(t=t, seen=hit is not None,
                    score=hit[3] if hit else 0.0,
                    cx=hit[0] if hit else float('nan'),
                    cy=hit[1] if hit else float('nan'),
                    area=hit[2] if hit else 0.0))
        idx += 1
        kept += 1
        if kept % 500 == 0:
            print(f'    ... {kept} frames', flush=True)
    cap.release()

    elapsed = time.monotonic() - t0
    print(f'\n  processed {kept} frames in {elapsed:.0f}s '
          f'({kept / max(elapsed, 1e-6):.1f} fps offline)')

    for cls, obs in sorted(per_class.items()):
        if not any(o.seen for o in obs):
            print(f'\n  === {cls} ===\n    NEVER DETECTED in this clip.')
            continue
        print(analyse(obs, cls).text())
    print()

    if a.save_obs:
        import json
        with open(a.save_obs, 'w') as fh:
            json.dump({c: [[o.t, o.seen, o.score, o.cx, o.cy, o.area]
                           for o in v] for c, v in per_class.items()}, fh)
        print(f'  observations -> {a.save_obs}\n')
    return 0


if __name__ == '__main__':
    sys.exit(main())
