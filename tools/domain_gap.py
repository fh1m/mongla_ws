#!/usr/bin/env python3
"""The same model on STILLS it was trained from, and on VIDEO of the same run.

WHY THIS EXISTS. Every one of our 2025 competition models reports mAP50 =
0.995 in its training log -- and on real footage of the same competition the
target is detected in a small fraction of frames. A validation score answers
"can it find the object in a frame like the ones it trained on". It does not
answer "will it find the object while the vehicle is moving", and those turn
out to be very different questions.

This measures both with ONE detector, ONE threshold, so the only variable is
the input. Stills go through the labelled dataset directory (ground truth is
present, so this is real RECALL, not a score distribution -- the limitation
`hailo-vision.md:232` records about our previous measurement). Video goes
through the clip, where a per-frame ground truth does not exist, so the
comparable quantity is per-frame PRESENCE.

The gap between them is what an augmentation or preprocessing change has to
close, and it is the number to re-run after any such change.
"""
import argparse
import glob
import os
import sys


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', required=True)
    ap.add_argument('--stills', required=True, help='dir with images/ + labels/')
    ap.add_argument('--video', default='')
    ap.add_argument('--conf', type=float, default=0.25)
    ap.add_argument('--imgsz', type=int, default=640)
    ap.add_argument('--max-stills', type=int, default=400)
    ap.add_argument('--max-frames', type=int, default=1500)
    ap.add_argument('--start-frame', type=int, default=0)
    a = ap.parse_args()

    from ultralytics import YOLO
    model = YOLO(a.model)
    names = model.names if isinstance(model.names, dict) else dict(
        enumerate(model.names))

    # ---- stills: real recall, because labels exist ------------------------
    img_dir = os.path.join(a.stills, 'images')
    lbl_dir = os.path.join(a.stills, 'labels')
    # Real images only. These directories mix .png with .npy frame dumps,
    # and ultralytics raises on the latter rather than skipping it.
    _EXT = ('.png', '.jpg', '.jpeg', '.bmp', '.webp', '.tif', '.tiff')
    imgs = sorted(f for f in glob.glob(os.path.join(img_dir, '*'))
                  if f.lower().endswith(_EXT))[:a.max_stills]
    with_label, found = 0, 0
    for im in imgs:
        stem = os.path.splitext(os.path.basename(im))[0]
        lp = os.path.join(lbl_dir, stem + '.txt')
        if not os.path.exists(lp) or not open(lp).read().strip():
            continue        # nothing annotated here -- not a miss
        with_label += 1
        r = model.predict(im, conf=a.conf, imgsz=a.imgsz, verbose=False)[0]
        if len(r.boxes):
            found += 1
    print(f'\n  === {os.path.basename(a.model)} ===')
    print(f'  STILLS  {a.stills}')
    if with_label:
        print(f'    {with_label} labelled images, detected in {found} '
              f'-> RECALL {100.0 * found / with_label:.1f} %')
    else:
        print('    no labelled images found')

    # ---- video: presence, because per-frame truth does not exist ---------
    if a.video:
        import cv2
        cap = cv2.VideoCapture(a.video)
        if a.start_frame:
            cap.set(cv2.CAP_PROP_POS_FRAMES, a.start_frame)
        seen = n = 0
        while n < a.max_frames:
            ok, f = cap.read()
            if not ok:
                break
            r = model.predict(f, conf=a.conf, imgsz=a.imgsz, verbose=False)[0]
            seen += 1 if len(r.boxes) else 0
            n += 1
        cap.release()
        print(f'  VIDEO   {os.path.basename(a.video)}')
        print(f'    {n} frames, target present in {seen} '
              f'-> PRESENCE {100.0 * seen / max(n, 1):.1f} %')
        if with_label and n:
            rec = 100.0 * found / with_label
            pres = 100.0 * seen / n
            print(f'\n    THE DOMAIN GAP: {rec:.1f} % on stills, '
                  f'{pres:.1f} % on video of the same competition.')
            print(f'    A validation score answers a different question than '
                  f'"will it hold a lock while moving".')
    print()
    return 0


if __name__ == '__main__':
    sys.exit(main())
