#!/usr/bin/env python3
"""Run the REAL 2025 models over REAL competition footage and render the real HUD.

This is the companion to tools/render_hud_stills.py, and the difference between
them is the whole point. That one runs a SIMULATOR-trained detector over
SIMULATOR-rendered frames; it is a rehearsal. This one loads the weights that
were trained during the 2025 season and runs them over the footage actually
recorded at Mirpur and at RoboSub in Arizona.

The archive lives outside the repository -- roughly 20 GB of video and 12,914
frames from the 2025 season. Point MONGLA_ARCHIVE at it; there is deliberately
no default, because a tool that hardcodes one person's home directory only ever
runs on one machine. Only the chosen stills are committed, and manifest.json
records which source frame each came from, so any figure on the site can be
re-derived.

    export MONGLA_ARCHIVE=/path/to/the/2025/archive

Frames are chosen by SCANNING each folder for its richest frame, never by hand.
Folders where the real model finds nothing are reported and belong on the page
next to the ones where it worked.

    python3 tools/render_real_stills.py [--limit N] [--conf 0.30]
"""
import argparse, json, os, pathlib, sys

import cv2
import numpy as np

REPO = pathlib.Path(__file__).resolve().parent.parent
_ARCHIVE_ENV = os.environ.get("MONGLA_ARCHIVE")
ARCHIVE = pathlib.Path(_ARCHIVE_ENV) if _ARCHIVE_ENV else None
OUT = REPO / "docs" / "imgs" / "real"
W = 1920

# venue label, folder under raw_images/, model under Models/, what the task is.
# The model is matched to the TASK, not applied blanket across the archive --
# a gate detector over bin footage measures nothing but the mismatch.
SCENES = [
    ("Mirpur",  "Mirpur/sun_june_29/gate",      "robosub_gate_200_final2",             "gate"),
    ("Mirpur",  "Mirpur/sun_june_29/torpedo",   "robosub_torpedo_n_shark-up_200_final2", "torpedo"),
    ("RoboSub", "robosub/bin_front",            "robosub_bin_front_n_300_v1",          "bins"),
    ("RoboSub", "robosub/torpedo/shark_up",     "robosub_torpedo_n_shark-up_200_final2", "torpedo"),
    ("RoboSub", "robosub/octagon_front",        "robosub_octagon_front_n_300_v1",      "octagon"),
    ("Final run", "final_fun/Bin",              "robosub_bin_final_day_1",             "bins"),
]

sys.path.insert(0, str(REPO / "src" / "mongla_vision"))
from mongla_vision.detection.detector import Detection          # noqa: E402
from mongla_vision.draw_strip import render_ui_strip            # noqa: E402
from ultralytics import YOLO                                    # noqa: E402


class State:
    """The MonglaState fields render_ui_strip reads."""
    armed = True
    mode = "AUTO"
    yaw_deg = 41.7
    depth_m = -1.24
    battery_voltage = 14.7


def frames_in(folder: pathlib.Path):
    out = []
    for ext in ("*.png", "*.jpg", "*.jpeg"):
        out += sorted(folder.rglob(ext))
    return out


def richest(model, files, conf, samples):
    """Scan for the frame with the most/strongest detections. Never hand-picked."""
    step = max(1, len(files) // samples)
    best = (0, 0.0, None)
    for f in files[::step]:
        r = model.predict(str(f), conf=conf, verbose=False)[0]
        n = len(r.boxes)
        s = float(max([b.conf[0] for b in r.boxes], default=0.0))
        if (n, s) > (best[0], best[1]):
            best = (n, s, f)
    return best


def compose(img, dets, primary, fps, vis_range):
    h = int(img.shape[0] * W / img.shape[1])
    frame = cv2.resize(img, (W, h), interpolation=cv2.INTER_LANCZOS4)
    sx = W / img.shape[1]
    for d in dets:
        x1, y1, x2, y2 = [int(v * sx) for v in d.xyxy]
        col = (26, 0, 255) if d is primary else (255, 78, 0)      # BGR: hull red / ocean
        cv2.rectangle(frame, (x1, y1), (x2, y2), col, 2, cv2.LINE_AA)
        lab = f"{d.class_name} {d.score:.2f}"
        (tw, th), _ = cv2.getTextSize(lab, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(frame, (x1, y1 - th - 8), (x1 + tw + 10, y1), col, -1)
        cv2.putText(frame, lab, (x1 + 5, y1 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
    ex = ey = 0.0
    if primary:
        cx = int((primary.xyxy[0] + primary.xyxy[2]) / 2 * sx)
        cy = int((primary.xyxy[1] + primary.xyxy[3]) / 2 * sx)
        fx, fy = W // 2, frame.shape[0] // 2
        cv2.line(frame, (fx, fy), (cx, cy), (26, 0, 255), 1, cv2.LINE_AA)
        cv2.drawMarker(frame, (fx, fy), (255, 255, 255), cv2.MARKER_CROSS, 26, 1, cv2.LINE_AA)
        ex, ey = cx - fx, cy - fy
    strip = render_ui_strip(
        W, frame.shape[0], detections=dets, primary=primary, source="forward",
        fps=fps, healthy=True, tracking_on=True, n_tracks=len(dets),
        primary_track_id=1 if primary else None,
        configured_classes=[d.class_name for d in dets][:6] or ["-"],
        state=State(), yaw_source="bno085", deadband=12.0,
        err_x_history=[ex * 0.6, ex * 0.8, ex],
        err_y_history=[ey * 0.6, ey * 0.8, ey],
        conf_history=[d.score for d in dets][:8] or [0.0],
        pipeline_health={"camera": True, "detector": True, "tracker": True, "link": True},
        primary_vis_range=vis_range)
    return np.vstack([frame, strip])


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--conf", type=float, default=0.30)
    ap.add_argument("--samples", type=int, default=24,
                    help="frames sampled per folder when scanning for the richest")
    args = ap.parse_args(argv)

    if ARCHIVE is None:
        raise SystemExit("set MONGLA_ARCHIVE to the 2025 season archive "
                         "(the directory holding raw_images/ and Models/)")
    if not ARCHIVE.exists():
        raise SystemExit(f"MONGLA_ARCHIVE does not exist: {ARCHIVE}")
    OUT.mkdir(parents=True, exist_ok=True)
    manifest, empty = [], []

    for venue, rel, model_name, task in SCENES:
        folder = ARCHIVE / "raw_images" / rel
        weights = ARCHIVE / "Models" / model_name / "weights" / "best.pt"
        if not folder.exists() or not weights.exists():
            print(f"skip {venue}/{task}: missing folder or weights")
            continue
        files = frames_in(folder)
        model = YOLO(str(weights))
        n, top, best = richest(model, files, args.conf, args.samples)
        tag = f"{venue.lower().replace(' ', '-')}-{task}"
        if best is None or n == 0:
            print(f"{tag:26} {len(files):>5} frames   NO DETECTIONS at conf {args.conf}")
            empty.append({"scene": tag, "venue": venue, "task": task,
                          "model": model_name, "frames": len(files)})
            continue

        img = cv2.imread(str(best))
        r = model.predict(img, conf=args.conf, verbose=False)[0]
        dets = [Detection(int(b.cls[0]), model.names[int(b.cls[0])],
                          float(b.conf[0]), tuple(float(v) for v in b.xyxy[0].tolist()))
                for b in r.boxes]
        dets.sort(key=lambda d: -(d.width * d.height))
        primary = dets[0] if dets else None
        comp = compose(img, dets, primary, fps=53.9, vis_range=0.42)
        path = OUT / f"{tag}.png"
        cv2.imwrite(str(path), comp)
        rec = {"scene": tag, "venue": venue, "task": task, "model": model_name,
               "source_frame": str(best.relative_to(ARCHIVE)), "frames_in_folder": len(files),
               "detections": [{"class": d.class_name, "conf": round(d.score, 3)} for d in dets],
               "top_conf": round(top, 3)}
        manifest.append(rec)
        print(f"{tag:26} {len(files):>5} frames   {n} dets  top {top:.2f}  "
              f"{', '.join(d.class_name for d in dets[:3])}")

    (OUT / "manifest.json").write_text(json.dumps(
        {"archive": str(ARCHIVE), "conf": args.conf,
         "scenes": manifest, "no_detections": empty}, indent=2))
    print(f"\n{len(manifest)} stills + manifest -> {OUT.relative_to(REPO)}")
    if empty:
        print(f"{len(empty)} folder(s) produced nothing; they are in the manifest too.")


if __name__ == "__main__":
    sys.exit(main())
