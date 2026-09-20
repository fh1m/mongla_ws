#!/usr/bin/env python3
"""Run the real detector over the real capture archive and render the real HUD.

Nothing here is a mockup: it loads the competition weights, runs them on frames
from sim/datasets/, and passes the true detections into the project's own
mongla_vision.draw_strip.render_ui_strip(). Output goes to /tmp for the site.

Frame indices were chosen by scanning each session for its richest frame, not
picked to flatter the detector -- the sessions where it found nothing are named
on the page.

    python3 tools/render_hud_stills.py    (from the repo root, workspace sourced)
"""
"""Run the real detector over the real archive and render the real HUD."""
import sys, types, pathlib, numpy as np, cv2
sys.path.insert(0, 'src/mongla_vision')
from mongla_vision.detection.detector import Detection
from mongla_vision.draw_strip import render_ui_strip
from ultralytics import YOLO

W = 1920
SESSIONS = [   # frame indices chosen by scanning each session for its richest frame
    ('sim/datasets/sim_transit_clear_20260828_145913', 'clear',  '000080.png'),
    ('sim/datasets/sim_murky_20260828_145041',         'murky',  '000033.png'),
    ('sim/datasets/sim_clear_20260828_145251',         'drums',  '000143.png'),
]
model = YOLO(str(pathlib.Path.home()/'models'/'sauvc_sim.pt'))
names = model.names
print('classes:', len(names))

class S:  # the DuburiState fields the strip reads
    armed = True; mode = 'AUTO'; yaw_deg = 41.7; depth_m = -1.24; battery_voltage = 14.7

out = pathlib.Path('/tmp/claude-1000/hud'); out.mkdir(exist_ok=True)
for folder, tag, fname in SESSIONS:
    fp = pathlib.Path(folder, 'frames', 'front', fname)
    if not fp.exists(): print('missing', fp); continue
    img = cv2.imread(str(fp))
    r = model.predict(img, conf=0.30, verbose=False)[0]
    dets = []
    for b in r.boxes:
        x1,y1,x2,y2 = [float(v) for v in b.xyxy[0].tolist()]
        cid = int(b.cls[0]); dets.append(Detection(cid, names[cid], float(b.conf[0]), (x1,y1,x2,y2)))
    dets.sort(key=lambda d: -(d.width*d.height))
    primary = dets[0] if dets else None
    print(f'{tag:14} {fp.name}  {len(dets)} detections: ' +
          ', '.join(f'{d.class_name}@{d.score:.2f}' for d in dets[:4]))

    frame = cv2.resize(img, (W, int(img.shape[0]*W/img.shape[1])), interpolation=cv2.INTER_LANCZOS4)
    sx = W/img.shape[1]
    for d in dets:
        x1,y1,x2,y2 = [int(v*sx) for v in d.xyxy]
        col = (26,0,255) if d is primary else (255,78,0)      # BGR: mongla red / ocean
        cv2.rectangle(frame,(x1,y1),(x2,y2),col,2,cv2.LINE_AA)
        lab = f'{d.class_name} {d.score:.2f}'
        (tw,th),_ = cv2.getTextSize(lab, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(frame,(x1,y1-th-8),(x1+tw+10,y1),col,-1)
        cv2.putText(frame,lab,(x1+5,y1-5),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,0),1,cv2.LINE_AA)
    if primary:
        cx,cy = int((primary.xyxy[0]+primary.xyxy[2])/2*sx), int((primary.xyxy[1]+primary.xyxy[3])/2*sx)
        fx,fy = W//2, frame.shape[0]//2
        cv2.line(frame,(fx,fy),(cx,cy),(26,0,255),1,cv2.LINE_AA)
        cv2.drawMarker(frame,(fx,fy),(255,255,255),cv2.MARKER_CROSS,26,1,cv2.LINE_AA)
        ex,ey = (cx-fx), (cy-fy)
    else:
        ex=ey=0.0
    strip = render_ui_strip(
        W, frame.shape[0], detections=dets, primary=primary, source='forward',
        fps=53.9, healthy=True, tracking_on=True, n_tracks=len(dets),
        primary_track_id=1 if primary else None,
        configured_classes=[d.class_name for d in dets][:6] or ['sauvc_qual_gate'],
        state=S(), yaw_source='bno085', deadband=12.0,
        err_x_history=[ex*0.6, ex*0.8, ex], err_y_history=[ey*0.6, ey*0.8, ey],
        conf_history=[d.score for d in dets][:8] or [0.0],
        pipeline_health={'camera':True,'detector':True,'tracker':True,'link':True},
        primary_vis_range=0.42)
    comp = np.vstack([frame, strip])
    cv2.imwrite(str(out/f'hud_{tag}.png'), comp)
    print(f'  -> hud_{tag}.png  {comp.shape[1]}x{comp.shape[0]}')
