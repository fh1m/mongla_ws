#!/usr/bin/env python3
"""The detection-gap distribution, measured at NATIVE FRAME RATE.

This is the number every rung of the lock ladder is sized on, and until now it
had never been recorded. `tools/detection_continuity.py` is the instrument;
this is the harness that runs it over archived competition footage so the
ladder can be checked against reality instead of against itself.

    coast_s 0.80    lost_grace_s 1.00    kalman 1.50    track_buffer 5.00

WHY NOT A SAMPLED SWEEP -- the mistake that produced this file. Sampling every
Nth frame makes the smallest observable gap one sample interval. At a ~1-4 s
interval EVERY gap exceeds a 0.8 s rung by construction, and the first run of
this measurement duly reported 307 of 307 gaps over `coast_s`: a devastating
finding, and pure arithmetic. An instrument coarser than the thing it measures
returns a plausible number and no error at all.

Two further artifacts, both caught the same way -- by a number that implied an
implausible mechanism:

  * CONCATENATING WINDOWS FABRICATES A GAP spanning the jump between them.
    That is where a 113 s "gap" inside a 30 s window came from. Gaps are
    collected per window and only then pooled.
  * A CONTAINER CAN LIE ABOUT FPS. Mirpur's mkv reports 1000.0 (ffprobe says
    `0/0`), which silently divides every gap by ~30 -- in the flattering
    direction. Anything outside 5..120 fps is skipped and said so.

Usage:  python3 tools/gap_distribution.py
"""
import sys, os, glob, contextlib, io, cv2
sys.path.insert(0,'src/mongla_vision')
from mongla_vision.continuity import Obs, analyse, LADDER, _pct
from ultralytics import YOLO
V='/home/fh1m/Work/Projects/Mongla/2025/raw_videos'; M='/home/fh1m/Music/detect'
CONF=0.10; WIN=300           # consecutive frames per window
CLIPS=[('final_run/gate_back.mkv','robosub_gate_200_final2'),
       ('final_run/bin.mkv','robosub_bin_200_v1'),
       ('final_run/octagon_Bottom.mkv','robosub_octagon_n_200_final'),
       ('robosub/clips/torpedo/torpedo_shark_up_1.mp4','robosub_torpedo_n_shark-up_200_final2'),
       ('robosub/clips/bin_front/bin_front_#1.mp4','robosub_bin_200_v1'),
       ('Mirpur/Sun_June_21/torpedo.mkv','robosub_torpedo_n_shark-up_200_final2')]
cache={}; allg=[]
print(f'\n  NATIVE-RATE gap distribution, {WIN} consecutive frames x 3 windows'
      f', conf {CONF}\n')
print(f'  {"clip":<40}{"fps":>6}{"n":>6}{"pres":>7}{"gaps":>6}'
      f'{"p50":>8}{"p90":>8}{"max":>8}')
for rel,mn in CLIPS:
    p=f'{V}/{rel}'
    if not os.path.exists(p): print(f'  {rel} MISSING'); continue
    if mn not in cache:
        with contextlib.redirect_stdout(io.StringIO()),contextlib.redirect_stderr(io.StringIO()):
            cache[mn]=YOLO(f'{M}/{mn}/weights/best.pt')
    mo=cache[mn]
    cap=cv2.VideoCapture(p); fps=cap.get(cv2.CAP_PROP_FPS) or 30.0
    if not (5.0 <= fps <= 120.0):
        # Mirpur's mkv reports 1000.0 -- container metadata, not a rate. A
        # bogus fps silently divides every gap by ~30, in the flattering
        # direction. ffprobe's avg_frame_rate said 0/0 for these too.
        print(f'  {rel[:39]:<40} SKIPPED -- container claims {fps:.0f} fps')
        cap.release(); continue
    total=int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0); cap.release()
    obs=[]; clip_gaps=[]
    for frac in (0.25,0.5,0.75):
        cap=cv2.VideoCapture(p)
        start=int(total*frac) if total else 0
        cap.set(cv2.CAP_PROP_POS_FRAMES,start)
        w=[]
        for k in range(WIN):
            ok,fr=cap.read()
            if not ok: break
            with contextlib.redirect_stdout(io.StringIO()),contextlib.redirect_stderr(io.StringIO()):
                r=mo.predict(fr,conf=CONF,imgsz=640,verbose=False)[0]
            t=(start+k)/fps
            if len(r.boxes):
                b=r.boxes[int(r.boxes.conf.argmax())]
                x1,y1,x2,y2=[float(v) for v in b.xyxy[0]]
                h,wd=fr.shape[:2]
                w.append(Obs(t=t,seen=True,score=float(b.conf[0]),
                             cx=(x1+x2)/2/wd,cy=(y1+y2)/2/h,
                             area=(x2-x1)*(y2-y1)/(wd*h)))
            else:
                w.append(Obs(t=t,seen=False))
        cap.release()
        rep=analyse(w,rel)
        allg+= [g.duration for g in rep.gaps]
        obs+=w
        clip_gaps+=[g.duration for g in rep.gaps]
    # Per-window, NEVER concatenated: joining three windows fabricates a gap
    # spanning the jump between them, which is where the 77 s / 113 s maxima
    # in the first run came from.
    rep=analyse(obs,rel); d=clip_gaps
    print(f'  {rel[:39]:<40}{fps:6.1f}{len(obs):6d}{100*rep.presence:6.1f}%'
          f'{len(d):6d}{(_pct(d,50) if d else 0):8.2f}'
          f'{(_pct(d,90) if d else 0):8.2f}{(max(d) if d else 0):8.2f}', flush=True)

print(f'\n  ALL {len(allg)} within-window gaps vs the shipped ladder:')
for name,thr in LADDER:
    n=sum(1 for g in allg if g>thr)
    print(f'    {name:<26} {n:5d} exceed  ({100*n/max(len(allg),1):5.1f} %)')
if allg:
    print(f'\n    gap p50 {_pct(allg,50):.3f}s  p90 {_pct(allg,90):.3f}s  '
          f'p99 {_pct(allg,99):.3f}s  max {max(allg):.2f}s')
    print(f'    smallest observable gap at 30 fps: {1/30:.3f}s -- '
          f'well under every rung, so this measurement can SEE them')
