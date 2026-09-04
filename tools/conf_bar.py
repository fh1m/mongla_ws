"""THE conf question, on labelled held-out sessions.

Matching is IoU-only (class-agnostic): "is there an object here and did we box
it in the right place", which is the question an alignment stack asks. It is
OPTIMISTIC about classification -- a shark box on a sawfish ground truth counts
-- and that is stated rather than hidden, because class names differ across
these datasets and a class-strict matcher would score cross-venue pairs at 0
for a naming reason, not a vision one.
"""
import sys, contextlib, io
sys.path.insert(0,'tools'); sys.path.insert(0,'src/duburi_vision')
from recall_matrix import conf_sweep
from ultralytics import YOLO
D='/home/fh1m/Work/Projects/Duburi/2025/datasets'; M='/home/fh1m/Music/detect'
T=[0.05,0.08,0.10,0.12,0.15,0.20,0.25,0.30,0.40,0.50]
PAIRS=[
 ('torpedo  held-out sess','robosub_torpedo_n_shark-down_200_v12','ROBOSUB/DAY-1/TORPEDO/Torpedo_Down'),
 ('torpedo  same-ish     ','robosub_torpedo_n_shark-down_200_v12','TASK_4_TORPEDO/torpedo_v1'),
 ('bin      held-out sess','robosub_bin_front_n_300_v1','ROBOSUB/DAY-1/BIN/bin-1'),
 ('octagon  held-out sess','robosub_octagon_n_200_final','TASK_5_OCTAGON/ext/oct_zawad'),
 ('octagon  2nd held-out ','robosub_octagon_n_200_final','TASK_5_OCTAGON/ext/slot-1_obb'),
 ('gate     CROSS-VENUE  ','robosub_gate_200_final2','TASK_1_GATE/Mirpur/sun_june_29/gate-1'),
]
print(f'\n  PRECISION and RECALL vs CONF -- the measurement the 0.15->0.10')
print(f'  decision never made (it moved on presence, which cannot see an FP).\n')
for tag,mn,ds in PAIRS:
    try:
        with contextlib.redirect_stdout(io.StringIO()),contextlib.redirect_stderr(io.StringIO()):
            m=YOLO(f'{M}/{mn}/weights/best.pt')
            rows,n=conf_sweep(m,f'{D}/{ds}',640,90,T)
    except Exception as e:
        print(f'  {tag}  FAILED: {type(e).__name__}'); continue
    if not n: print(f'  {tag}  no labelled pairs'); continue
    best=max(rows,key=lambda r:r['f1'])
    r10=next(r for r in rows if abs(r['conf']-0.10)<1e-9)
    r15=next(r for r in rows if abs(r['conf']-0.15)<1e-9)
    print(f'  {tag}  n={n}')
    print(f'      conf 0.10  R {100*r10["recall"]:5.1f}%  P {100*r10["precision"]:5.1f}%  F1 {r10["f1"]:.3f}')
    print(f'      conf 0.15  R {100*r15["recall"]:5.1f}%  P {100*r15["precision"]:5.1f}%  F1 {r15["f1"]:.3f}')
    print(f'      F1 knee at conf {best["conf"]:.2f}  R {100*best["recall"]:5.1f}%  P {100*best["precision"]:5.1f}%  F1 {best["f1"]:.3f}')
    d=r10['precision']-r15['precision']
    print(f'      0.15 -> 0.10 costs {100*d:+.1f} pts of precision, '
          f'buys {100*(r10["recall"]-r15["recall"]):+.1f} pts of recall\n', flush=True)
