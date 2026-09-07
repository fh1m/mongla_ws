"""A2 -- RETURN: 2,882 raw frames + 302 labelled.

⛔ THIS TOOL'S ORIGINAL ANSWER (0.0 %) WAS RIGHT BY ACCIDENT AND IS NOT
EVIDENCE. Two defects, both of which produce a confident number:

1. It NAME-MATCHES `gate_backward`, a class no 2025 model has, so 0.0 % is
   arithmetic rather than a measurement.
2. The `robosub_gate*` weights it tests predict **nothing on the FRONT side
   of this same season's gate data** either -- 0 boxes at conf 0.10 on 6/6
   frames. Weights that see nothing anywhere cannot answer a question about
   one viewpoint.

The measurement that DOES answer it is class-agnostic, on the SHIPPING
detector, WITH a null baseline -- see `.claude/context/measured-bars.md`
§24. Result: at IoU 0.5 the shipping model scores 9.7 % against a
whole-frame null model's 15.3 %, and the `gate` class specifically is 0.0 %
at every threshold. The structure is not localised.

The water statistics below are still useful and are kept. The model section
now runs the honest scorer.
"""
import sys, glob, random, contextlib, io, os, cv2
sys.path.insert(0,'tools'); sys.path.insert(0,'src/duburi_vision')
from duburi_vision.underwater import analyse_frames
from recall_matrix import conf_sweep
from ultralytics import YOLO
R='/home/fh1m/Work/Projects/Duburi/2025/raw_images'
D='/home/fh1m/Work/Projects/Duburi/2025/datasets'; M='/home/fh1m/Music/detect'

print('\n  === A2. RETURN -- water ===')
for tag,d in (('final_fun/RETURN    ',f'{R}/final_fun/RETURN'),
              ('final_fun/RETURN_tmp',f'{R}/final_fun/RETURN_tmp'),
              ('datasets/RETURN     ',f'{D}/RETURN/backside/train/images')):
    f=sorted(glob.glob(d+'/*.jpg')+glob.glob(d+'/*.png'))
    if not f: print(f'  {tag} none'); continue
    random.seed(11)
    ims=[im for im in (cv2.imread(p) for p in random.sample(f,min(60,len(f))))
         if im is not None and im.ndim==3]
    a=analyse_frames(ims)
    print(f'  {tag} n={len(f):5d}  sharp {a.sharpness:6.0f}  sat {a.saturation:5.1f}'
          f'  contr {a.contrast:5.1f}  bright {a.brightness:5.1f}  cast {a.cast:+6.1f}')

print('\n  === A2. RETURN -- is there a model that reads it? ===')
print('  ⛔ the name-matched sweep below CANNOT answer this -- see the module')
print('     docstring. It is kept only to show what it actually measures.')
cands=[d for d in os.listdir(M) if os.path.isdir(f'{M}/{d}/weights')]
gate=[d for d in cands if 'gate' in d]
print(f'  no "return" model exists; testing the {len(gate)} GATE models '
      f'(class is `gate_backward`)\n')
best=None
for mn in sorted(gate):
    try:
        with contextlib.redirect_stdout(io.StringIO()),contextlib.redirect_stderr(io.StringIO()):
            m=YOLO(f'{M}/{mn}/weights/best.pt')
            rows,n=conf_sweep(m,f'{D}/RETURN/backside',640,80,[0.10,0.25])
    except Exception as e:
        print(f'  {mn:<30} FAILED {type(e).__name__}'); continue
    if not n: continue
    r=rows[0]
    print(f'  {mn:<30} n={n:3d}  R {100*r["recall"]:5.1f}%  P {100*r["precision"]:5.1f}%')
    if best is None or r['recall']>best[1]: best=(mn,r['recall'])
if best: print(f'\n  best on RETURN: {best[0]}  R {100*best[1]:.1f}%')
print('\n  ^ 0.0 % here means "no model declares a gate_backward class",')
print('    NOT "no model sees the gate". The real answer is measured-bars')
print('    §24: class-agnostic, on the shipping HEF, against a null model.')
