# The machine-vision pipeline — raw video to a model on the vehicle, and every control over it during a run

A detector is not trained once. It is the output of a loop: record, choose
frames, label, train, score, compile, check on the vehicle, fly, and go round
again with the frames the last model got wrong. This page is that loop in the
order it runs, with the tool that does each step, and then the list of
everything about a model that can be changed while the vehicle is in the
water.

**Key point:** the models in `src/mongla_vision/models/` are development
harnesses. The models that matter are trained and swapped on water days, so
every step here keys on the model's **stem** and its sidecar, never on a
class list written somewhere else.

**State.** Steps 1 to 8 RUN TODAY on the dev box and on Kutum against archive
footage. Nothing on this page has been checked in water on the current
platform.

## The loop

| step | what happens | tool | what it must show before the next step |
|---|---|---|---|
| 1 | record | `tools/pool/pool_record.sh`, the camera nodes | raw frames from the camera that will fly, in the water it will fly in |
| 2 | inventory | `tools/bench/footage_inventory.py`, `tools/bench/dataset_survey.py` | every clip with its venue, date and water; which are clean |
| 3 | choose frames | `tools/bench/label_contact_sheet.py`, `tools/bench/mine_hard_negatives.py` | frames worth labelling: the prop at every range and angle, and the frames the current model is confidently wrong about |
| 4 | label | any YOLO-format labeller | `cls cx cy w h`, normalised, one file per image |
| 5 | split and train | `yolo detect train` | a `.pt`, trained on sessions the validation set never saw |
| 6 | score the float model | `tools/studies/recall_matrix.py`, `conf_bar.py`, `negative_clip_check.py`, `domain_gap.py`, `detector_viewpoint_check.py` | recall per water, the bar where empty water stops reading as the prop, and what it does on video of the same run |
| 7 | write the sidecar, compile | `tools/bench/hailo_compile.sh <stem>` | a `.hef` whose INT8 scores are within 0.10 of its own float model |
| 8 | check on the vehicle | `ros2 run mongla_vision vision_check`, `tools/studies/composed_lock_harness.py`, `tools/bench/hailo_nms_floor.py` | detections on real footage, through the real graph, at the bar from step 6 |
| 9 | fly, record, repeat | step 1 | the misses of this model are the training set of the next |

### 1. Record

Record from the camera that will fly. A model trained on one camera and flown
on another loses recall for reasons that look like water
(`detector_viewpoint_check.py` measures it). Record raw: the stack applies no
image preprocessing, so the training frames must not have any either.

The archive on the dev box is found through `tools/archive_root.py`; never
conclude a clip is absent from one `find`.

### 2. Inventory before anything else

`footage_inventory.py` lists every clip with its venue and date and whether
it is clean. `dataset_survey.py` describes each labelled set by its water.
The point of both is the split in step 5: a session is the unit, and a
session has to be known before it can be held out.

### 3. Choose frames

- **Positives across the approach.** The prop small and far, large and near,
  cut by the frame edge, and seen from the side. `label_contact_sheet.py`
  renders each clip's strongest detection of a class so a person can pick.
- **Hard negatives.** `mine_hard_negatives.py` pulls the frames the current
  model is most confidently wrong about. These are worth more than any
  number of extra easy positives.
- **Empty water from the venue.** Without it the model has never seen what
  the absence of the prop looks like there.

**Warning: image preprocessing stays off**, in the dataset and on the
vehicle. Measured in 17 configurations across four props and three venues:
never positive, and on the gate it destroyed 95 % of detections
(`underwater-vision.md`). A test keeps it off.

### 4. Label

YOLO format. Label the whole visible prop, including the part a pole or the
frame edge cuts. A class name here is the name a mission will type; it goes
into the sidecar in step 7 and nowhere else.

### 5. Split and train

```
<data root>/<task>/
    images/{train,val}/*.jpg
    labels/{train,val}/*.txt
    <task>.yaml
```

```bash
env -u PYTHONPATH yolo detect train model=yolo11n.pt data=<task>.yaml \
    epochs=100 imgsz=640 seed=0
```

- **Split by session, never by frame.** Neighbouring frames of one clip on
  both sides make validation recall meaningless: a 361-frame gate and flare
  set split by frame scored mAP50 0.995 on two different backbones and told
  them apart by nothing.
- **Keep the data out of `/tmp`.** It is a RAM disk on the dev box and
  training is killed for memory with no message. `tools/data_root.py` says
  where it goes.
- **640 is the input the chip runs.** Train at the size that will be
  compiled.
- **Which backbone** is an open measurement (issue #46): YOLOv8s against
  YOLO11n on the same data. The answer has to come from held-out sessions
  and from the frame rate on the chip, not from validation mAP.

### 6. Score the float model, before compiling

| question | tool | the number it gives |
|---|---|---|
| does it find the prop, per water? | `recall_matrix.py` | recall against ground-truth labels, per regime |
| what `conf` should it run at? | `conf_bar.py`, `negative_clip_check.py` | detection rate on clips with the prop and on clips without, at each bar; the bar where the second reaches zero |
| does it hold on video, not only stills? | `domain_gap.py` | the same model on its training stills and on video of the same run |
| does it hold from another camera or angle? | `detector_viewpoint_check.py` | recall on a viewpoint it was not trained for |

**Warning: render the negatives before scoring them.** Two clips picked as
shark negatives turned out to hold the shark board (measured-bars 72).

The bar this step produces is the model's `recommended_conf`. It belongs to
the weights, so it is written in the sidecar beside them and read from there.

### 7. Sidecar and compile

`<stem>.yaml` beside `<stem>.pt`: `names` exactly as trained, and the
`separation:` block with the measured bar. **Warning:** without the sidecar
the detector has no class names and returns nothing while looking healthy,
so both the compile and the detector refuse to run without it.

```bash
./tools/bench/hailo_compile.sh --check
./tools/bench/hailo_compile.sh <stem> --nms-score-th 0.05
```

The compile calibrates INT8 on 1024 real frames from the archive (fewer
silently drops the accuracy optimisation and corrupts scores, B-61), measures
the INT8 model against its own float on held-out frames, and refuses a build
that loses more than 0.10 confidence. It needs the DFC 3.x that pairs with
the vehicle's HailoRT 4.24, and a clean `PYTHONPATH`; the wrapper handles
both.

### 8. Check on the vehicle

`./build_mongla.sh` mirrors `~/models` in; `kutum/deploy.sh` puts it on
Kutum. Then, on real footage and through the real graph:

```bash
ros2 run mongla_vision vision_check --camera forward --require-class <class>
python3 tools/studies/composed_lock_harness.py --help
```

Never call a model working without seeing detections on real footage at the
bar written in its sidecar.

### 9. Go round again

After a run, the recorded frames go back to step 2. What to look for:

| what the run showed | what it means | where to go |
|---|---|---|
| the prop in frame, no box, scores just under `conf` | the bar is too high for this water | lower `conf` live (below), then re-measure the bar in step 6 |
| the prop in frame, nothing at any score | the model has not seen this view or water | step 3: label these frames |
| boxes on empty water | the model has not seen this absence | step 3: these are hard negatives |
| the right box, the wrong class | an identity fault no threshold fixes | step 3 and 4: more of both classes, same water |
| good on stills, poor on video | a domain gap | step 6, `domain_gap.py`; train on video frames |

## Control during a run

Everything below changes on a running vehicle, from a mission or from a
shell. Nothing here needs a relaunch.

| what | from a mission | from a shell (`ros2 param set /mongla_detector_<camera> ...`) | takes effect | cost |
|---|---|---|---|---|
| confidence, all models | `mongla.set_conf(0.30)` | `conf 0.30` | next frame | none |
| confidence, one model | `mongla.set_conf(0.55, model='<name>')` | `model_conf '<name>=0.55'` | next frame | none |
| which classes are reported | `mongla.set_classes('gate')` | `classes gate` | next frame | none |
| which model runs | `mongla.set_model('<stem>')` | `active_model <name>` | next frame | none: every model in `models:=` is loaded at launch |
| several models at once | `mongla.set_model(['a', 'b'])` | `active_model a,b` | next frame | the chip runs one graph at a time: measured 37.5 Hz for a pair against 95 Hz for one |
| association floor (weak boxes that may keep a lock, never start one) | `mongla.set_detector(assoc_conf=0.10)` | `assoc_conf 0.10` | next frame | none |
| most boxes per frame | `mongla.set_detector(max_det=10)` | `max_det 10` | next frame | none |
| lowest score the chip returns | `mongla.set_detector(nms_floor=0.02)` | `nms_floor 0.02` | about 65 to 90 ms later | the detector misses those frames, once |
| overlap above which two boxes are one object | `mongla.set_detector(nms_iou=0.5)` | `nms_iou 0.5` | about 65 to 90 ms later | the same |
| mask decoding (segmentation models) | `mongla.set_detector(masks=False)` | `masks false` | next frame | none |
| stop and start inference | `mongla.pause_detector()` / `resume_detector()` | `paused true` | next frame | none |
| what the lock ladder holds | `mongla.lock_class('gate')` | | next frame | none |

How the three score settings stack: the chip's NMS returns every box at or
above `nms_floor` (0.05). `assoc_conf` and above reach the tracker and the
lock ladder. `conf` and above reach the verbs that steer. So `conf` moves
freely from 0.05 to 1.0 during a run with no cost, and `nms_floor` is only
needed to go below 0.05.

**Key point:** the score threshold compiled into a `.hef` is a default, not a
limit. Until 2026-11-11 this stack believed a model compiled at 0.200 could
not run below it without a recompile; the detector now configures every graph
at 0.05 and can move that floor live (measured-bars 79, B-112).

A setting the detector cannot honour is **rejected**, with the reason, and
the mission call raises. `iou` is the example: it is read once when a `.pt`
model is built, so a live change is refused and the message names `nms_iou`.

### What cannot change during a run, and why

| what | why | how to change it |
|---|---|---|
| a model that was not loaded at launch | loading configures the chip, and a model that fails to load mid-run is worse than one that was never asked for | list it in `models:=` at launch; switching among loaded models is free |
| the input size (640) | compiled into the graph | recompile |
| the weights | compiled into the graph | train and compile; then `set_model` if it was loaded at launch |
| the classes a model knows | trained in | train |
| image preprocessing | held off by a test, on measurement | not a tuning setting |

## Where the detail is

- [`hailo-vision.md`](hailo-vision.md): the chip, measured.
- [`underwater-vision.md`](underwater-vision.md): what the water does to a model, from our own footage.
- [`detection-continuity.md`](detection-continuity.md): what happens between detections.
- `.claude/skills/train-model/SKILL.md`: the short form of steps 5 to 8.
- [`../measured-bars.md`](../measured-bars.md): every number above, with its method.
