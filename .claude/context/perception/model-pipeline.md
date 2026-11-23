# The model pipeline, from footage to the chip

How a detector is made, checked and put on the vehicle, step by step, with the tool for each
step and the gate it has to pass. The detectors in `models/` are development harnesses; the real
ones are trained close to the event and swapped on water days. So this pipeline is a workflow
we run under time pressure, and every step has to be a command that can fail.

The research behind the order (sources, rejected options, what would reverse it) is the report
*YOLO on Hailo 8 model pipeline*, kept in `mongla_data/research/` beside the data. Its finding,
in one line: speed is settled, and the open question is whether a model trained in other pools
works in a pool it has never seen. Every step below is ordered by that question.

## What is settled, and what is not

| | state | evidence |
|---|---|---|
| yolov8n runs in one context, 319 FPS on the chip; yolo11n needs three, 98 FPS | RUNS TODAY | [measured-bars](../measured-bars.md) §98, [hailo-vision](hailo-vision.md) §3 |
| A random-split mAP50 of 0.98 to 0.995 says the model transfers to a new pool | **not shown** | a random split puts near-copies of training frames in the test set |
| INT8 costs 0.04 to 0.06 mAP50 on the chip against the `.pt` | RUNS TODAY | measured-bars §1 |
| The confidence bar belongs to the weights, in the sidecar | RUNS TODAY | measured-bars §59, `detection/hailo.py` |

## The steps

| # | step | tool | gate |
|---|---|---|---|
| 1 | Record each pool, with an empty-water segment | `tools/pool/pool_record.sh` | every run has frames with no prop |
| 2 | Inventory the footage | `tools/bench/footage_inventory.py` | each clip has a venue, a date, and clean or not |
| 3 | Label: the current model proposes, a person fixes | `tools/bench/label_contact_sheet.py` | every frame has a label file, empty ones included |
| 4 | Split by source, never by frame | `tools/studies/pool_holdout.py` | no frame of a held-out source in train, checked by image hash |
| 5 | Write the bar file **before** training | `BAR.md` beside the split | its timestamp precedes the run |
| 6 | Train | `yolo detect train`, from the last `best.pt` | held-out mAP50 and false positives per minute against the previous model |
| 7 | Compile, level 2, fine-tune off | `tools/bench/hailo_compile.sh` | the applied script matches the template; yolov8n reports one context |
| 8 | Score the HEF on the chip, then look at the boxes | `tools/bench/hef_map.py`, `tools/bench/hailo_int8_shift.py` | held-out mAP50 on the chip; INT8 shift under the refusal bar; a random sample (48+) of unmatched detections and missed labels, read by a person against the team's original class names, before the number is believed |
| 9 | Set the sidecar bar | `tools/studies/conf_bar.py` | `recommended_conf` measured on held-out frames, written in the ledger |
| 10 | Deploy the weights and sidecar together; keep the previous pair | `kutum/deploy.sh` | the previous pair is on the vehicle and the rollback was rehearsed |

**Key point: the fallback is a step, not a hope.** If a new compile is not finished by the time
set in the bar file, the previous HEF and sidecar fly, unchanged.

## Data from other teams

Other teams publish labelled sets of the same props from their own pools. They are the cheapest
held-out data there is, with two rules:

- **Keep each team's published split.** Their train, valid and test sets are used as they were
  published: never re-split, capped or shuffled. The held-out source contributes only its own
  test split; its train and valid are left out.
- **Drop copies.** Teams fork each other's datasets, so one frame can sit in two sets under two
  names. `pool_holdout.py` drops any training frame within 4 bits of a held-out frame's 64-bit
  difference hash. On the first real build it dropped 327 copies of 237 held-out test frames,
  308 of them in one other team's set; without it the held-out score would have read high.

Every team names the props its own way (`Red pail`, `red_tub`, `Blue Bucket`, `ember-biru` are
one drum), so the tool maps names to one coarse class set by keyword and drops what it cannot
map (`Cloth`, `pole`, a group box around several tubs). Colour is the open question: teams
disagree on it, and nothing in the stack reads a hue from a coarse box. So the tool builds
both, `--classes coarse` (gate, flare, drum) and `--classes fine` (the colour in the class,
taken from the team's own name; a frame whose prop names no colour is left out), and
`tools/studies/colour_ab.py` scores a fine detector against the coarse one plus a learned hue
reader on the same held-out pool, against a bar written before either trained.

The datasets live in `mongla_data/datasets/`, outside git, each with a `SOURCE.md` naming its
author and licence.

## Levers, in the order to try them

Each lever is a bead under epic `5ew` with its bar written in. They are tried only after step 4
gives a held-out number, because without it no lever can be ranked.

| lever | bead | when |
|---|---|---|
| held-out split and empty-water false positives | 5ew.2 | first; it tests the premise |
| hard negatives from each pool | 5ew.3 | if false positives appear on empty water |
| fine-tune from the last `best.pt`, early layers frozen | 5ew.4 | every retrain on a new pool |
| calibration set: 256 / 512 / 1024 frames from the new pool | 5ew.5 | every compile |
| read what the compiler applied (`.auto.alls`) | 5ew.6 | once per Dataflow Compiler version |
| noise and blur augmentation | 5ew.7 | if the held-out gap is large |
| distillation from a larger teacher | 5ew.8 | overnight, after the above |
| host NMS cost at the 0.05 floor | 5ew.9 | before any on-chip NMS switch |

**Rejected:** optimisation level 3 (exhausts 32 GB of RAM in reported cases), PyTorch QAT (the
compiler quantises again), unstructured sparsity (no evidence the compiler uses zeros), image
enhancement before detection (measured in 17 configurations, never positive).

## Results

The first held-out run (bead 5ew.2): yolov8n, 20 epochs, ten other sources in train, one team's
own test split held out (237 images). The bar is in `mongla_data/holdout/<source>/BAR.md`,
written before training.

| test set | mAP50 | gate | flare | drum |
|---|---|---|---|---|
| held-out team | 0.764 | 0.988 | 0.327 | 0.976 |
| in-pool: nine other teams' own test splits, mean / median | 0.886 / 0.964 | | | |

**Retracted: this run measured a class-map bug, not the model.** The first reading, from 16
boxes, was that the held-out team had not labelled most flares. A random 48 of the 185 unmatched
flare boxes, cropped tight, and a count against the team's ORIGINAL class names, showed instead:

- That team labels the orange flare (a coloured pole on a weighted base) `pole`, and
  `pool_holdout.py` dropped every `pole` as a gate post. 137 of the 185 "false" flare boxes sit on
  their `pole` boxes, and the model found 123 of their 132.
- A training team (`malocclusion`) does the same: its `flare` and `pole` are one kind of prop,
  so 4 182 flare boxes were removed from training and the frames taught "flare is background".
  It is also why that team's in-pool flare score read 0.08 against 0.84-1.00 for the others.
- Gate: 85 of 123 labelled gates found at confidence 0.25 (69 %). The misses are this team's
  gate (red and black frame on a white floor frame), a real transfer failure.
- No gate box was ever called a flare, or the reverse, where a box matched a label.

A bare `pole` now maps to flare (`gate pole` and `tiang` stay dropped); the self-test fails on
the old rule. The split is rebuilt (held-out flare labels 72 -> 204) and both families retrain on
it; the numbers above are kept only as the record of the bug.

**Key point: audit the labels before believing a held-out number, and audit enough of them.** Read
the unmatched boxes against the team's original class names, on a random sample big enough to
count (48, not 16), cropped so the object is visible. Sixteen boxes in file order were one video
sequence and gave a confident wrong cause.

**The rebuilt split, both families (2026-11-21, beads 5ew.2 and 5ew.1).** Same bar, same data,
COCO start, 20 epochs, seed 0. measured-bars 102 has every in-pool source.

| model | held-out mAP50 | gate | flare | drum | in-pool mean (9 sources) | gap |
|---|---|---|---|---|---|---|
| yolov8n | 0.882 | 0.992 | 0.680 | 0.975 | 0.922 | 0.040 |
| yolo11n | 0.872 | 0.992 | 0.687 | 0.937 | 0.925 | 0.053 |

- **Family: yolov8n stays.** yolo11n would have had to score 0.02 higher on the held-out team; it
  scored 0.010 lower (one seed), and yolov8n is the faster one on the chip (measured-bars 98).
- **Premise: inconclusive.** The bar confirms a held-out drop above 0.05 and rejects one under
  0.01. yolov8n's 0.040 sits between; yolo11n's 0.053 confirms. The drop is concentrated in the
  flare (0.68 against 0.84-1.00 in-pool), which is also where teams label the prop most
  differently, so this split cannot separate water from labelling convention.
- The next lever for the flare is the data, not the family: the flare is a thin coloured pole,
  and the held-out team's flare boxes are recalled at 0.53 (v8n) and 0.63 (11n).
