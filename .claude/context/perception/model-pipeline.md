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
| 8 | Score the HEF on the chip | `tools/bench/hef_map.py`, `tools/bench/hailo_int8_shift.py` | held-out mAP50 on the chip; INT8 shift under the refusal bar |
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
map (`Cloth`, `pole`, a group box around several tubs). Colour is read after detection, not
trained as a class, because teams disagree on it.

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

The first held-out run (bead 5ew.2) trains yolov8n and yolo11n on the same split. Its bar is in
`mongla_data/holdout/<source>/BAR.md`, and its numbers go into [measured-bars](../measured-bars.md)
when it finishes. Until then this chapter claims no held-out number.
