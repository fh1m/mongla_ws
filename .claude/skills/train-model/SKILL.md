---
name: train-model
description: YOLO11 fine-tune workflow for a new RoboSub detection task (pipes, bin symbols, torpedo board openings). Use when building a custom detection model for a competition task.
---

# Train a YOLO11 task model

Fine-tune `yolo11n.pt` for a RoboSub detection target and wire it into the detector.
Verify current Ultralytics CLI/args against `docs.ultralytics.com` before running — the
`train`/`predict` arg surface drifts between releases.

## 1. Dataset layout

```
datasets/<task>/
├── images/{train,val}/*.jpg
├── labels/{train,val}/*.txt      # YOLO format: cls cx cy w h (normalized)
└── <task>.yaml                   # paths + names: [class0, class1, ...]
```

Aim for ~100–200 images/class (roadmap guidance), balanced, with underwater-realistic
augmentation (color shift, glare, motion blur). Split ~80/20 train/val.

## 2. Train

```bash
yolo train data=datasets/<task>/<task>.yaml model=yolo11n.pt \
     epochs=100 imgsz=640 half=True
```

`yolov11n` keeps the 30 Hz / ~7 ms Jetson budget. Step up to `s/m` only if recall is
insufficient and the frame budget allows (ask `vision-model-reviewer`).

## 3. Drop weights + wire detector

```bash
cp runs/detect/train/weights/best.pt \
   src/duburi_vision/models/<task>_nano_100ep.pt
```

Edit `src/duburi_vision/config/detector.yaml`: set `model` to the new weights and
`classes` to the trained class names.

## 4. Verify

```bash
./build_dubomini.sh
ros2 launch duburi_vision cameras_.launch.py
ros2 run duburi_vision vision_check --camera forward --require-class <class>
```

Confirm detections appear with sane `conf` before any in-water mission run.
