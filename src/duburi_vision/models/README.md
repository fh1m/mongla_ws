# duburi_vision / models

Drop YOLO26 weight files here. Ultralytics will auto-download on first
use if the path doesn't exist locally, but you'll want them cached in
this folder for offline pool sessions.

## Pretrained YOLO26 detection weights

| File           | Size    | Notes                                        |
| -------------- | ------- | -------------------------------------------- |
| `yolo26n.pt`   | ~5 MB   | Default. Fast on RTX 2060 + Jetson Orin.     |
| `yolo26s.pt`   | ~19 MB  | A notch more accurate.                       |
| `yolo26m.pt`   | ~42 MB  | Decent on Jetson Orin Nano.                  |
| `yolo26l.pt`   | ~51 MB  | Desktop / training rig.                      |
| `yolo26x.pt`   | ~113 MB | Desktop only.                                |

References:
- https://docs.ultralytics.com/models/yolo26
- https://docs.ultralytics.com/guides/yolo26-training-recipe/

## Custom-trained weights

Once we have RoboSub-class data (gate, buoys, dropper marker, torpedo
target, etc.), train with the YOLO26 recipe and drop the resulting
`best.pt` here. The detector node accepts any ultralytics-compatible
weight file via `model_path`.
