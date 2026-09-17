"""Detector warmup is TIMED and logged: first inference vs steady, as numbers."""
import time
from unittest.mock import MagicMock

import numpy as np

from duburi_vision.detection import yolo as Y


def test_yolo_warmup_reports_cold_and_steady_ms():
    det = Y.YoloDetector.__new__(Y.YoloDetector)
    calls = {'n': 0}

    def predict(*a, **k):
        calls['n'] += 1
        time.sleep(0.03 if calls['n'] == 1 else 0.005)   # first pass pays setup
    det._model = MagicMock(predict=predict)
    det._imgsz, det._conf, det._iou, det._device = 64, 0.2, 0.5, 'cpu'
    det._half, det._max_det = False, 10
    det._log = MagicMock()
    det._do_warmup()
    cold, warm = det.warmup_ms
    assert cold > warm and cold >= 25.0
    assert 'warmup: first pass' in det._log.info.call_args[0][0]
