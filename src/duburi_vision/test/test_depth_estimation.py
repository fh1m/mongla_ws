#!/usr/bin/env python3
"""Standalone tests for depth estimation components.

Usage:
    python test_depth_estimation.py [model_path]

    model_path  Path to the ONNX model file.
                Defaults to /home/fh1m/Envs/dockers/auv-ros2/model.onnx

Exit codes:
    0  All tests passed.
    1  One or more tests failed.
"""

import math
import sys

import numpy as np

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

_PASSED: list[str] = []
_FAILED: list[str] = []


def _report(name: str, ok: bool, detail: str = '') -> None:
    tag = 'PASS' if ok else 'FAIL'
    line = f'[{tag}] {name}'
    if detail:
        line += f'  — {detail}'
    print(line)
    (_PASSED if ok else _FAILED).append(name)


# ---------------------------------------------------------------------------
# Test 1: onnxruntime importable
# ---------------------------------------------------------------------------

def test_onnxruntime_import() -> None:
    name = 'onnxruntime importable'
    try:
        import onnxruntime  # noqa: F401
        _report(name, True, f'version={onnxruntime.__version__}')
    except ImportError as exc:
        _report(name, False, str(exc))


# ---------------------------------------------------------------------------
# Test 2: bbox-area fallback (no ROS, no model)
# ---------------------------------------------------------------------------

def test_bbox_area_fallback() -> None:
    name = 'bbox_area_fallback correctness'
    try:
        # Import only the pure function — does not trigger rclpy or ROS.
        # We add the package root to sys.path if needed.
        import importlib
        import importlib.util
        import os

        pkg_dir = os.path.join(
            os.path.dirname(__file__),
            '..',
            'duburi_vision',
        )
        pkg_dir = os.path.realpath(pkg_dir)

        spec = importlib.util.spec_from_file_location(
            'duburi_vision.depth.depth_estimation_node',
            os.path.join(pkg_dir, 'depth', 'depth_estimation_node.py'),
        )
        assert spec is not None and spec.loader is not None
        mod = importlib.util.module_from_spec(spec)
        # Stub out heavy imports that depth_estimation_node pulls at module level
        # so we can load the file without a live ROS environment.
        import types
        _dummy_class = type('_Stub', (), {})
        for stub in ('rclpy', 'sensor_msgs', 'std_msgs', 'vision_msgs'):
            if stub not in sys.modules:
                sys.modules[stub] = types.ModuleType(stub)

        rclpy_node = types.ModuleType('rclpy.node')
        rclpy_node.Node = _dummy_class  # type: ignore[attr-defined]
        sys.modules.setdefault('rclpy.node', rclpy_node)

        rclpy_qos = types.ModuleType('rclpy.qos')
        rclpy_qos.QoSProfile = _dummy_class           # type: ignore[attr-defined]
        rclpy_qos.QoSReliabilityPolicy = _dummy_class  # type: ignore[attr-defined]
        sys.modules.setdefault('rclpy.qos', rclpy_qos)

        for msg_stub in ('sensor_msgs.msg', 'std_msgs.msg', 'vision_msgs.msg'):
            m = types.ModuleType(msg_stub)
            for attr in ('Image', 'Float32MultiArray', 'Detection2DArray'):
                setattr(m, attr, _dummy_class)
            sys.modules.setdefault(msg_stub, m)

        if 'cv_bridge' not in sys.modules:
            cvb = types.ModuleType('cv_bridge')
            cvb.CvBridge = _dummy_class  # type: ignore[attr-defined]
            sys.modules['cv_bridge'] = cvb

        # cv2 must return sensible values — use the real one if present,
        # otherwise create a minimal stub.
        try:
            import cv2  # noqa: F401
        except ImportError:
            cv2_stub = types.ModuleType('cv2')
            cv2_stub.COLOR_BGR2RGB = 4  # type: ignore[attr-defined]
            cv2_stub.INTER_LINEAR  = 1  # type: ignore[attr-defined]
            def _cvtColor(src, code):   # noqa: N802
                return src
            def _resize(src, dsize, interpolation=1):  # noqa: N802
                return src
            cv2_stub.cvtColor = _cvtColor  # type: ignore[attr-defined]
            cv2_stub.resize   = _resize    # type: ignore[attr-defined]
            sys.modules['cv2'] = cv2_stub

        spec.loader.exec_module(mod)  # type: ignore[union-attr]
        fn = mod._bbox_area_fallback  # type: ignore[attr-defined]

        # Test case: 100×80 px bbox inside a 640×480 image.
        result = fn(320, 240, 100, 80, 640, 480)
        expected = math.sqrt((100 / 640) * (80 / 480))  # ≈ 0.1622

        ok = (
            isinstance(result, float)
            and 0.0 <= result <= 1.0
            and abs(result - expected) < 1e-5
        )
        _report(name, ok, f'got={result:.6f} expected={expected:.6f}')
    except Exception as exc:
        _report(name, False, repr(exc))


# ---------------------------------------------------------------------------
# Test 3: ONNX model loads and produces valid output
# ---------------------------------------------------------------------------

def test_onnx_model(model_path: str) -> None:
    base_name = f'ONNX model ({model_path})'

    # 3a: model file exists
    import os
    if not os.path.isfile(model_path):
        _report(f'{base_name} — file exists', False, 'file not found; skipping inference tests')
        return
    _report(f'{base_name} — file exists', True)

    # 3b: onnxruntime available
    try:
        import onnxruntime as ort
    except ImportError as exc:
        _report(f'{base_name} — onnxruntime available', False, str(exc))
        return
    _report(f'{base_name} — onnxruntime available', True)

    # 3c: session loads with CPUExecutionProvider
    try:
        opts = ort.SessionOptions()
        opts.inter_op_num_threads = 2
        opts.intra_op_num_threads = 2
        session = ort.InferenceSession(
            model_path, sess_options=opts, providers=['CPUExecutionProvider'])
        input_name  = session.get_inputs()[0].name
        output_name = session.get_outputs()[0].name
        _report(f'{base_name} — session loads', True,
                f'input={input_name!r} output={output_name!r}')
    except Exception as exc:
        _report(f'{base_name} — session loads', False, repr(exc))
        return

    # 3d: inference runs on dummy 364×364 input
    try:
        dummy = np.random.rand(1, 3, 364, 364).astype(np.float32)
        outputs = session.run([output_name], {input_name: dummy})
        raw = outputs[0]
        _report(f'{base_name} — inference runs', True, f'raw shape={raw.shape}')
    except Exception as exc:
        _report(f'{base_name} — inference runs', False, repr(exc))
        return

    # 3e: output shape is 2-D or 3-D and consistent with 364×364
    depth = raw.squeeze().astype(np.float32)
    shape_ok = depth.ndim in (2, 3) and 364 in depth.shape
    _report(f'{base_name} — output shape valid',
            shape_ok, f'squeezed shape={depth.shape}')

    # 3f: output values are finite
    finite_ok = bool(np.all(np.isfinite(depth)))
    _report(f'{base_name} — output values finite', finite_ok,
            '' if finite_ok else f'NaN/Inf count={np.sum(~np.isfinite(depth))}')

    if not finite_ok:
        return

    # 3g: normalise (invert → subtract min → divide by range) and check [0, 1]
    inverted = -depth
    d_min, d_max = float(inverted.min()), float(inverted.max())
    rng = d_max - d_min
    if rng > 1e-6:
        normed = (inverted - d_min) / rng
    else:
        normed = np.full_like(inverted, 0.5)

    norm_ok = (
        float(normed.min()) >= -1e-6
        and float(normed.max()) <= 1.0 + 1e-6
    )
    _report(f'{base_name} — normalised values in [0, 1]',
            norm_ok, f'min={normed.min():.4f} max={normed.max():.4f}')


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def main() -> int:
    default_model = '/home/fh1m/Envs/dockers/auv-ros2/model.onnx'
    model_path = sys.argv[1] if len(sys.argv) > 1 else default_model

    print('=' * 60)
    print('depth_estimation tests')
    print(f'model: {model_path}')
    print('=' * 60)

    test_onnxruntime_import()
    test_bbox_area_fallback()
    test_onnx_model(model_path)

    print('=' * 60)
    print(f'Results: {len(_PASSED)} passed, {len(_FAILED)} failed')
    if _FAILED:
        print('Failed tests:')
        for f in _FAILED:
            print(f'  - {f}')
    print('=' * 60)

    return 0 if not _FAILED else 1


if __name__ == '__main__':
    sys.exit(main())
