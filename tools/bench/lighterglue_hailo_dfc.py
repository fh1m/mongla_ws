"""Step 2 of 2: parse, quantise and emulate a LighterGlue half on the Hailo-8.

Runs in the DFC venv (no torch): `tools/bench/hailo_compile.sh --check` shows
how to enter it. Reads what lighterglue_hailo_prep.py wrote.

  stage parse     ONNX -> HAR; fails here if the graph has an op the parser refuses
  stage optimize  8-bit quantisation, calibrated on the real pairs
  stage emulate   the quantised model (InferenceContext.SDK_QUANTIZED, which the
                  forum reports is bit-faithful to the silicon) on the same pairs;
                  writes the outputs for the scorer
  stage compile   HAR -> HEF, the context allocator (the step that fails on
                  graphs that are too big; give it a timeout)

    env -u PYTHONPATH ~/hailo-dfc-venv/bin/python lighterglue_hailo_dfc.py \\
        --dir <prep out> --half 1 --stage parse
"""
from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np


def nhwc(a):
    return np.ascontiguousarray(np.transpose(a, (0, 2, 3, 1)))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--dir', required=True)
    ap.add_argument('--half', type=int, choices=(1, 2), required=True)
    ap.add_argument('--stage', choices=('parse', 'optimize', 'emulate', 'compile'), required=True)
    ap.add_argument('--arch', default='hailo8')
    a = ap.parse_args()
    from hailo_sdk_client import ClientRunner, InferenceContext

    name = f'half{a.half}'
    har_p, opt_p = (os.path.join(a.dir, f'{name}.{s}.har') for s in ('parsed', 'quantized'))
    t0 = time.time()
    if a.stage == 'parse':
        r = ClientRunner(hw_arch=a.arch)
        r.translate_onnx_model(os.path.join(a.dir, f'{name}.onnx'), name)
        r.save_har(har_p)
        print(f'[{name}] parsed in {time.time() - t0:.0f} s; layers: {len(r.get_hn_dict()["layers"])}')
        for n, l in list(r.get_hn_dict()['layers'].items())[:6]:
            print('   ', n, l.get('type'), l.get('input_shapes', ''))
        return 0

    z = np.load(os.path.join(a.dir, 'pairs.npz'))
    # Calibration and emulation inputs. Half 1 takes descriptors; half 2 takes
    # half 1's float outputs. The positional encodings are recomputed by prep
    # and stored as c0/s0/c1/s1 in pairs.npz.
    need = ('c0', 's0', 'c1', 's1')
    if not all(k in z for k in need):
        print('pairs.npz has no c0/s0/c1/s1: rerun prep after adding the encodings')
        return 2
    if a.half == 1:
        x0 = nhwc(np.transpose(z['d0'], (0, 2, 1)).reshape(len(z['d0']), 64, 32, 32))
        x1 = nhwc(np.transpose(z['d1'], (0, 2, 1)).reshape(len(z['d1']), 64, 32, 32))
    else:
        x0, x1 = nhwc(z['x0'][:, 0]), nhwc(z['x1'][:, 0])
    data = {'x0': x0, 'x1': x1, 'c0': nhwc(z['c0'][:, 0]), 's0': nhwc(z['s0'][:, 0]),
            'c1': nhwc(z['c1'][:, 0]), 's1': nhwc(z['s1'][:, 0])}

    r = ClientRunner(har=har_p if a.stage == 'optimize' else opt_p)
    layers = [n for n, l in r.get_hn_dict()['layers'].items() if l.get('type') == 'input_layer']
    layers.sort(key=lambda s: int(''.join(ch for ch in s.rsplit('/', 1)[-1] if ch.isdigit()) or 0))
    order = ['x0', 'x1', 'c0', 's0', 'c1', 's1']
    feed = {l: data[o] for l, o in zip(layers, order)}
    if a.stage == 'optimize':
        # Every matmul that consumes a softmax asks for a forced input range,
        # and the DFC prints the exact command. Take each one it asks for, add
        # it to the model script and run again, until it quantises.
        import re
        ranges, first, asked = {}, {}, {}                   # layer -> range in use, first suggestion, times asked

        def _script():
            return ''.join(f'quantization_param([{l}], force_range_in=[{", ".join(f"{x:.6g}" for x in v)}], '
                           f'force_range_index=[0,1])\n' for l, v in ranges.items())
        for attempt in range(80):
            r = ClientRunner(har=har_p)
            r.load_model_script(_script()) if ranges else None
            try:
                r.optimize(feed)
                break
            except Exception as e:                          # AccelerasUnsupportedError
                # The message holds a generic template ([layer_name]) first and
                # the command for the failing layer after it; take the latter.
                m = [x for x in re.findall(r'quantization_param\(\[([^\]]+)\], force_range_in=\[([^\]]+)\]',
                                           str(e)) if x[0] != 'layer_name']
                if not m:
                    print(f'[{name}] stopped on: {str(e)[:600]}')
                    return 1
                layer, v = m[-1][0], [float(x) for x in m[-1][1].split(',')]
                # The suggested range can exclude 0 (a softmax output reads
                # [0.001, 0.026]), which the DFC's own validator refuses and
                # then reports as KeyError('force_range_in'): every pair holds 0.
                # A layer asked about again means the suggestion was too small
                # ("does not support shift delta"); the DFC re-suggests a range
                # a few percent off the last one, so the range doubles instead.
                n = asked[layer] = asked.get(layer, 0) + 1
                if n > 8:
                    print(f'[{name}] {layer} still refuses at {2 ** 7}x its range: {str(e)[:400]}')
                    return 1
                first.setdefault(layer, [min(x, 0.0) if i % 2 == 0 else max(x, 0.0) for i, x in enumerate(v)])
                ranges[layer] = [x * 2 ** (n - 1) for x in first[layer]]
                print(f'[{name}] attempt {attempt}: {layer} x{2 ** (n - 1)} {ranges[layer]}')
        else:
            print(f'[{name}] gave up after 80 forced ranges')
            return 1
        r.save_har(opt_p)
        open(os.path.join(a.dir, f'{name}.model_script.alls'), 'w').write(_script())
        print(f'[{name}] quantised in {time.time() - t0:.0f} s with {len(ranges)} forced ranges')
        return 0
    if a.stage == 'emulate':
        outs = []
        with r.infer_context(InferenceContext.SDK_QUANTIZED) as ctx:
            for i in range(len(x0)):
                outs.append(r.infer(ctx, {l: v[i:i + 1] for l, v in feed.items()}))
        np.savez_compressed(os.path.join(a.dir, f'{name}.quantized_out.npz'),
                            **{f'o{j}': np.concatenate([o[j] if isinstance(o, (list, tuple)) else o
                                                        for o in [q for q in outs]], 0)
                               for j in range(len(outs[0]) if isinstance(outs[0], (list, tuple)) else 1)})
        print(f'[{name}] emulated {len(outs)} pairs in {time.time() - t0:.0f} s')
        return 0
    hef = r.compile()
    open(os.path.join(a.dir, f'{name}.hef'), 'wb').write(hef)
    print(f'[{name}] compiled in {time.time() - t0:.0f} s, {len(hef) / 1e6:.1f} MB')
    return 0


if __name__ == '__main__':
    sys.exit(main())
