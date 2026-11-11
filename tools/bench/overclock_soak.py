#!/usr/bin/env python3
"""Is a faster ARM clock worth keeping on Kutum?  A soak with the answer.

    python3 tools/bench/overclock_soak.py --seconds 1200 --hef MODEL.hef \
        --clip CLIP.mkv --out soak_2400.json

Runs the vision hot path on every core and the Hailo-8 flat out for the whole
run, and samples the Pi's own firmware every 5 s. Run once at the stock clock
and once at the candidate; compare the two files.

Bar, written before either run (2026-11-11). The faster clock is kept only if
ALL hold over 20 minutes, open air, fan as fitted:

  throttle   `vcgencmd get_throttled` reads 0x0 at every sample (no
             under-voltage, no frequency cap, no soft temperature limit, ever)
  heat       the hottest sample is under 80 C, where the firmware starts
             slowing the cores
  clock      the ARM clock the firmware reports is the one asked for at every
             sample (a clock that sags under load is not an overclock)
  gain       work done per second on the CPU load is at least 10 % above the
             stock run (the clock rises 16.7 %; less than 10 % of it arriving
             means something else is the limit)
  chip       the Hailo-8's inference rate is not lower than the stock run by
             more than 1 %

Falsified by any one of them. A pass here is an open-air bench pass only: the
sealed hull is hotter and is a separate run.
"""
import argparse
import json
import multiprocessing as mp
import subprocess
import time

import numpy as np


def vc(*args):
    return subprocess.run(['vcgencmd', *args], capture_output=True,
                          text=True).stdout.strip()


def cpu_worker(stop, count, frame_path):
    import cv2
    # One thread a worker: left at its default each of the four workers
    # starts four threads, sixteen on four cores.
    cv2.setNumThreads(1)
    # A real frame: on noise every pixel is a corner, which is not the load
    # the vehicle runs (measured-bars 82).
    frame = np.load(frame_path)
    ok, jpg = cv2.imencode('.jpg', frame)
    n = 0
    while not stop.is_set():
        img = cv2.imdecode(jpg, cv2.IMREAD_COLOR)
        big = cv2.resize(img, (640, 640))
        cv2.cvtColor(big, cv2.COLOR_BGR2RGB)
        grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        cv2.goodFeaturesToTrack(grey, 200, 0.01, 7)
        n += 1
        if n % 20 == 0:
            with count.get_lock():
                count.value += 20


def hailo_worker(stop, count, hef):
    from hailo_platform import (FormatType, HailoSchedulingAlgorithm, VDevice)
    p = VDevice.create_params()
    p.scheduling_algorithm = HailoSchedulingAlgorithm.ROUND_ROBIN
    with VDevice(p) as vd:
        m = vd.create_infer_model(hef)
        m.input().set_format_type(FormatType.UINT8)
        m.output().set_format_type(FormatType.FLOAT32)
        h, w = m.input().shape[:2]
        img = np.zeros((h, w, 3), np.uint8)
        with m.configure() as c:
            b = c.create_bindings()
            b.input().set_buffer(img)
            b.output().set_buffer(np.empty(m.output().shape, np.float32))
            n = 0
            while not stop.is_set():
                c.run([b], 5000)
                n += 1
                if n % 20 == 0:
                    with count.get_lock():
                        count.value += 20


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=1200)
    ap.add_argument('--hef', required=True)
    ap.add_argument('--clip', required=True)
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    stop = mp.Event()
    cpu, chip = mp.Value('q', 0), mp.Value('q', 0)
    import cv2
    cap = cv2.VideoCapture(a.clip)
    cap.set(cv2.CAP_PROP_POS_FRAMES, 100)
    ok, frame = cap.read()
    assert ok, a.clip
    np.save('/tmp/soak_frame.npy', cv2.resize(frame, (640, 360)))
    procs = [mp.Process(target=cpu_worker, args=(stop, cpu, '/tmp/soak_frame.npy'))
             for _ in range(4)]
    procs.append(mp.Process(target=hailo_worker, args=(stop, chip, a.hef)))
    for p in procs:
        p.start()
    asked = vc('get_config', 'arm_freq')
    rows, t0 = [], time.monotonic()
    while time.monotonic() - t0 < a.seconds:
        time.sleep(5)
        try:
            fan = open(subprocess.run(
                'ls /sys/class/hwmon/hwmon*/fan1_input', shell=True,
                capture_output=True, text=True).stdout.split()[0]).read().strip()
        except Exception:
            fan = ''
        rows.append({
            't': round(time.monotonic() - t0, 1),
            'temp': float(vc('measure_temp').split('=')[1].split("'")[0]),
            'throttled': vc('get_throttled').split('=')[1],
            'clock': int(vc('measure_clock', 'arm').split('=')[1]),
            'volts': vc('measure_volts', 'core').split('=')[1],
            'fan': fan, 'cpu': cpu.value, 'chip': chip.value})
        json.dump({'asked': asked, 'rows': rows}, open(a.out, 'w'))
    stop.set()
    for p in procs:
        p.join(10)
    span = rows[-1]['t'] - rows[0]['t']
    summary = {
        'asked': asked, 'samples': len(rows),
        'throttled_values': sorted({r['throttled'] for r in rows}),
        'temp_max': max(r['temp'] for r in rows),
        'temp_last': rows[-1]['temp'],
        'clock_min_mhz': round(min(r['clock'] for r in rows) / 1e6),
        'clock_max_mhz': round(max(r['clock'] for r in rows) / 1e6),
        'cpu_per_s': round((rows[-1]['cpu'] - rows[0]['cpu']) / span, 2),
        'chip_per_s': round((rows[-1]['chip'] - rows[0]['chip']) / span, 2),
        'fan_max': max(r['fan'] for r in rows),
        'volts': sorted({r['volts'] for r in rows})[-1],
    }
    json.dump({'asked': asked, 'summary': summary, 'rows': rows},
              open(a.out, 'w'))
    print(json.dumps(summary))


if __name__ == '__main__':
    main()
