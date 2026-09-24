"""What the board's IMU says when the vehicle is NOT moving.

A vehicle at rest is the only condition outside water where the truth is known
exactly: every rate is zero and the only acceleration is gravity. Everything
the IMU reports beyond that is bias and noise -- the two numbers a filter needs
and the two this stack has never measured on its own hardware.

Reports, per axis:
  bias      the mean. At rest this is pure offset; the filter must estimate it
            or integrate it into unbounded heading drift.
  noise     the standard deviation of one sample.
  ARW/VRW   angle / velocity random walk, read off the Allan deviation at
            tau = 1 s. This is the sigma a process model should carry.
  stability the Allan deviation MINIMUM -- the floor below which averaging for
            longer stops helping, because bias drift has taken over.

Allan deviation rather than a plain standard deviation because the two answer
different questions: std says how much one sample scatters, Allan says how long
the sensor can be trusted to average. A filter tuned on std alone will believe
a gyro for far longer than the hardware earns.
"""
import sys

import numpy as np
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from sensor_msgs.msg import Imu

G = 9.80665


def allan_dev(x: np.ndarray, dt: float):
    """Overlapping Allan deviation. Returns (taus, sigmas)."""
    n = len(x)
    theta = np.cumsum(x) * dt            # integrate rate -> angle
    out_t, out_s = [], []
    m = 1
    while m <= (n - 1) // 2:
        tau = m * dt
        d = theta[2 * m:] - 2 * theta[m:-m] + theta[:-2 * m]
        if len(d) < 2:
            break
        out_t.append(tau)
        out_s.append(np.sqrt(np.mean(d ** 2) / (2 * tau ** 2)))
        m = int(np.ceil(m * 1.3))
    return np.array(out_t), np.array(out_s)


def read_imu(bag: str):
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id='mcap'),
           ConverterOptions('cdr', 'cdr'))
    t, g, a = [], [], []
    while r.has_next():
        topic, data, stamp = r.read_next()
        if not topic.endswith('/imu'):
            continue
        m = deserialize_message(data, Imu)
        t.append(stamp / 1e9)
        g.append((m.angular_velocity.x, m.angular_velocity.y,
                  m.angular_velocity.z))
        a.append((m.linear_acceleration.x, m.linear_acceleration.y,
                  m.linear_acceleration.z))
    return np.array(t), np.array(g), np.array(a)


def report(name, arr, dt, unit, scale=1.0):
    print(f'\n{name}   [{unit}]')
    print(f'{"axis":>5} {"bias":>12} {"noise 1σ":>12} '
          f'{"ARW @1s":>12} {"stability":>12} {"@ tau":>9}')
    for i, ax in enumerate('xyz'):
        v = arr[:, i] * scale
        taus, sig = allan_dev(v, dt)
        at1 = sig[np.argmin(np.abs(taus - 1.0))] if len(taus) else float('nan')
        j = int(np.argmin(sig)) if len(sig) else 0
        print(f'{ax:>5} {v.mean():12.6f} {v.std():12.6f} '
              f'{at1:12.6f} {sig[j]:12.6f} {taus[j]:8.1f}s')


def main():
    bag = sys.argv[1]
    t, g, a = read_imu(bag)
    if len(t) < 100:
        print('not enough IMU samples -- not a usable rest record')
        return 1

    dur = t[-1] - t[0]
    dt = dur / (len(t) - 1)
    print(f'samples   {len(t)}')
    print(f'duration  {dur:.1f} s')
    print(f'rate      {1/dt:.2f} Hz   (dt {dt*1e3:.2f} ms)')

    # A rest record is only valid if the vehicle was ACTUALLY at rest. The
    # gravity vector's magnitude is the check that costs nothing: it must sit
    # at 1 g and barely move. If it wandered, something knocked the bench and
    # every number below is measuring that instead of the sensor.
    mag = np.linalg.norm(a, axis=1)
    print(f'|accel|   {mag.mean():.4f} ± {mag.std():.4f} m/s²  '
          f'({mag.mean()/G:.4f} g)')
    if mag.std() > 0.5:
        print('  ⚠ the board MOVED during this record -- these are not '
              'rest statistics')

    report('gyro', g, dt, 'deg/s', scale=180.0 / np.pi)
    report('accel', a, dt, 'm/s²')

    # Heading drift is the consequence a reader actually cares about: an
    # unestimated z-bias integrates straight into yaw error.
    bz = abs(g[:, 2].mean()) * 180.0 / np.pi
    print(f'\nunestimated yaw-bias drift: {bz*60:.2f} deg/min '
          f'({bz*3600:.1f} deg/hour)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
