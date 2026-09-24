"""Read a recorded session and report what it is WORTH as a fixture.

A bag is not a fixture because it is large. It is a fixture because it contains
the transitions the thing under test has to survive. So this reports the gap
structure -- how often the detector lost the target, and for how long -- rather
than message counts, which say only that the graph was running.
"""
import sys
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from vision_msgs.msg import Detection2DArray

bag = sys.argv[1]
r = SequentialReader()
r.open(StorageOptions(uri=bag, storage_id='mcap'),
       ConverterOptions('cdr', 'cdr'))
types = {t.name: t.type for t in r.get_all_topics_and_types()}

det, lock = [], []
while r.has_next():
    topic, data, t = r.read_next()
    if topic.endswith('forward/detections'):
        m = deserialize_message(data, Detection2DArray)
        det.append((t / 1e9, len(m.detections)))
    elif topic.endswith('forward/lock'):
        m = deserialize_message(data, Detection2DArray)
        lock.append((t / 1e9, len(m.detections)))

if not det:
    print('NO DETECTION MESSAGES -- not a usable fixture'); sys.exit(1)

t0 = det[0][0]
dur = det[-1][0] - t0
hits = sum(1 for _, n in det if n)
print(f'duration        {dur:.1f} s')
print(f'detection msgs  {len(det)}  ({hits} carried a box, '
      f'{100*hits/len(det):.0f}%)')

# Gap = a maximal run of empty detection messages. This is the quantity the
# lock ladder exists to cover, so it is the quantity that decides whether this
# recording can test it.
gaps, run = [], None
for ts, n in det:
    if not n:
        run = run or ts
    elif run is not None:
        gaps.append(ts - run); run = None
if run is not None:
    gaps.append(det[-1][0] - run)

real = [g for g in gaps if g >= 0.2]
print(f'gaps >= 0.2 s   {len(real)}')
if real:
    real.sort()
    print(f'  shortest      {real[0]:.2f} s')
    print(f'  median        {real[len(real)//2]:.2f} s')
    print(f'  longest       {real[-1]:.2f} s')
    print(f'  total dark    {sum(real):.1f} s of {dur:.1f} s '
          f'({100*sum(real)/dur:.0f}%)')

if lock:
    held = sum(1 for _, n in lock if n)
    print(f'lock msgs       {len(lock)}  ({100*held/len(lock):.0f}% held a target)')
    # The interesting case: the ladder holding a target while the detector has
    # nothing. That is the fixture proving a rung below DETECTION did work.
    di = {round(ts, 1): n for ts, n in det}
    carried = sum(1 for ts, n in lock if n and di.get(round(ts, 1), 1) == 0)
    print(f'  held with NO detection: {carried} msgs '
          f'-- the ladder covering a real gap')
