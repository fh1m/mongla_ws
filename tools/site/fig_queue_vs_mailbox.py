#!/usr/bin/env python3
"""Animate why a camera queue hands you an old frame, and a mailbox does not.

The model is the measurement in camera-latency.md section 1: a 60 fps camera,
a consumer that stalls for 400 ms, and the kernel's four V4L2 buffers. While
the consumer is away the queue fills with the first four frames of the stall
and the driver drops the rest; when it comes back, the queue hands it the
oldest. The mailbox keeps one slot and every new frame replaces it.

The animation steps the model frame by frame; the captions quote the measured
numbers (396.1 ms, 348.1 ms, 16.9 ms), not the model's, so the picture
explains and the ledger proves.

    python3 tools/site/fig_queue_vs_mailbox.py  ->  docs/imgs/community/queue-vs-mailbox.gif
"""
from __future__ import annotations

from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / 'docs' / 'imgs' / 'community' / 'queue-vs-mailbox.gif'

PERIOD_MS = 1000 / 60          # the camera, 60 fps
BUFFERS = 4                    # the kernel's V4L2 queue
STALL = (6, 30)                # consumer away from tick 6 to tick 30: 400 ms
TICKS = 40
W, H = 1200, 560
BG, PANEL, INK, MUTED, LINE = '#0b0f17', '#121826', '#e6e8ee', '#7a8194', '#232b3d'
FRESH, STALE, RED, GREEN = (139, 124, 246), (255, 69, 58), '#ff453a', '#34d399'
FONT = '/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf'
BOLD = '/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf'


def font(size, bold=False):
    try:
        return ImageFont.truetype(BOLD if bold else FONT, size)
    except OSError:
        return ImageFont.load_default()


F_TITLE, F_LABEL, F_SMALL, F_BIG = font(26, True), font(17, True), font(14), font(22, True)


def age_colour(age_ms):
    k = max(0.0, min(1.0, age_ms / 400.0))
    return tuple(int(a + (b - a) * k) for a, b in zip(FRESH, STALE))


def simulate():
    """Per tick: queue contents, mailbox contents, dropped count, what was read."""
    queue, mailbox, dropped, states = [], None, 0, []
    read_q = read_m = None
    for t in range(TICKS + 1):
        if len(queue) < BUFFERS:
            queue.append(t)
        else:
            dropped += 1
        mailbox = t
        stalled = STALL[0] <= t < STALL[1]
        if not stalled:
            read_q = queue.pop(0) if queue else None
            read_m = mailbox
        states.append(dict(t=t, queue=list(queue), mailbox=mailbox, dropped=dropped,
                           stalled=stalled, read_q=read_q, read_m=read_m))
    return states


def frame_box(d, x, y, seq, now, w=86, h=64):
    age = (now - seq) * PERIOD_MS
    d.rounded_rectangle([x, y, x + w, y + h], 10, fill=age_colour(age))
    d.text((x + w / 2, y + 20), f'#{seq}', font=F_LABEL, fill='#0b0f17', anchor='mm')
    d.text((x + w / 2, y + 44), f'{age:.0f} ms', font=F_SMALL, fill='#0b0f17', anchor='mm')


def lane(d, top, title, s, kind):
    d.rounded_rectangle([30, top, W - 30, top + 190], 16, fill=PANEL, outline=LINE)
    d.text((54, top + 18), title, font=F_LABEL, fill=INK)
    now = s['t']
    # The camera.
    d.text((54, top + 62), 'camera', font=F_SMALL, fill=MUTED)
    d.text((54, top + 84), f'#{now}', font=F_BIG, fill=INK)
    d.text((54, top + 116), '60 fps', font=F_SMALL, fill=MUTED)
    # The slots.
    x0 = 210
    slots = BUFFERS if kind == 'queue' else 1
    held = s['queue'] if kind == 'queue' else [s['mailbox']]
    for i in range(slots):
        x = x0 + i * 100
        d.rounded_rectangle([x, top + 64, x + 86, top + 128], 10, outline=LINE, width=2)
        if i < len(held):
            frame_box(d, x, top + 64, held[i], now)
    label = (f'{BUFFERS} kernel buffers, oldest first' if kind == 'queue'
             else 'one slot: the newest replaces it')
    d.text((x0, top + 140), label, font=F_SMALL, fill=MUTED)
    if kind == 'queue' and s['dropped']:
        d.text((x0, top + 160), f'dropped by the driver: {s["dropped"]} frames, never captured',
               font=F_SMALL, fill=RED)
    # The consumer.
    cx = 760
    d.text((cx, top + 62), 'your code', font=F_SMALL, fill=MUTED)
    if s['stalled']:
        k = (now - STALL[0]) / (STALL[1] - STALL[0])
        d.text((cx, top + 84), 'busy (stalled 400 ms)', font=F_LABEL, fill=RED)
        d.rounded_rectangle([cx, top + 116, cx + 380, top + 126], 5, fill=LINE)
        d.rounded_rectangle([cx, top + 116, cx + int(380 * k), top + 126], 5, fill=RED)
    else:
        got = s['read_q'] if kind == 'queue' else s['read_m']
        if got is not None:
            age = (now - got) * PERIOD_MS
            colour = RED if age > 100 else GREEN
            d.text((cx, top + 84), f'reads #{got}', font=F_LABEL, fill=INK)
            d.text((cx + 150, top + 84), f'{age:.0f} ms old', font=F_LABEL, fill=colour)
    if now >= STALL[1]:
        msg = ('measured: 396.1 ms old  (draining the queue: still 348.1 ms)' if kind == 'queue'
               else 'measured: 16.9 ms old, never blocked')
        d.text((cx, top + 150), msg, font=F_SMALL, fill=RED if kind == 'queue' else GREEN)


def draw(s):
    img = Image.new('RGB', (W, H), BG)
    d = ImageDraw.Draw(img)
    d.text((30, 22), 'THE QUEUE IS A FOSSIL', font=F_TITLE, fill=INK)
    d.text((372, 30), 'what a camera hands you after your code was busy for 400 ms',
           font=F_SMALL, fill=MUTED)
    lane(d, 70, 'A. a plain read() from the V4L2 queue', s, 'queue')
    lane(d, 276, 'B. a keep-up thread with a one-frame mailbox', s, 'mailbox')
    # The timeline.
    ty, x0, x1 = 500, 54, W - 54
    d.line([x0, ty, x1, ty], fill=LINE, width=4)
    sx = lambda t: x0 + (x1 - x0) * t / TICKS                   # noqa: E731
    d.line([sx(STALL[0]), ty, sx(STALL[1]), ty], fill=RED, width=4)
    d.text((sx(STALL[0]), ty + 12), 'stall', font=F_SMALL, fill=RED)
    d.ellipse([sx(s['t']) - 7, ty - 7, sx(s['t']) + 7, ty + 7], fill=INK)
    d.text((x1, ty + 12), f't = {s["t"] * PERIOD_MS:.0f} ms', font=F_SMALL, fill=MUTED,
           anchor='ra')
    return img


def main():
    states = simulate()
    frames = [draw(s) for s in states]
    durations = [140] * len(frames)
    durations[STALL[1]] = 1600          # dwell on the moment the code comes back
    durations[-1] = 3500
    OUT.parent.mkdir(parents=True, exist_ok=True)
    pal = [f.quantize(colors=64, method=Image.Quantize.MEDIANCUT) for f in frames]
    pal[0].save(OUT, save_all=True, append_images=pal[1:], duration=durations, loop=0,
                optimize=True)
    print(f'{OUT.relative_to(ROOT)}  {len(frames)} frames  {OUT.stat().st_size / 1024:.0f} KB')


if __name__ == '__main__':
    main()
