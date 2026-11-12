"""`sensor_msgs/Image` to a NumPy array and back, with no `cv_bridge`.

`cv_bridge` did two things for us: view a message's buffer as an array, and
fill a message from an array. It also tied the whole vision stack to the NumPy
its C extension was compiled against (measured-bars 77: beside a NumPy 2
OpenCV it imports and then fails at the first conversion). The stack uses
three encodings and two directions, which is the whole of this module.

Measured on Kutum against `cv_bridge` 4.1.0, 640x360 `bgr8` (measured-bars
78): identical bytes and pixels in every encoding below.

    to_array(msg, 'bgr8')        the image as an array; a view when the
                                 message is already in that encoding
    to_msg(array, 'bgr8')        a message holding one copy of the array

An encoding this module does not know is refused, never guessed: a wrong
channel order is a detector looking at a blue gate.
"""
from __future__ import annotations

import array
import sys

import numpy as np
from sensor_msgs.msg import Image

#: encoding -> (dtype, channels)
ENCODINGS = {
    'bgr8': (np.uint8, 3),
    'rgb8': (np.uint8, 3),
    'mono8': (np.uint8, 1),
    '32FC1': (np.float32, 1),
}


# Jazzy's generated setter for `data` assigns an `array.array` as it is only
# when field checks are on; with them off (the default) it copies the whole
# frame into a new array, 0.13 ms for 640x360 on Kutum. Humble has no such
# switch and always assigns directly.
_CHECKED = {'check_fields': True} if '_check_fields' in Image.__slots__ else {}


def _spec(encoding: str):
    try:
        return ENCODINGS[encoding]
    except KeyError:
        raise ValueError(f'image encoding {encoding!r} is not one this stack '
                         f'uses ({", ".join(sorted(ENCODINGS))})') from None


def to_array(msg: Image, want: str | None = None) -> np.ndarray:
    """The image in `msg` as an array, in encoding `want` (default: its own).

    A view on the message's buffer when no conversion is needed, so keep the
    message alive as long as the array, exactly as with `cv_bridge`.
    """
    dtype, channels = _spec(msg.encoding)
    dt = np.dtype(dtype)
    if dt.itemsize > 1:
        dt = dt.newbyteorder('>' if msg.is_bigendian else '<')
    row = msg.width * channels * dt.itemsize
    step = msg.step or row
    if step < row or len(msg.data) < step * msg.height:
        raise ValueError(f'image {msg.width}x{msg.height} {msg.encoding} needs '
                         f'{row} bytes a row and {row * msg.height} in all; '
                         f'the message has step {step} and {len(msg.data)}')
    flat = np.frombuffer(msg.data, dtype=np.uint8, count=step * msg.height)
    rows = flat.reshape(msg.height, step)[:, :row]      # drop any row padding
    if not rows.flags['C_CONTIGUOUS']:
        rows = np.ascontiguousarray(rows)
    img = rows.view(dt)
    if dt.byteorder not in ('=', '|') and dt.byteorder != (
            '<' if sys.byteorder == 'little' else '>'):
        img = img.astype(dt.newbyteorder('='))
    img = img.reshape(msg.height, msg.width, channels) if channels > 1 \
        else img.reshape(msg.height, msg.width)
    return img if want in (None, msg.encoding) else _convert(img, msg.encoding, want)


def _convert(img: np.ndarray, have: str, want: str) -> np.ndarray:
    import cv2
    _spec(want)
    code = {('rgb8', 'bgr8'): cv2.COLOR_RGB2BGR, ('bgr8', 'rgb8'): cv2.COLOR_BGR2RGB,
            ('mono8', 'bgr8'): cv2.COLOR_GRAY2BGR, ('mono8', 'rgb8'): cv2.COLOR_GRAY2RGB,
            ('bgr8', 'mono8'): cv2.COLOR_BGR2GRAY, ('rgb8', 'mono8'): cv2.COLOR_RGB2GRAY,
            }.get((have, want))
    if code is None:
        raise ValueError(f'no conversion from {have!r} to {want!r}')
    return cv2.cvtColor(img, code)


def to_msg(img: np.ndarray, encoding: str, header=None) -> Image:
    """A message holding `img`, which is already in `encoding`."""
    dtype, channels = _spec(encoding)
    shape_ok = (img.ndim == 3 and img.shape[2] == channels) or (
        img.ndim == 2 and channels == 1)
    if not shape_ok or img.dtype != np.dtype(dtype):
        raise ValueError(f'an array of shape {img.shape} and dtype {img.dtype} '
                         f'is not a {encoding} image')
    msg = Image(**_CHECKED)
    if header is not None:
        msg.header = header
    msg.height, msg.width = int(img.shape[0]), int(img.shape[1])
    msg.encoding = encoding
    msg.is_bigendian = int(sys.byteorder == 'big')
    msg.step = msg.width * channels * np.dtype(dtype).itemsize
    data = array.array('B')
    # One copy. `frombytes` wants a flat byte buffer, so view the pixels as one.
    data.frombytes(np.ascontiguousarray(img).reshape(-1).view(np.uint8))
    msg.data = data
    return msg
