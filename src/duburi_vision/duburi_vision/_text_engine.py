"""TrueType text rendering engine for HUD widgets.

Extracted from draw_widgets.py so the PIL/Pillow font-loading and caching
logic can be maintained separately from widget geometry.

Public API
----------
pil_text(img, text, org, cv2_fs, color_bgr) -> None
    Render TrueType text onto a BGR numpy array. org is (x, y_bottom).
    Cached by (text, size, color) -- subsequent frames with identical
    strings are pure array blits with no PIL allocation.

pil_text_size(text, cv2_fs) -> (width, height)
    Return pixel dimensions of text at the given cv2-style scale.
"""

from __future__ import annotations

import os as _os

import numpy as np
from PIL import Image as _PILImage, ImageDraw as _PILDraw, ImageFont as _PILFont

_PTF: dict = {}   # int -> FreeTypeFont (or default ImageFont)
_PTF_PATH: str | None = None
_TEXT_CACHE: dict = {}        # (text, px, r, g, b) -> (surf_rgba: np.ndarray, th: int)
_TEXT_CACHE_MAX = 512         # evict-all when full (simple; HUD strings are bounded)

_MONO_CANDIDATES = (
    '/usr/local/share/fonts/TTF/IosevkaNerdFontMono-Regular.ttf',
    '/usr/share/fonts/truetype/noto/NotoSansMono-Regular.ttf',
    '/usr/local/share/fonts/noto/NotoSansMono-Regular.ttf',
    '/usr/local/share/fonts/TTF/DejaVuSansMono.ttf',
    '/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf',
)

_CV2FS_TO_PX = 34  # cv2 SIMPLEX scale=1.0 ~= 34px cap height


def _pil_font(px: int) -> _PILFont.FreeTypeFont:
    global _PTF_PATH
    if _PTF_PATH is None:
        for c in _MONO_CANDIDATES:
            if _os.path.exists(c):
                _PTF_PATH = c
                break
        else:
            _PTF_PATH = ''
    if px not in _PTF:
        try:
            _PTF[px] = (_PILFont.truetype(_PTF_PATH, px) if _PTF_PATH
                        else _PILFont.load_default())
        except Exception:
            _PTF[px] = _PILFont.load_default()
    return _PTF[px]


def _blit_rgba(img: np.ndarray, surf: np.ndarray, x: int, y: int) -> None:
    """Alpha-composite RGBA surface patch onto BGR image at (x, y) top-left."""
    ih, iw = img.shape[:2]
    sh, sw = surf.shape[:2]
    sx1 = max(0, -x);   dx1 = max(0, x)
    sy1 = max(0, -y);   dy1 = max(0, y)
    sx2 = min(sw, iw - x);  dx2 = min(iw, x + sw)
    sy2 = min(sh, ih - y);  dy2 = min(ih, y + sh)
    if sx2 <= sx1 or sy2 <= sy1 or dx2 <= dx1 or dy2 <= dy1:
        return
    patch = surf[sy1:sy2, sx1:sx2]
    # uint16 blend: (a*fg + (255-a)*bg) >> 8 -- avoids float32, imperceptible vs /255
    alpha = patch[:, :, 3:4].astype(np.uint16)
    inv   = np.uint16(255) - alpha
    fg    = patch[:, :, :3][:, :, ::-1].astype(np.uint16)   # RGBA -> BGR
    bg    = img[dy1:dy2, dx1:dx2].astype(np.uint16)
    img[dy1:dy2, dx1:dx2] = ((alpha * fg + inv * bg) >> 8).astype(np.uint8)


def _render_cached(text: str, px: int,
                   r: int, g: int, b: int) -> tuple:
    """Return cached (RGBA_array, text_height). Renders on first call per unique key."""
    key = (text, px, r, g, b)
    cached = _TEXT_CACHE.get(key)
    if cached is not None:
        return cached
    font = _pil_font(px)
    try:
        bbox = font.getbbox(text)
    except Exception:
        result: tuple = (np.zeros((4, 4, 4), dtype=np.uint8), 0)
        _TEXT_CACHE[key] = result
        return result
    if bbox[2] <= bbox[0]:
        result = (np.zeros((4, 4, 4), dtype=np.uint8), 0)
        _TEXT_CACHE[key] = result
        return result
    tw = int(bbox[2] - bbox[0])
    th = int(bbox[3] - bbox[1])
    surf = _PILImage.new('RGBA', (tw + 4, th + 4), (0, 0, 0, 0))
    _PILDraw.Draw(surf).text((2 - bbox[0], 2 - bbox[1]),
                              text, font=font, fill=(r, g, b, 255))
    arr = np.array(surf)
    if len(_TEXT_CACHE) >= _TEXT_CACHE_MAX:
        _TEXT_CACHE.clear()
    result = (arr, th)
    _TEXT_CACHE[key] = result
    return result


def pil_text(img: np.ndarray, text: str, org: tuple,
             cv2_fs: float, color_bgr: tuple) -> None:
    """Render TrueType text onto BGR array.

    org is (x, y_bottom) -- same bottom-left baseline convention as cv2.putText.
    Rendered surfaces are cached by (text, size, color) -- subsequent frames with
    identical strings are pure array blits with no PIL allocation.
    """
    if not text:
        return
    px = max(8, round(cv2_fs * _CV2FS_TO_PX))
    r, g, b = int(color_bgr[2]), int(color_bgr[1]), int(color_bgr[0])
    surf_arr, th = _render_cached(text, px, r, g, b)
    x, y_base = int(org[0]), int(org[1])
    _blit_rgba(img, surf_arr, x, int(y_base - th - 2))


def pil_text_size(text: str, cv2_fs: float) -> tuple[int, int]:
    """Return (width, height) of text rendered at given cv2-style scale."""
    if not text:
        return (0, 0)
    px = max(8, round(cv2_fs * _CV2FS_TO_PX))
    font = _pil_font(px)
    try:
        bbox = font.getbbox(text)
        return (max(0, int(bbox[2] - bbox[0])), max(0, int(bbox[3] - bbox[1])))
    except Exception:
        return (0, 0)
