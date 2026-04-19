"""Camera implementations. The factory in `duburi_vision.factory` is the
single dispatch point — node code should never import these directly."""

from .camera import Camera, FrameMeta

__all__ = ['Camera', 'FrameMeta']
