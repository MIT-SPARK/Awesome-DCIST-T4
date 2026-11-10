"""Server-side image variants for ``/api/graph/{src}/image/{sym}/{i}``:
``w=`` downscale, ``mask=1`` segmentation-mask overlay, ``crop=1`` crop to the
object (mask / 2D box). Rendered with Pillow + numpy and kept in a small LRU.
"""

from __future__ import annotations

import io
import logging
import threading
from collections import OrderedDict

import numpy as np

log = logging.getLogger(__name__)

MIN_W, MAX_W = 16, 2048
MASK_RGB = np.array([255, 120, 0], dtype=np.float32)  # orange fill + outline
MASK_ALPHA = 0.45
CACHE_BYTES = 64 << 20


def available() -> bool:
    try:
        import PIL.Image  # noqa: F401
    except Exception:  # noqa: BLE001
        return False
    return True


def _erode(m: np.ndarray) -> np.ndarray:
    e = m.copy()
    e[1:, :] &= m[:-1, :]
    e[:-1, :] &= m[1:, :]
    e[:, 1:] &= m[:, :-1]
    e[:, :-1] &= m[:, 1:]
    return e


def outline(m: np.ndarray, width: int = 1) -> np.ndarray:
    """Pixels of ``m`` within ``width`` px of its boundary."""
    inner = m
    for _ in range(max(1, width)):
        inner = _erode(inner)
    return m & ~inner


def overlay_mask(rgb: np.ndarray, m: np.ndarray) -> np.ndarray:
    """``rgb`` (HxWx3 uint8) with ``m`` tinted and outlined."""
    out = rgb.astype(np.float32)
    out[m] = out[m] * (1 - MASK_ALPHA) + MASK_RGB * MASK_ALPHA
    edge = outline(m, max(1, round(min(m.shape) / 160)))
    out[edge] = MASK_RGB
    return out.clip(0, 255).astype(np.uint8)


def crop_box(shape, m: np.ndarray | None, bbox2d) -> tuple[int, int, int, int] | None:
    """(x0, y0, x1, y1) around the object with context, roughly square; None = whole."""
    h, w = shape[:2]
    if m is not None and m.any():
        ys, xs = np.nonzero(m)
        x0, y0, x1, y1 = xs.min(), ys.min(), xs.max() + 1, ys.max() + 1
    elif bbox2d is not None:
        x0, y0, x1, y1 = (int(v) for v in bbox2d)
    else:
        return None
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    side = max(max(x1 - x0, y1 - y0) * 1.6 + 24, min(h, w) * 0.25)
    sx, sy = min(w, side), min(h, side)
    bx0 = int(max(0, min(w - sx, cx - sx / 2)))
    by0 = int(max(0, min(h - sy, cy - sy / 2)))
    bx1 = int(min(w, bx0 + sx))
    by1 = int(min(h, by0 + sy))
    if bx1 - bx0 < 2 or by1 - by0 < 2:
        return None
    return bx0, by0, bx1, by1


def render(img, w: int | None = None, mask: bool = False, crop: bool = False) -> bytes:
    """JPEG bytes of a :class:`graph_service.NodeImage` variant."""
    from PIL import Image

    with Image.open(img.path) as im:
        rgb = np.asarray(im.convert("RGB"))
    m = None
    if mask or crop:
        m = img.mask(shape=rgb.shape[:2])
        if m is not None:
            m = np.asarray(m, dtype=bool)
            if m.shape != rgb.shape[:2]:  # mask at another resolution than the frame
                m = (
                    np.asarray(
                        Image.fromarray(m.astype(np.uint8) * 255).resize(
                            (rgb.shape[1], rgb.shape[0]), Image.NEAREST
                        )
                    )
                    > 127
                )
    if mask and m is not None:
        rgb = overlay_mask(rgb, m)
    if crop:
        box = crop_box(rgb.shape, m, getattr(img, "bbox2d", None))
        if box is not None:
            x0, y0, x1, y1 = box
            rgb = rgb[y0:y1, x0:x1]
    out = Image.fromarray(np.ascontiguousarray(rgb))
    if w:
        w = max(MIN_W, min(MAX_W, int(w)))
        if w < out.width:
            out = out.resize(
                (w, max(1, round(out.height * w / out.width))), Image.LANCZOS
            )
    buf = io.BytesIO()
    out.save(buf, "JPEG", quality=85)
    return buf.getvalue()


class RenderCache:
    """LRU of rendered variants, bounded by total bytes."""

    def __init__(self, max_bytes: int = CACHE_BYTES):
        self.max_bytes = max_bytes
        self._d: OrderedDict = OrderedDict()
        self._bytes = 0
        self._lock = threading.Lock()

    def get(self, key):
        with self._lock:
            v = self._d.get(key)
            if v is not None:
                self._d.move_to_end(key)
            return v

    def put(self, key, data: bytes) -> None:
        with self._lock:
            old = self._d.pop(key, None)
            if old is not None:
                self._bytes -= len(old)
            self._d[key] = data
            self._bytes += len(data)
            while self._bytes > self.max_bytes and len(self._d) > 1:
                _, v = self._d.popitem(last=False)
                self._bytes -= len(v)

    def __len__(self):
        return len(self._d)


def variant_key(src: str, img, w, mask: bool, crop: bool):
    try:
        mtime = img.path.stat().st_mtime_ns
    except OSError:
        mtime = None
    obs = getattr(img, "obs", None)
    if obs is not None:
        mask_id = getattr(obs, "obs_id", None)
    else:
        mask_id = (str(img.mask_path), hash(img.mask_rle))
    return (src, str(img.path), mtime, w, bool(mask), bool(crop), mask_id)
