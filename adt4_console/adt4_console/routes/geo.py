"""``/api/geo`` — map anchors and the satellite tile cache (API.md §6)."""

from __future__ import annotations

from fastapi import APIRouter, HTTPException, Request, Response

from . import services

router = APIRouter(prefix="/api/geo")


@router.get("/layers")
def layers(request: Request):
    return {"layers": services(request).geo.tiles.layers()}


@router.get("/anchor")
def default_anchor(request: Request):
    a = services(request).geo.default_anchor()
    return {"anchor": a.to_dict() if a is not None else None}


@router.get("/anchor/{src:path}")
def anchor(src: str, request: Request, cx: float = 0.0, cy: float = 0.0):
    """The source's map→UTM anchor (or ``null``: no GPS alignment), with a local
    affine map↔lon/lat around ``(cx, cy)`` for the browser."""
    a = services(request).geo.anchor(src)
    if a is None:
        return {"src": src, "anchor": None}
    try:
        affine = a.local_affine(cx, cy)
    except Exception as e:  # noqa: BLE001 (pyproj missing)
        raise HTTPException(503, f"geo conversion unavailable: {e}") from None
    return {"src": src, "anchor": a.to_dict(), "affine": affine}


@router.get("/tiles/{layer}/{z}/{x}/{y}")
def tile(layer: str, z: int, x: int, y: int, request: Request):
    try:
        data = services(request).geo.tiles.get(layer, z, x, y)
    except ValueError as e:
        raise HTTPException(400, str(e)) from None
    if data is None:
        raise HTTPException(404, "tile not cached (and live fetching is off)")
    media = "image/png" if data[:4] == b"\x89PNG" else "image/jpeg"
    return Response(
        data, media_type=media, headers={"Cache-Control": "public, max-age=86400"}
    )
