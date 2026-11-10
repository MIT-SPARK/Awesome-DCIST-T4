#!/usr/bin/env python3
"""Render the console's home-screen icons (web app manifest + iOS) into public/.

The logo is the favicon's green triangle (index.html) on the theme background.
Run from the frontend directory: python3 scripts/make-icons.py
"""

import pathlib

from PIL import Image, ImageDraw

BG = (18, 22, 28, 255)  # #12161c, the favicon's tile
FG = (53, 208, 127, 255)  # --ok
TRI = [(3, 12), (8, 3), (13, 12)]  # favicon path, 16-unit box
OUT = pathlib.Path(__file__).resolve().parent.parent / "public"
SS = 4  # supersampling for smooth edges


def icon(size: int, logo_frac: float, radius_frac: float) -> Image.Image:
    s = size * SS
    im = Image.new("RGBA", (s, s), (0, 0, 0, 0))
    d = ImageDraw.Draw(im)
    d.rounded_rectangle([0, 0, s - 1, s - 1], radius=int(s * radius_frac), fill=BG)
    # scale the 10x9 triangle box (x 3..13, y 3..12) to logo_frac of the icon
    box = logo_frac * s
    k = box / 10
    ox = (s - 10 * k) / 2 - 3 * k
    oy = (s - 9 * k) / 2 - 3 * k
    d.polygon([(ox + x * k, oy + y * k) for x, y in TRI], fill=FG)
    return im.resize((size, size), Image.LANCZOS)


OUT.mkdir(exist_ok=True)
icon(192, 0.62, 0.19).save(OUT / "icon-192.png")
icon(512, 0.62, 0.19).save(OUT / "icon-512.png")
icon(512, 0.5, 0).save(
    OUT / "icon-maskable-512.png"
)  # full bleed, logo inside the 80% safe zone
icon(180, 0.56, 0).convert("RGB").save(
    OUT / "apple-touch-icon.png"
)  # iOS rounds the corners itself
print("wrote", sorted(p.name for p in OUT.glob("*.png")))
