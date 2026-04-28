#!/usr/bin/env python3
"""Render a poster figure: alpha and bravo GPS map trajectories side-by-side.

Each panel shows the GPS trajectory used to build that map on top of an Esri
WorldImagery basemap, annotated with total path length and a metric scale
bar. Bounds are tight to the trajectory plus a small padding so the panels
read as compact map tiles.

The trajectories are taken from `gps_trajectory_max_sigma_0.01.csv` for both
maps — per Mason's note, the strictest GPS-covariance rejection produces the
smoothest, most accurate trajectory (the looser thresholds keep noisy fixes
that drift especially around the bravo portapotty area).

Run from `$ADT_ENV/spark_env` (provides contextily, pyproj, scipy,
matplotlib):

    $ADT_ENV/spark_env/bin/python3 scripts/experiment_postprocess/plot_poster_maps_overview.py
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import numpy as np


# ---- Inputs -------------------------------------------------------------

ALPHA_CSV = Path(
    "/data/dcist/west_point_2026/clean_graphs/alpha_trajectories/"
    "csv_downloads_2/gps_trajectory_max_sigma_0.01.csv"
)
BRAVO_CSV = Path(
    "/data/dcist/west_point_2026/clean_graphs/bravo_trajectories/"
    "csv_downloads/gps_trajectory_max_sigma_0.01.csv"
)
OUT_DIR_DEFAULT = Path("/home/harel/data/west_point_2026/poster")

UTM_EPSG     = "EPSG:32618"   # West Point 18N
WEB_MERCATOR = "EPSG:3857"


# ---- Visual style -------------------------------------------------------

# MIT/CSAIL palette (https://www.csail.mit.edu/sites/default/files/inline-images/Color.png).
# Burnt orange (PANTONE 1525 C) + bright blue (PANTONE 2925) is the highest-
# contrast pair in the palette and reads cleanly over green/dark satellite
# tiles.
ALPHA_COLOR = "#B94700"   # MIT burnt orange
BRAVO_COLOR = "#009CDE"   # MIT bright blue
TRAIL_LW = 5.0

EXTENT_PAD_M = 18.0
PANEL_DPI = 300
SQUARE_EXTENT = True       # pad the smaller dimension so panels are square
BASEMAP_ZOOM = 19


# ---- Helpers ------------------------------------------------------------


def load_utm_trajectory(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Read a `#timestamp_kf,x,y,...` UTM CSV, return (x, y)."""
    xs, ys = [], []
    with open(path, newline="") as f:
        rdr = csv.DictReader(f)
        for row in rdr:
            xs.append(float(row["x"]))
            ys.append(float(row["y"]))
    return np.asarray(xs, dtype=np.float64), np.asarray(ys, dtype=np.float64)


def path_length(x: np.ndarray, y: np.ndarray) -> float:
    if len(x) < 2:
        return 0.0
    return float(np.sum(np.hypot(np.diff(x), np.diff(y))))


def compute_extent(x: np.ndarray, y: np.ndarray
                   ) -> tuple[float, float, float, float]:
    xmin, xmax = float(x.min()) - EXTENT_PAD_M, float(x.max()) + EXTENT_PAD_M
    ymin, ymax = float(y.min()) - EXTENT_PAD_M, float(y.max()) + EXTENT_PAD_M
    if SQUARE_EXTENT:
        w, h = xmax - xmin, ymax - ymin
        if h > w:
            extra = (h - w) / 2.0
            xmin -= extra; xmax += extra
        elif w > h:
            extra = (w - h) / 2.0
            ymin -= extra; ymax += extra
    return xmin, xmax, ymin, ymax


def build_projector():
    from pyproj import Transformer
    tr = Transformer.from_crs(UTM_EPSG, WEB_MERCATOR, always_xy=True)
    def project(xs, ys):
        x, y = tr.transform(np.asarray(xs), np.asarray(ys))
        return np.asarray(x), np.asarray(y)
    return project


def _pick_scale_length_m(extent_m: float) -> int:
    """Pick a tidy scale-bar length (m) ≈ 1/5 of the panel's metric x-extent."""
    target = extent_m / 5.0
    candidates = [10, 20, 25, 50, 100, 150, 200, 250, 500]
    return min(candidates, key=lambda v: abs(v - target))


def _draw_scale_bar(ax, project, extent_utm: tuple[float, float, float, float],
                    *, length_m: int) -> None:
    """Draw a metric scale bar in the lower-right of the panel.

    The bar is anchored in UTM metres and re-projected, so its length is
    accurate in the displayed (Web Mercator) units even though Mercator
    stretches slightly at this latitude.
    """
    xmin_u, xmax_u, ymin_u, ymax_u = extent_utm
    width_u = xmax_u - xmin_u
    height_u = ymax_u - ymin_u
    # Right edge inset 6% from xmax; bottom edge inset 6% from ymin.
    right_u = xmax_u - 0.06 * width_u
    left_u  = right_u - length_m
    y_u     = ymin_u + 0.06 * height_u

    xs_p, ys_p = project(np.array([left_u, right_u]),
                          np.array([y_u,    y_u]))
    # Outline pass (dark, slightly thicker) for legibility against bright tiles.
    ax.plot(xs_p, ys_p, "-", color="black", linewidth=8.0,
            solid_capstyle="butt", zorder=11)
    ax.plot(xs_p, ys_p, "-", color="white", linewidth=5.0,
            solid_capstyle="butt", zorder=12)
    # End ticks.
    tick_h_u = 0.012 * height_u
    for x_u in (left_u, right_u):
        tx_p, ty_p = project(np.array([x_u, x_u]),
                              np.array([y_u - tick_h_u, y_u + tick_h_u]))
        ax.plot(tx_p, ty_p, "-", color="black", linewidth=8.0,
                solid_capstyle="butt", zorder=11)
        ax.plot(tx_p, ty_p, "-", color="white", linewidth=5.0,
                solid_capstyle="butt", zorder=12)
    # Label above the bar.
    mid_u = (left_u + right_u) / 2.0
    lbl_p = project(np.array([mid_u]),
                    np.array([y_u + 0.025 * height_u]))
    ax.annotate(
        f"{length_m} m",
        xy=(float(lbl_p[0][0]), float(lbl_p[1][0])),
        xycoords="data",
        ha="center", va="bottom",
        fontsize=22, fontweight="bold", color="white",
        path_effects=None,
        bbox=dict(boxstyle="round,pad=0.25", fc="black", ec="white",
                  alpha=0.75),
        zorder=13,
    )


def render_panel(ax, x_utm: np.ndarray, y_utm: np.ndarray, *,
                 title: str, color: str) -> float:
    """Render a single map panel. Returns path_length_m."""
    import contextily as cx

    project = build_projector()
    extent_utm = compute_extent(x_utm, y_utm)
    xmin, xmax, ymin, ymax = extent_utm
    cxs, cys = project(
        np.array([xmin, xmax, xmax, xmin]),
        np.array([ymin, ymin, ymax, ymax]),
    )
    ax.set_xlim(cxs.min(), cxs.max())
    ax.set_ylim(cys.min(), cys.max())
    ax.set_aspect("equal", adjustable="box")
    ax.set_xticks([])
    ax.set_yticks([])
    for spine in ax.spines.values():
        spine.set_visible(False)

    try:
        cx.add_basemap(ax, source=cx.providers.Esri.WorldImagery,
                       crs=WEB_MERCATOR, alpha=0.95, zoom=BASEMAP_ZOOM,
                       attribution=False)
    except Exception as e:
        print(f"  [basemap] fetch failed: {e}", file=sys.stderr)

    x_p, y_p = project(x_utm, y_utm)
    ax.plot(x_p, y_p, "-", color=color, linewidth=TRAIL_LW, alpha=0.97,
            solid_capstyle="round", zorder=4)

    plen = path_length(x_utm, y_utm)
    # Title overlaid inside the panel (top-left) to save vertical space.
    ax.text(
        0.02, 0.98, title, transform=ax.transAxes,
        ha="left", va="top",
        fontsize=30, fontweight="bold", color="#111",
        bbox=dict(boxstyle="round,pad=0.45", fc="white", ec="#888",
                  alpha=0.92),
        zorder=10,
    )
    ax.text(
        0.02, 0.02, f"Path length: {plen:,.0f} m",
        transform=ax.transAxes,
        ha="left", va="bottom",
        fontsize=20, fontweight="bold", color="#111",
        bbox=dict(boxstyle="round,pad=0.45", fc="white", ec="#888",
                  alpha=0.92),
        zorder=10,
    )

    scale_m = _pick_scale_length_m(xmax - xmin)
    _draw_scale_bar(ax, project, extent_utm, length_m=scale_m)
    return plen


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--output-dir", type=Path, default=OUT_DIR_DEFAULT)
    ap.add_argument("--alpha-csv", type=Path, default=ALPHA_CSV)
    ap.add_argument("--bravo-csv", type=Path, default=BRAVO_CSV)
    ap.add_argument("--filename", default="maps_overview.png")
    args = ap.parse_args()

    import matplotlib.pyplot as plt

    print(f"loading alpha {args.alpha_csv}")
    ax_, ay_ = load_utm_trajectory(args.alpha_csv)
    print(f"  {len(ax_)} samples")

    print(f"loading bravo {args.bravo_csv}")
    bx_, by_ = load_utm_trajectory(args.bravo_csv)
    print(f"  {len(bx_)} samples")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    # Square panels (titles are inside the axes now, so no extra top space).
    fig, axes = plt.subplots(1, 2, figsize=(20, 10), dpi=PANEL_DPI)
    fig.patch.set_alpha(0.0)

    # Layout: Bravo on the left as "Area 1", Alpha on the right as "Area 2".
    plen_b = render_panel(axes[0], bx_, by_,
                          title="Area 1", color=BRAVO_COLOR)
    plen_a = render_panel(axes[1], ax_, ay_,
                          title="Area 2", color=ALPHA_COLOR)

    print(f"Area 1 (bravo): path {plen_b:.1f} m")
    print(f"Area 2 (alpha): path {plen_a:.1f} m")

    fig.subplots_adjust(left=0.01, right=0.99, top=0.99, bottom=0.01,
                        wspace=0.03)
    out = args.output_dir / args.filename
    fig.savefig(out, dpi=PANEL_DPI, bbox_inches="tight", pad_inches=0.08,
                facecolor="none", edgecolor="none")
    plt.close(fig)
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
