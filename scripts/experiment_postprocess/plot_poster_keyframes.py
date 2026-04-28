#!/usr/bin/env python3
"""Render poster keyframes for april_17_alpha__exp_0003 cone-blocking mission.

Produces three keyframes (start / pickup / mission-complete) for the
multi-robot cone-blocking experiment, each rendered twice:

  * `_clean`   variant: scene-graph-only (white background, UTM frame).
  * `_basemap` variant: Esri WorldImagery satellite tiles, scene graph on top.

Plus 1x3 composite tiles for each variant.

Run with the ROS 2 Jazzy workspace + spark_env sourced (needs spark_dsg,
robotdatapy, contextily, pyproj, PIL, matplotlib, imageio_ffmpeg, urllib).
"""

from __future__ import annotations

import argparse
import csv
import datetime
import sys
import urllib.request
from pathlib import Path
from typing import Optional

import numpy as np

_THIS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(_THIS_DIR))
import _scene_graph_utils as sgu  # noqa: E402


# ---- Hardcoded experiment configuration ---------------------------------

EXPERIMENT_ID    = "april_17_alpha__exp_0003"
SCENE_GRAPH_JSON = Path("/data/dcist/west_point_2026/clean_graphs/alpha_thurs_clean_2.json")
ANCHOR_DIR       = Path("/home/harel/data/west_point_2026/processed/scene_graph_anchors")
ANCHOR_KEY       = "alpha_thurs_clean_2"
HAMILTON_CSV     = Path("/home/harel/data/west_point_2026/processed/hydra/april_17_alpha_hamilton__recorded_data.csv")
EUCLID_CSV       = Path("/home/harel/data/west_point_2026/processed/fused/april_17_alpha_euclid__recorded_data.csv")
ASSETS_DIR       = Path("/home/harel/data/west_point_2026/animation_plot")
OUT_DIR_DEFAULT  = Path("/home/harel/data/west_point_2026/poster/april_17_alpha_keyframes")

EXPERIMENT_START_UNIX = 1776441241.617781   # exp_0003 start (15:54:01 UTC)
EXPERIMENT_END_UNIX   = 1776442211.756396   # exp_0003 end   (16:10:11 UTC)

INSTRUCTION_TEXT = (
    "Mr.X's car is in the parking lot by the government building. "
    "Tell hamilton to use a cone to block the intersection nearest the "
    "barracks. Tell euclid to use the other cone to block the other "
    "intersection."
)

UTM_EPSG     = "EPSG:32618"   # West Point 18N
WEB_MERCATOR = "EPSG:3857"

# Curated cone events (copied verbatim from animate_experiment_april_17.py).
CONE_EVENTS: list[dict] = [
    {
        "who": "hamilton",
        "object_id": "O102",   # PDDL goal: (object-in-place O102 t4987)
        "spawn_t": 1776441241.617781,
        "pickup_t": 1776441361.12,
        "drop_t":   1776441600.62,
        "spawn_xy": (579010.35, 4578522.06),
        "drop_xy":  (579080.28, 4578419.70),
    },
    {
        "who": "euclid",
        "object_id": "O103",   # PDDL goal: (object-in-place O103 t3917)
        "spawn_t": 1776441241.617781,
        "pickup_t": 1776441459.12,
        "drop_t":   1776441600.62,
        "spawn_xy": (579016.73, 4578526.18),
        "drop_xy":  (579038.10, 4578462.63),
    },
]

# PDDL-goal place IDs per room index in the loaded scene graph (room[1] is
# Euclid's drop, room[2] is Hamilton's drop after the override in main()).
ROOM_PLACE_IDS: dict[int, str] = {
    1: "t3917",   # Euclid's intersection (Intersection 1)
    2: "t4987",   # Hamilton's intersection (Intersection 2)
}

# Hand-tuned landmark UTM positions (from door/window clusters in the scene graph).
# The named-class landmarks (government_building, barracks, ...) in the graph are
# all placeholders at the anchor origin, so we hand-place these instead.
LANDMARKS: list[dict] = [
    {
        "name": "Mr.X's car",
        "icon_hex": "1F697",   # 🚗 red car
        "xy_utm":   (578998.0, 4578532.0),    # NW corner of parking lot
        "label_offset_m": (-15.0, 0.0),        # further left of icon — clears robots
    },
    {
        "name": "Government building",
        "icon_hex": "1F3DB",   # 🏛️ classical building
        "xy_utm":   (579035.0, 4578534.0),    # door cluster north of cone spawn
        "label_offset_m": (0.0, 8.0),          # above icon (clears parking-lot interior)
    },
    {
        "name": "Barracks",
        "icon_hex": "1F3F0",   # 🏰 castle
        "xy_utm":   (579042.0, 4578378.0),    # door cluster south, near hamilton drop
        "label_offset_m": (0.0, 9.0),          # text above icon (clears legend at bottom)
    },
]

# Three keyframes for the poster. Each keyframe gets two renders (clean + basemap).
def keyframes() -> list[dict]:
    ham_pickup = next(e["pickup_t"] for e in CONE_EVENTS if e["who"] == "hamilton")
    euc_pickup = next(e["pickup_t"] for e in CONE_EVENTS if e["who"] == "euclid")
    drop_t     = max(e["drop_t"]   for e in CONE_EVENTS)
    return [
        {
            "key":   "start",
            "title": "",
            "t":     EXPERIMENT_START_UNIX + 5.0,
            "show_trajectory": False,         # no motion yet
            "show_full_trajectory": True,
        },
        {
            "key":   "pickup",
            "title": "",
            "t":     max(ham_pickup, euc_pickup) + 1.0,
            "show_trajectory": True,
            "show_full_trajectory": False,    # bold trail only, no faded preview
        },
        {
            "key":   "complete",
            "title": "",
            "t":     drop_t + 5.0,
            "show_trajectory": True,
            "show_full_trajectory": True,
        },
    ]


# ---- Visual style -------------------------------------------------------

EUCLID_COLOR   = "#e41a1c"   # saturated red
HAMILTON_COLOR = "#0328fc"   # saturated blue
TRAIL_FADE_ALPHA = 0.30
TRAIL_HOT_ALPHA  = 0.95
TRAIL_FADE_LW    = 1.8
TRAIL_HOT_LW     = 4.0

SPOT_ZOOM    = 0.030       # AnnotationBbox zoom for spot sprites in static figures
CONE_ZOOM    = 0.040
LMARK_ZOOM   = 0.060       # OpenMoji icons; 618x618 base

EXTENT_PAD_M = 14.0        # padding around (trajectories ∪ cone events) extent
PANEL_DPI    = 300

# Each panel renders into a near-square data area so equal-aspect doesn't
# collapse the trajectories. We pad the smaller dimension up to match the
# larger so panels are square in metric units.
SQUARE_EXTENT = True

# Trim this fraction off the bottom of the saved basemap PNGs to remove
# unused satellite imagery and shrink the figure's vertical footprint on
# the poster. Applied to both individual and composite basemap outputs.
BASEMAP_BOTTOM_CROP_FRAC = 0.03

# Mission-relevant object labels to draw as bboxes. Empty set = hide all
# scene-graph object bboxes (the mission narrative is already conveyed by the
# landmark icons + cone/spot sprites). Override here to surface specific
# semantic classes (e.g. {6} for cars to highlight the parking lot context).
MISSION_RELEVANT_LABELS: set[int] = set()

# Region coloring is by semantic label from the ROOMS labelspace (_l4p0):
#   0: unknown, 1: parking_lot, 2: intersection
# Distinct hues so a viewer immediately reads "parking lot" vs "intersection".
REGION_LABEL_NAMES: dict[int, str] = {
    0: "unknown",
    1: "Parking lot",
    2: "Intersection",
}
REGION_LABEL_COLORS: dict[int, str] = {
    0: "#bbbbbb",  # gray for unknown
    1: "#b3cde3",  # sky blue — parking lot
    2: "#fdb462",  # amber — intersection (matches cone color family)
}
REGION_FALLBACK_COLOR = "#cccccc"

# Scene-graph place dots. On the satellite variant they need to fight tile
# colors; bright yellow with a dark outline reads well without clashing
# with the red (Euclid) / blue (Hamilton) trajectory colors. On the clean
# variant we keep them softer since the white background gives plenty of
# contrast.
PLACE_COLOR_BASEMAP = "#ffeb3b"   # vivid yellow
PLACE_COLOR_CLEAN   = "#444444"   # neutral dark gray

# Intersection regions are drawn as a small fixed-size octagon centered on the
# room's stored center, instead of the convex hull of the room's
# nearest-neighbor place cluster (which sprawls way past the actual
# intersection footprint).
INTERSECTION_LABEL = 2
INTERSECTION_POLY_RADIUS_M = 5.0
INTERSECTION_POLY_SIDES = 8   # octagon — reads like a stop-sign / "blocked" cue


# ---- CSV trajectory loader ----------------------------------------------


def load_utm_trajectory(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Read `#timestamp_kf,x,y,...` UTM CSV, return (t_unix, x, y)."""
    t_ns, xs, ys = [], [], []
    with open(path, newline="") as f:
        rdr = csv.DictReader(f)
        ts_key = next(k for k in rdr.fieldnames if "timestamp" in k)
        for row in rdr:
            t_ns.append(int(row[ts_key]))
            xs.append(float(row["x"]))
            ys.append(float(row["y"]))
    return (
        np.asarray(t_ns, dtype=np.int64) * 1e-9,
        np.asarray(xs, dtype=np.float64),
        np.asarray(ys, dtype=np.float64),
    )


def interp_xy(t_arr: np.ndarray, x: np.ndarray, y: np.ndarray, t_query: float
              ) -> Optional[tuple[float, float]]:
    if len(t_arr) == 0 or t_query < t_arr[0] or t_query > t_arr[-1]:
        return None
    return float(np.interp(t_query, t_arr, x)), float(np.interp(t_query, t_arr, y))


def total_path_length(traj: tuple[np.ndarray, np.ndarray, np.ndarray],
                      t_lo: float, t_hi: float) -> float:
    """Sum Euclidean step distances of `traj` samples in [t_lo, t_hi]."""
    t_arr, x, y = traj
    mask = (t_arr >= t_lo) & (t_arr <= t_hi)
    xs, ys = x[mask], y[mask]
    if len(xs) < 2:
        return 0.0
    return float(np.sum(np.hypot(np.diff(xs), np.diff(ys))))


# ---- Asset / icon loading -----------------------------------------------


OPENMOJI_URL = "https://raw.githubusercontent.com/hfg-gmuend/openmoji/master/color/618x618/{hex}.png"


def fetch_openmoji_png(hex_code: str, cache_dir: Path) -> Optional[Path]:
    """Download (cached) an OpenMoji color PNG for the given hex codepoint."""
    cache_dir.mkdir(parents=True, exist_ok=True)
    out = cache_dir / f"{hex_code}.png"
    if out.exists() and out.stat().st_size > 0:
        return out
    url = OPENMOJI_URL.format(hex=hex_code)
    try:
        urllib.request.urlretrieve(url, out)
    except Exception as e:
        print(f"  [icons] failed to fetch {hex_code} from {url}: {e}", file=sys.stderr)
        return None
    return out


def load_sprite_cache(assets_dir: Path) -> dict:
    """Load the spot/cone PNGs and pre-flip horizontal variants. Returns a dict.

    Keys: 'spot_r', 'spot_l', 'spot_cone_r', 'spot_cone_l', 'cone'.
    """
    import matplotlib.image as mpimg
    spot   = mpimg.imread(str(assets_dir / "spot_halo.png"))
    spotc  = mpimg.imread(str(assets_dir / "spot_with_cone.png"))
    cone   = mpimg.imread(str(assets_dir / "cone.png"))
    return {
        "spot_r":      spot,
        "spot_l":      spot[:, ::-1, :],
        "spot_cone_r": spotc,
        "spot_cone_l": spotc[:, ::-1, :],
        "cone":        cone,
    }


def load_rooms(graph_path: Path) -> list[dict]:
    """Read the ROOMS layer (layer 4) from the spark_dsg JSON directly.

    The Python binding for ROOMS attribute positions returns garbage X values
    for these clean graphs (uninitialized memory: ~-6.7e+112). Going through
    the raw JSON gives clean (-137, -20) style map-frame coordinates.

    Returns a list of dicts with `pos_map` (np.ndarray (2,)) and `label`
    (semantic_label int). Empty if the layer has no nodes.
    """
    import json
    try:
        with open(graph_path) as f:
            raw = json.load(f)
    except Exception:
        return []
    rooms: list[dict] = []
    for n in raw.get("nodes", []):
        if n.get("layer") != 4:
            continue
        attrs = n.get("attributes", {})
        pos = attrs.get("position") or [0.0, 0.0, 0.0]
        rooms.append({
            "pos_map": np.asarray(pos[:2], dtype=float),
            "label": int(attrs.get("semantic_label", 0)),
        })
    return rooms


def assign_places_to_rooms(place_centers_map: np.ndarray,
                            rooms: list[dict]) -> Optional[np.ndarray]:
    """For each place center, return the index of the nearest room. None if no rooms."""
    if not rooms or len(place_centers_map) == 0:
        return None
    room_pos = np.array([r["pos_map"] for r in rooms])
    diffs = place_centers_map[:, None, :] - room_pos[None, :, :]
    return np.argmin(np.linalg.norm(diffs, axis=2), axis=1)


def carrying_at(t_unix: float, robot: str) -> bool:
    for ev in CONE_EVENTS:
        if ev["who"] != robot:
            continue
        if ev["pickup_t"] <= t_unix < ev["drop_t"]:
            return True
    return False


def cone_state_at(ev: dict, t_unix: float) -> Optional[tuple[float, float]]:
    """Return (x_utm, y_utm) where this cone is visible at t_unix, or None."""
    if t_unix < ev["spawn_t"]:
        return None
    if t_unix < ev["pickup_t"]:
        return ev["spawn_xy"]
    if t_unix < ev["drop_t"]:
        return None  # in robot's gripper; sprite swap handles this
    return ev["drop_xy"]


# ---- Panel rendering ----------------------------------------------------


def _add_image(ax, img: np.ndarray, xy: tuple[float, float], zoom: float,
               zorder: int = 7) -> None:
    from matplotlib.offsetbox import AnnotationBbox, OffsetImage
    oi = OffsetImage(img, zoom=zoom)
    ab = AnnotationBbox(oi, xy, frameon=False, pad=0.0, zorder=zorder)
    ax.add_artist(ab)


def _draw_scene_graph(ax, graph: dict, anchor: dict, project,
                      face_alpha: float, names: dict[int, str],
                      use_basemap: bool) -> set[int]:
    """Underlay scene-graph regions + traversability places + (optional) object bboxes.

    Regions come from the ROOMS layer; places (MESH_PLACES) are colored by
    their nearest room and a translucent convex hull is drawn per region.
    Returns the set of object labels actually plotted (for the legend).
    """
    import matplotlib.patches as mpatches

    rooms: list[dict] = graph.get("rooms", [])
    place_map = graph.get("place_centers", np.zeros((0, 2)))
    place_assign = graph.get("place_room_idx")

    def _room_color(ri: int) -> str:
        if ri < len(rooms):
            return REGION_LABEL_COLORS.get(int(rooms[ri]["label"]),
                                            REGION_FALLBACK_COLOR)
        return REGION_FALLBACK_COLOR

    # ---- region polygons + center labels ------------------------------
    # Parking-lot rooms: convex hull of their nearest-neighbor place cluster.
    # Intersection rooms: small fixed-size octagon centered on the stored
    # room center (the convex hull would sprawl past the real intersection).
    if rooms and len(place_map):
        try:
            from scipy.spatial import ConvexHull
        except ImportError:
            ConvexHull = None
        place_utm = sgu.apply_anchor_to_xy(place_map, anchor)
        face_a = 0.30 if use_basemap else 0.32
        edge_a = 0.95 if use_basemap else 0.80

        intersection_count = 0
        for ri, room in enumerate(rooms):
            if room.get("excluded"):
                continue
            label = int(room["label"])
            color = _room_color(ri)
            label_text: str
            label_xy_utm: tuple[float, float]
            if label == INTERSECTION_LABEL:
                # Build a regular polygon centered on the stored room center.
                center_utm = sgu.apply_anchor_to_xy(
                    room["pos_map"][None, :], anchor)[0]
                angles = np.linspace(0, 2 * np.pi, INTERSECTION_POLY_SIDES,
                                     endpoint=False) + np.pi / INTERSECTION_POLY_SIDES
                ring_utm = np.column_stack([
                    center_utm[0] + INTERSECTION_POLY_RADIUS_M * np.cos(angles),
                    center_utm[1] + INTERSECTION_POLY_RADIUS_M * np.sin(angles),
                ])
                px, py = project(ring_utm[:, 0], ring_utm[:, 1])
                intersection_count += 1
                place_id = ROOM_PLACE_IDS.get(ri, "")
                label_text = place_id or f"Intersection {intersection_count}"
                # Place text to the right of the octagon — clears the cone
                # and the robot at drop time, and avoids collisions with
                # robot positions from prior experiments at the start frame.
                # Tag sits above the octagon — clears the spot sprite +
                # robot pill (which sit below the spot center).
                label_xy_utm = (float(center_utm[0]),
                                 float(center_utm[1]) + INTERSECTION_POLY_RADIUS_M + 5.0)
            else:
                if place_assign is None:
                    continue
                pts_utm = place_utm[place_assign == ri]
                if len(pts_utm) < 3:
                    continue
                if ConvexHull is not None:
                    try:
                        hull = ConvexHull(pts_utm)
                        poly_utm = pts_utm[hull.vertices]
                    except Exception:
                        poly_utm = pts_utm
                else:
                    poly_utm = pts_utm
                px, py = project(poly_utm[:, 0], poly_utm[:, 1])
                label_text = REGION_LABEL_NAMES.get(label, "Region")
                # Place text at the polygon centroid in UTM.
                label_xy_utm = (float(np.mean(poly_utm[:, 0])),
                                 float(np.mean(poly_utm[:, 1])))

            poly_xy = np.column_stack([px, py])
            ax.add_patch(mpatches.Polygon(
                poly_xy, closed=True, facecolor=color, edgecolor=color,
                alpha=face_a, linewidth=2.0, zorder=1,
            ))
            ax.plot(np.r_[px, px[:1]], np.r_[py, py[:1]], "-",
                    color=color, alpha=edge_a, linewidth=2.6, zorder=1.5)

            # Region text label (centered on polygon).
            tx_p, ty_p = project(np.array([label_xy_utm[0]]),
                                  np.array([label_xy_utm[1]]))
            # Parking-lot label gets the full name; intersection labels are
            # the compact place-id tags. Both sit below sprites (zorder 4)
            # so spots/cones/robot pills remain on top.
            is_intersection = (label == INTERSECTION_LABEL)
            ax.annotate(
                label_text,
                xy=(float(tx_p[0]), float(ty_p[0])),
                xycoords="data",
                ha="center", va="center",
                fontsize=11 if is_intersection else 14,
                fontweight="bold", color="#222",
                bbox=dict(boxstyle="round,pad=0.18", fc=color,
                          ec="white", alpha=0.88),
                zorder=6 if is_intersection else 6,
                # 6 puts both region labels above trail polylines (zorder 4–5)
                # but still below sprites/robot pills (zorder 7+).
            )

    # ---- traversability places ----------------------------------------
    if len(place_map):
        place_utm = sgu.apply_anchor_to_xy(place_map, anchor)
        px, py = project(place_utm[:, 0], place_utm[:, 1])
        if use_basemap:
            # Bright yellow with dark edge — pops against satellite tiles
            # without conflicting with the red/blue trail colors.
            ax.scatter(px, py, s=18, c=PLACE_COLOR_BASEMAP, alpha=0.95,
                       linewidths=0.7, edgecolors="#222", zorder=2)
        else:
            ax.scatter(px, py, s=10, c=PLACE_COLOR_CLEAN, alpha=0.75,
                       linewidths=0, zorder=2)

    # ---- mission-relevant object bboxes (filtered) ---------------------
    plotted: set[int] = set()
    if not MISSION_RELEVANT_LABELS:
        return plotted

    centers = graph["object_centers"]
    if len(centers) == 0:
        return plotted
    centers_utm = sgu.apply_anchor_to_xy(centers, anchor)
    edge_lw = 2.4 if use_basemap else 2.0
    for i in range(len(centers)):
        lab = int(graph["object_labels"][i])
        if lab not in MISSION_RELEVANT_LABELS:
            continue
        # Skip placeholder objects: 1.0×1.0 boxes at the anchor origin.
        if (abs(centers_utm[i, 0] - anchor["t"][0]) < 0.05 and
                abs(centers_utm[i, 1] - anchor["t"][1]) < 0.05):
            continue
        cx, cy = centers[i, 0], centers[i, 1]
        dx, dy = float(graph["object_dims"][i, 0]), float(graph["object_dims"][i, 1])
        yaw = float(graph["object_yaws"][i])
        corners_map = sgu.bbox_corners(cx, cy, dx, dy, yaw)
        corners_utm = sgu.apply_anchor_to_xy(corners_map, anchor)
        cxp, cyp = project(corners_utm[:, 0], corners_utm[:, 1])
        poly_xy = np.column_stack([cxp, cyp])
        color = sgu.class_color(lab)
        ax.add_patch(mpatches.Polygon(
            poly_xy, closed=True, facecolor=color, edgecolor=color,
            alpha=face_alpha, linewidth=edge_lw, zorder=3,
        ))
        plotted.add(lab)
    return plotted


def _build_projector(use_basemap: bool):
    """Return a callable (x_utm, y_utm) -> (x_axes, y_axes)."""
    if not use_basemap:
        return lambda xs, ys: (np.asarray(xs), np.asarray(ys))
    from pyproj import Transformer
    tr = Transformer.from_crs(UTM_EPSG, WEB_MERCATOR, always_xy=True)
    def project(xs, ys):
        x, y = tr.transform(np.asarray(xs), np.asarray(ys))
        return np.asarray(x), np.asarray(y)
    return project


def render_panel(
    ax,
    *,
    t_kf: float,
    title: str,
    use_basemap: bool,
    extent_utm: tuple[float, float, float, float],
    ham: tuple[np.ndarray, np.ndarray, np.ndarray],
    euc: tuple[np.ndarray, np.ndarray, np.ndarray],
    sprites: dict,
    icon_pngs: dict,
    graph: dict,
    anchor: dict,
    label_names: dict[int, str],
    show_trajectory: bool = True,
    show_full_trajectory: bool = True,
) -> set[int]:
    """Render a single keyframe panel into `ax`. Returns labels actually drawn."""
    import matplotlib.image as mpimg

    project = _build_projector(use_basemap)

    # Extent in axes coordinates.
    xmin_u, xmax_u, ymin_u, ymax_u = extent_utm
    cxs, cys = project(
        np.array([xmin_u, xmax_u, xmax_u, xmin_u]),
        np.array([ymin_u, ymin_u, ymax_u, ymax_u]),
    )
    ax.set_xlim(cxs.min(), cxs.max())
    ax.set_ylim(cys.min(), cys.max())
    ax.set_aspect("equal", adjustable="box")
    ax.set_xticks([])
    ax.set_yticks([])
    for spine in ax.spines.values():
        spine.set_visible(False)

    if use_basemap:
        try:
            import contextily as cx
            cx.add_basemap(ax, source=cx.providers.Esri.WorldImagery,
                           crs=WEB_MERCATOR, alpha=0.85, zoom=20,
                           attribution=False)
        except Exception as e:
            print(f"  [basemap] fetch failed: {e}", file=sys.stderr)

    face_alpha = 0.32 if use_basemap else 0.30
    plotted_labels = _draw_scene_graph(ax, graph, anchor, project,
                                       face_alpha=face_alpha, names=label_names,
                                       use_basemap=use_basemap)

    # ---- trajectories: faded full + bold trail-up-to-t_kf ---------------
    if show_trajectory:
        for traj, color in ((euc, EUCLID_COLOR), (ham, HAMILTON_COLOR)):
            t_arr, x_u, y_u = traj
            if len(t_arr) < 2:
                continue
            win_mask = (t_arr >= EXPERIMENT_START_UNIX) & (t_arr <= EXPERIMENT_END_UNIX)
            t_w = t_arr[win_mask]
            x_w = x_u[win_mask]
            y_w = y_u[win_mask]
            if len(t_w) < 2:
                continue
            x_p, y_p = project(x_w, y_w)
            if show_full_trajectory:
                ax.plot(x_p, y_p, "-", color=color, alpha=TRAIL_FADE_ALPHA,
                        linewidth=TRAIL_FADE_LW, zorder=4,
                        solid_capstyle="round")
            cut = np.searchsorted(t_w, t_kf, side="right")
            if cut >= 2:
                ax.plot(x_p[:cut], y_p[:cut], "-", color=color,
                        alpha=TRAIL_HOT_ALPHA, linewidth=TRAIL_HOT_LW,
                        zorder=5, solid_capstyle="round")

    # ---- cones at t_kf --------------------------------------------------
    for ev in CONE_EVENTS:
        pos_utm = cone_state_at(ev, t_kf)
        if pos_utm is None:
            continue
        x_p, y_p = project(np.array([pos_utm[0]]), np.array([pos_utm[1]]))
        _add_image(ax, sprites["cone"], (float(x_p[0]), float(y_p[0])),
                   zoom=CONE_ZOOM, zorder=6)
        # PDDL object ID tag near the cone — small and behind the spot/
        # robot-pill so it doesn't crowd the primary labels. Staggered
        # horizontally so close-together cones at spawn don't stack.
        obj_id = ev.get("object_id")
        if obj_id:
            # Snug to the cone — Hamilton's tag sits directly above the cone,
            # Euclid's goes lower-left. At spawn this puts O103 between the
            # two cone sprites (above the parking-lot label) while O102
            # crowns Hamilton's cone; at drop both tags stay next to their
            # cones instead of drifting east.
            if ev["who"] == "hamilton":
                lbl_dx, lbl_dy = 0.0, 5.0
            else:
                lbl_dx, lbl_dy = -4.0, -5.0
            lbl_x_p, lbl_y_p = project(np.array([pos_utm[0] + lbl_dx]),
                                        np.array([pos_utm[1] + lbl_dy]))
            ax.annotate(
                obj_id,
                xy=(float(lbl_x_p[0]), float(lbl_y_p[0])),
                xycoords="data",
                ha="left", va="center",
                fontsize=10, fontweight="bold", color="#222",
                bbox=dict(boxstyle="round,pad=0.15", fc="#fdb462",
                          ec="white", alpha=0.92),
                zorder=11,  # sits above landmark labels (10) so close
                            # neighbors like Mr.X's car can't clip them
            )

    # ---- spot sprites + name labels at t_kf -----------------------------
    for traj_name, traj, color in (("hamilton", ham, HAMILTON_COLOR),
                                    ("euclid",   euc, EUCLID_COLOR)):
        pos = interp_xy(*traj, t_query=t_kf)
        if pos is None:
            continue
        x_p, y_p = project(np.array([pos[0]]), np.array([pos[1]]))
        carrying = carrying_at(t_kf, traj_name)
        img = sprites["spot_cone_r"] if carrying else sprites["spot_r"]
        _add_image(ax, img, (float(x_p[0]), float(y_p[0])),
                   zoom=SPOT_ZOOM, zorder=8)
        # Color-coded position dot under the sprite.
        ax.scatter([float(x_p[0])], [float(y_p[0])], s=120, color=color,
                   edgecolor="white", linewidths=2.0, zorder=7, alpha=0.85)
        # Robot name label, with the carried object ID when applicable.
        carried_obj = next((ev["object_id"] for ev in CONE_EVENTS
                            if ev["who"] == traj_name and carrying), None)
        label_text = (f"{traj_name.capitalize()} ({carried_obj})"
                      if carried_obj else traj_name.capitalize())
        # Euclid pill is nudged west so it doesn't sit on top of the t4987
        # tag at the start frame (when Euclid's spot has drifted east from
        # a previous experiment).
        lbl_x_offset = -8.0 if traj_name == "euclid" else 0.0
        lbl_x_p, lbl_y_p = project(np.array([pos[0] + lbl_x_offset]),
                                    np.array([pos[1]]) - 6.0)
        ax.annotate(
            label_text,
            xy=(float(lbl_x_p[0]), float(lbl_y_p[0])),
            xycoords="data",
            ha="center", va="top",
            fontsize=20, fontweight="bold", color="white",
            bbox=dict(boxstyle="round,pad=0.30", fc=color,
                      ec="white", alpha=0.95),
            zorder=10,
        )

    # ---- landmark icons + labels ---------------------------------------
    for lm in LANDMARKS:
        png = icon_pngs.get(lm["icon_hex"])
        x_p, y_p = project(np.array([lm["xy_utm"][0]]),
                           np.array([lm["xy_utm"][1]]))
        if png is not None:
            img = mpimg.imread(str(png))
            _add_image(ax, img, (float(x_p[0]), float(y_p[0])),
                       zoom=LMARK_ZOOM, zorder=9)
        # Place text label with offset (interpret offset in UTM metres).
        ox, oy = lm.get("label_offset_m", (0.0, -8.0))
        tx_p, ty_p = project(np.array([lm["xy_utm"][0] + ox]),
                              np.array([lm["xy_utm"][1] + oy]))
        ax.annotate(
            lm["name"],
            xy=(float(tx_p[0]), float(ty_p[0])),
            xycoords="data",
            ha="center", va="center",
            fontsize=22, fontweight="bold",
            color="#222",
            bbox=dict(boxstyle="round,pad=0.30", fc="white",
                      ec="#888", alpha=0.88),
            zorder=10,
        )

    # Title / time chips intentionally omitted — captions are added in the
    # poster layout outside this script.

    return plotted_labels


# ---- Top-level orchestration --------------------------------------------


def compute_extent_utm(
    ham: tuple[np.ndarray, np.ndarray, np.ndarray],
    euc: tuple[np.ndarray, np.ndarray, np.ndarray],
) -> tuple[float, float, float, float]:
    """Bounding box of (window-clipped trajectories ∪ cone events ∪ landmarks)."""
    xs, ys = [], []
    for t_arr, x_u, y_u in (ham, euc):
        m = (t_arr >= EXPERIMENT_START_UNIX) & (t_arr <= EXPERIMENT_END_UNIX)
        xs.append(x_u[m]); ys.append(y_u[m])
    for ev in CONE_EVENTS:
        for k in ("spawn_xy", "drop_xy"):
            xs.append(np.array([ev[k][0]])); ys.append(np.array([ev[k][1]]))
    for lm in LANDMARKS:
        xs.append(np.array([lm["xy_utm"][0]]))
        ys.append(np.array([lm["xy_utm"][1]]))
    x = np.concatenate(xs); y = np.concatenate(ys)
    xmin, xmax = x.min() - EXTENT_PAD_M, x.max() + EXTENT_PAD_M
    ymin, ymax = y.min() - EXTENT_PAD_M, y.max() + EXTENT_PAD_M
    if SQUARE_EXTENT:
        w = xmax - xmin
        h = ymax - ymin
        if h > w:
            extra = (h - w) / 2.0
            xmin -= extra; xmax += extra
        elif w > h:
            extra = (w - h) / 2.0
            ymin -= extra; ymax += extra
    return (xmin, xmax, ymin, ymax)


def add_legend(fig_or_ax, plotted_labels: set[int], label_names: dict[int, str],
               *, location: str = "lower left", compact: bool = True,
               ncols: int = 1,
               bbox_to_anchor: Optional[tuple[float, float]] = None) -> None:
    """Build a legend from trajectory + plotted scene-graph labels.

    `compact=True` collapses scene-graph classes into a single 'Scene-graph
    objects' patch entry so the legend stays small on per-panel figures.
    """
    import matplotlib.lines as mlines
    import matplotlib.patches as mpatches

    handles = [
        mlines.Line2D([], [], color=HAMILTON_COLOR, lw=4, label="Hamilton path"),
        mlines.Line2D([], [], color=EUCLID_COLOR,   lw=4, label="Euclid path"),
        mpatches.Patch(facecolor=REGION_LABEL_COLORS[1],
                       edgecolor=REGION_LABEL_COLORS[1],
                       alpha=0.5, label=REGION_LABEL_NAMES[1]),
        mpatches.Patch(facecolor=REGION_LABEL_COLORS[2],
                       edgecolor=REGION_LABEL_COLORS[2],
                       alpha=0.5, label=REGION_LABEL_NAMES[2]),
        mlines.Line2D([], [], marker="o", color="none",
                      markerfacecolor=PLACE_COLOR_BASEMAP,
                      markeredgecolor="#222", markeredgewidth=0.7,
                      markersize=8, label="Scene Graph Places"),
    ]
    if compact:
        if plotted_labels:
            handles.append(mpatches.Patch(facecolor="#999", edgecolor="#666",
                                          alpha=0.4, label="Mission objects"))
    else:
        for lab in sorted(plotted_labels):
            nm = label_names.get(lab, f"class {lab}")
            col = sgu.class_color(lab)
            handles.append(mpatches.Patch(facecolor=col, edgecolor=col,
                                          alpha=0.6, label=nm))
    legend_kw = dict(handles=handles, loc=location, fontsize=14,
                     frameon=True, framealpha=0.9, ncols=ncols)
    if bbox_to_anchor is not None:
        legend_kw["bbox_to_anchor"] = bbox_to_anchor
    fig_or_ax.legend(**legend_kw)


def _save_fig(fig, path: Path, *, crop_bottom_frac: float = 0.0,
              **savefig_kw) -> None:
    """Save figure, optionally cropping `crop_bottom_frac` of pixels off the bottom."""
    if crop_bottom_frac <= 0.0:
        fig.savefig(path, **savefig_kw)
        return
    import io
    from PIL import Image
    buf = io.BytesIO()
    fig.savefig(buf, format="png", **savefig_kw)
    buf.seek(0)
    img = Image.open(buf)
    crop_px = int(img.height * crop_bottom_frac)
    if crop_px > 0:
        img = img.crop((0, 0, img.width, img.height - crop_px))
    img.save(path)


def render_keyframe_files(
    ham, euc, sprites, icon_pngs, graph, anchor, label_names,
    out_dir: Path,
) -> None:
    import matplotlib.pyplot as plt

    extent_utm = compute_extent_utm(ham, euc)
    out_dir.mkdir(parents=True, exist_ok=True)
    kfs = keyframes()

    for variant in ("clean", "basemap"):
        use_basemap = (variant == "basemap")
        for kf in kfs:
            fig, ax = plt.subplots(figsize=(8, 8), dpi=PANEL_DPI)
            plotted = render_panel(
                ax, t_kf=kf["t"], title=kf["title"],
                use_basemap=use_basemap, extent_utm=extent_utm,
                ham=ham, euc=euc, sprites=sprites, icon_pngs=icon_pngs,
                graph=graph, anchor=anchor, label_names=label_names,
                show_trajectory=kf.get("show_trajectory", True),
                show_full_trajectory=kf.get("show_full_trajectory", True),
            )
            crop_frac = BASEMAP_BOTTOM_CROP_FRAC if use_basemap else 0.0
            # Anchor legend above the bottom-crop strip AND above the Barracks
            # landmark label (Barracks icon sits at ~axes y=0.09 with label at
            # ~y=0.14 in uncropped coords).
            legend_anchor = (0.0, max(crop_frac + 0.02, 0.18))
            add_legend(ax, plotted, label_names, location="lower left",
                       compact=True, bbox_to_anchor=legend_anchor)
            out = out_dir / f"{kf['key']}_{variant}.png"
            _save_fig(fig, out, dpi=PANEL_DPI, bbox_inches="tight",
                      pad_inches=0.05, crop_bottom_frac=crop_frac)
            plt.close(fig)
            print(f"  wrote {out}")

        # Composite 1x3 — square panels in a wide figure.
        fig, axes = plt.subplots(1, 3, figsize=(24, 8.7), dpi=PANEL_DPI)
        # Transparent inter-panel background so the poster shows through.
        fig.patch.set_alpha(0.0)
        plotted_all: set[int] = set()
        for ax, kf in zip(axes, kfs):
            plotted = render_panel(
                ax, t_kf=kf["t"], title=kf["title"],
                use_basemap=use_basemap, extent_utm=extent_utm,
                ham=ham, euc=euc, sprites=sprites, icon_pngs=icon_pngs,
                graph=graph, anchor=anchor, label_names=label_names,
                show_trajectory=kf.get("show_trajectory", True),
                show_full_trajectory=kf.get("show_full_trajectory", True),
            )
            plotted_all |= plotted
        # Per-axis compact legend on the first panel only.
        comp_crop = BASEMAP_BOTTOM_CROP_FRAC if use_basemap else 0.0
        comp_legend_anchor = (0.0, max(comp_crop + 0.02, 0.18))
        add_legend(axes[0], plotted_all, label_names, location="lower left",
                   compact=True, bbox_to_anchor=comp_legend_anchor)
        fig.subplots_adjust(left=0.01, right=0.99, top=0.99, bottom=0.01,
                            wspace=0.025)
        out = out_dir / f"composite_{variant}.png"
        crop_frac = BASEMAP_BOTTOM_CROP_FRAC if use_basemap else 0.0
        _save_fig(fig, out, dpi=PANEL_DPI, bbox_inches="tight",
                  pad_inches=0.06, facecolor="none", edgecolor="none",
                  crop_bottom_frac=crop_frac)
        plt.close(fig)
        print(f"  wrote {out}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--output-dir", type=Path, default=OUT_DIR_DEFAULT)
    ap.add_argument("--skip-basemap", action="store_true",
                    help="render only the clean variants (faster, no tile fetch)")
    args = ap.parse_args()

    print(f"loading hamilton {HAMILTON_CSV}")
    ham = load_utm_trajectory(HAMILTON_CSV)
    print(f"  {len(ham[0])} samples, t in [{ham[0][0]:.0f}, {ham[0][-1]:.0f}]")

    print(f"loading euclid   {EUCLID_CSV}")
    euc = load_utm_trajectory(EUCLID_CSV)
    print(f"  {len(euc[0])} samples, t in [{euc[0][0]:.0f}, {euc[0][-1]:.0f}]")

    ham_len = total_path_length(ham, EXPERIMENT_START_UNIX, EXPERIMENT_END_UNIX)
    euc_len = total_path_length(euc, EXPERIMENT_START_UNIX, EXPERIMENT_END_UNIX)
    print(f"path length within experiment window:")
    print(f"  hamilton: {ham_len:8.2f} m")
    print(f"  euclid:   {euc_len:8.2f} m")

    print(f"loading scene graph {SCENE_GRAPH_JSON}")
    graph = sgu.load_graph_overlay(SCENE_GRAPH_JSON)
    anchor = sgu.load_anchor(ANCHOR_DIR, ANCHOR_KEY)
    if anchor is None:
        print(f"FATAL: missing anchor {ANCHOR_KEY} in {ANCHOR_DIR}", file=sys.stderr)
        return 1
    rooms = load_rooms(SCENE_GRAPH_JSON)
    graph["rooms"] = rooms
    graph["place_room_idx"] = assign_places_to_rooms(graph["place_centers"], rooms)

    # Override hamilton's intersection (room 2) center to track hamilton's
    # spot at the "complete" keyframe — the stored room center is offset
    # from where hamilton actually places his cone / ends up.
    drop_t = max(e["drop_t"] for e in CONE_EVENTS)
    ham_end_xy = interp_xy(*ham, t_query=drop_t + 5.0)
    if ham_end_xy is not None and len(rooms) > 2:
        end_map = np.asarray(ham_end_xy) - np.asarray(anchor["t"])
        rooms[2]["pos_map"] = end_map
        print(f"  hamilton intersection re-centered → UTM "
              f"({ham_end_xy[0]:.1f}, {ham_end_xy[1]:.1f})")

    # Mission-relevant parking lot: the one closest to the cone-spawn area
    # (i.e. the parking lot Mr.X's car / cones are in). Other parking lots
    # in the graph (e.g. the east lot) get marked as excluded and skipped
    # by the renderer.
    spawn_mid_utm = np.mean(
        np.array([list(ev["spawn_xy"]) for ev in CONE_EVENTS]), axis=0)
    spawn_mid_map = spawn_mid_utm - np.asarray(anchor["t"])
    parking_idx = [i for i, r in enumerate(rooms) if int(r["label"]) == 1]
    if parking_idx:
        nearest = min(
            parking_idx,
            key=lambda i: float(np.linalg.norm(rooms[i]["pos_map"] - spawn_mid_map)),
        )
        for i in parking_idx:
            rooms[i]["excluded"] = (i != nearest)
        print(f"  parking lot kept: room[{nearest}]; "
              f"excluded {[i for i in parking_idx if i != nearest]}")

    label_names = graph.get("label_names") or sgu.load_label_names()
    print(f"  {len(graph['object_centers'])} objects, "
          f"{len(graph['place_centers'])} places, "
          f"{len(rooms)} regions, "
          f"{len(label_names)} labels")

    print(f"loading sprites from {ASSETS_DIR}")
    sprites = load_sprite_cache(ASSETS_DIR)

    icon_dir = args.output_dir / "icons"
    print(f"fetching landmark icons → {icon_dir}")
    icon_pngs: dict[str, Optional[Path]] = {}
    for lm in LANDMARKS:
        h = lm["icon_hex"]
        icon_pngs[h] = fetch_openmoji_png(h, icon_dir)
        if icon_pngs[h] is None:
            print(f"  warn: {h} unavailable, will draw text only", file=sys.stderr)

    print(f"rendering keyframes → {args.output_dir}")
    render_keyframe_files(ham, euc, sprites, icon_pngs, graph, anchor,
                          label_names, args.output_dir)
    return 0


if __name__ == "__main__":
    sys.exit(main())
