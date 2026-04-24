"""Scene graph loading + map-frame to lat/lon projection for viz overlays.

Kept deliberately tiny and side-effect-free so `visualize_experiment.py`
can iterate without re-running the anchor preprocessing step.

The anchor JSON files are produced by `anchor_scene_graphs.py`.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Optional

import numpy as np


# Default ade20k-MIT labelspace is what the West Point 2026 dataset was built
# against (dcist_launch_system/config/*/heracles_publisher_node.yaml).
DEFAULT_LABELSPACE_YAML = Path(
    "/home/harel/dcist_ws/src/awesome_dcist_t4/hydra/config/label_spaces/"
    "ade20k_mit_label_space.yaml"
)


# Red-/blue-free qualitative palette — the trajectory lines are #e41a1c (red,
# euclid) and #377eb8 (blue, hamilton), so class colors here skip those hues
# to keep objects visually distinct from robot paths.
CLASS_PALETTE = [
    "#ff7f00",  # orange
    "#4daf4a",  # green
    "#984ea3",  # purple
    "#a65628",  # brown
    "#ffff33",  # yellow
    "#1b9e77",  # teal
    "#d95f02",  # dark orange
    "#e7298a",  # magenta
    "#66a61e",  # olive
    "#e6ab02",  # gold
    "#bc80bd",  # light purple
    "#8c510a",  # dark brown
    "#c51b7d",  # dark pink
    "#01665e",  # dark teal
    "#762a83",  # deep purple
    "#f1a340",  # amber
    "#b2abd2",  # pale lavender
    "#dfc27d",  # tan
    "#35978f",  # muted teal
    "#f781bf",  # pink (kept away from red by saturation)
]


# ---- anchor loading -----------------------------------------------------


def load_anchor(anchor_dir: Path, exp_set: str) -> Optional[dict]:
    """Return the anchor dict for `exp_set`, or None if missing."""
    p = anchor_dir / f"{exp_set}.json"
    if not p.exists():
        return None
    with open(p) as f:
        return json.load(f)


def apply_anchor_to_xy(xy_map: np.ndarray, anchor: dict) -> np.ndarray:
    """Apply (R, t) to N×2 map-frame XY → UTM XY."""
    R = np.asarray(anchor["R"])
    t = np.asarray(anchor["t"])
    return (R @ xy_map.T).T + t


def utm_to_latlon(xy_utm: np.ndarray, epsg: str) -> tuple[np.ndarray, np.ndarray]:
    from pyproj import Transformer
    tr = Transformer.from_crs(epsg, "EPSG:4326", always_xy=True)
    lon, lat = tr.transform(xy_utm[:, 0], xy_utm[:, 1])
    return np.asarray(lat), np.asarray(lon)


def project_to_latlon(xy_map: np.ndarray, anchor: dict) -> tuple[np.ndarray, np.ndarray]:
    utm = apply_anchor_to_xy(xy_map, anchor)
    return utm_to_latlon(utm, anchor["utm_epsg"])


# ---- scene graph loading ------------------------------------------------


def _quat_yaw(R_mat: np.ndarray) -> float:
    """Yaw of a 3x3 rotation matrix (rotation about +Z, world frame)."""
    return float(np.arctan2(R_mat[1, 0], R_mat[0, 0]))


def _labelspace_from_json(graph_path: Path, layer: int = 2, partition: int = 0) -> dict[int, str]:
    """Read {label_id: name} from `metadata.labelspaces._l<L>p<P>` in the graph JSON.

    The spark_dsg Python bindings leave `get_labelspace().labels_to_names`
    empty for these clean graphs, but the mapping is present in the raw JSON
    under `metadata.labelspaces._l<layer>p<partition>` as a list of
    `[label_id, name]` pairs.
    """
    try:
        with open(graph_path) as f:
            data = json.load(f)
    except Exception:
        return {}
    key = f"_l{layer}p{partition}"
    entries = (data.get("metadata") or {}).get("labelspaces", {}).get(key, []) or []
    out: dict[int, str] = {}
    for entry in entries:
        try:
            out[int(entry[0])] = str(entry[1])
        except (IndexError, TypeError, ValueError):
            continue
    return out


def load_graph_overlay(graph_path: Path) -> dict:
    """Load OBJECTS and MESH_PLACES from a spark_dsg JSON.

    Returns a dict with numpy arrays ready for plotting:
      object_centers  (N, 2)   — world-frame (map) x,y
      object_dims     (N, 2)   — bbox dx, dy
      object_yaws     (N,)     — bbox yaw (radians, about +Z)
      object_labels   (N,)     — semantic_label ints
      place_centers   (M, 2)
      label_names     dict     — {label_id: name} parsed from the JSON metadata
    """
    import spark_dsg as dsg

    g = dsg.DynamicSceneGraph.load(str(graph_path))

    obj_layer = g.get_layer("OBJECTS")
    centers, dims, yaws, labels = [], [], [], []
    for node in obj_layer.nodes:
        a = node.attributes
        bbox = a.bounding_box
        c = np.asarray(bbox.world_P_center)[:2]
        d = np.asarray(bbox.dimensions)[:2]
        R_mat = np.asarray(bbox.world_R_center)
        yaw = _quat_yaw(R_mat) if R_mat.shape == (3, 3) else 0.0
        centers.append(c)
        dims.append(d)
        yaws.append(yaw)
        labels.append(int(a.semantic_label))

    places_layer = g.get_layer("MESH_PLACES")
    place_centers = np.array(
        [np.asarray(n.attributes.position)[:2] for n in places_layer.nodes]
    ) if places_layer.num_nodes() > 0 else np.zeros((0, 2))

    return {
        "object_centers": np.asarray(centers) if centers else np.zeros((0, 2)),
        "object_dims":    np.asarray(dims)    if dims else np.zeros((0, 2)),
        "object_yaws":    np.asarray(yaws)    if yaws else np.zeros((0,)),
        "object_labels":  np.asarray(labels, dtype=int) if labels else np.zeros((0,), dtype=int),
        "place_centers":  place_centers,
        "label_names":    _labelspace_from_json(graph_path, layer=2, partition=0),
    }


# ---- plotting helpers ---------------------------------------------------


def class_color(label: int) -> tuple[float, float, float]:
    """Deterministic color per semantic label id from CLASS_PALETTE."""
    hex_col = CLASS_PALETTE[int(label) % len(CLASS_PALETTE)]
    return tuple(int(hex_col[i:i + 2], 16) / 255.0 for i in (1, 3, 5))


def load_label_names(yaml_path: Path = DEFAULT_LABELSPACE_YAML) -> dict[int, str]:
    """Return {label_id: name} from a hydra label_space YAML.

    Returns an empty dict if the file is missing or malformed — callers fall
    back to ``class <id>`` strings for unknown labels.
    """
    try:
        import yaml
    except ImportError:
        return {}
    if not yaml_path.exists():
        return {}
    try:
        with open(yaml_path) as f:
            data = yaml.safe_load(f)
    except Exception:
        return {}
    out: dict[int, str] = {}
    for entry in data.get("label_names", []) or []:
        try:
            out[int(entry["label"])] = str(entry["name"])
        except (KeyError, TypeError, ValueError):
            continue
    return out


def bbox_corners(cx: float, cy: float, dx: float, dy: float, yaw: float) -> np.ndarray:
    """Return (4, 2) corners of an axis-aligned bbox rotated by yaw about center."""
    hx, hy = dx / 2.0, dy / 2.0
    local = np.array([
        [-hx, -hy],
        [ hx, -hy],
        [ hx,  hy],
        [-hx,  hy],
    ])
    c, s = np.cos(yaw), np.sin(yaw)
    R = np.array([[c, -s], [s, c]])
    return (R @ local.T).T + np.array([cx, cy])
