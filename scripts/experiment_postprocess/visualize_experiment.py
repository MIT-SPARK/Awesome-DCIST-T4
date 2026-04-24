#!/usr/bin/env python3
"""Visualize a robot experiment from the master CSV.

Reads one row of `experiments.csv`, renders:
  * `trajectory.png` — overhead GPS trajectory on a satellite basemap (via
    contextily; falls back to plain lat/lon axes with --no-basemap).
  * `<robot>/rgb.mp4`   — FPV video from /<ns>/<ns>_zed/rgb/image_rect_color
  * `<robot>/depth.mp4` — colorized depth from /<ns>/<ns>_zed/depth/depth_registered

Run with the ROS 2 Jazzy workspace sourced and `spark_env` python on PATH.
"""

from __future__ import annotations

import argparse
import csv
import datetime
import subprocess
import sys
from pathlib import Path
from typing import Iterable, Optional

import numpy as np

_THIS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(_THIS_DIR))
import _bag_utils as bu  # noqa: E402
import _scene_graph_utils as sgu  # noqa: E402


# experiment_set -> clean scene graph filename (under --scene-graph-dir).
# Anchor JSONs under --anchor-dir are named after the graph stem
# (e.g. alpha_thurs_clean_2.json -> alpha_thurs_clean_2.json).
SCENE_GRAPHS = {
    "april_15_bravo": "bravo_wed_clean_bbox_corrected.json",
    "april_16_alpha": "alpha_thurs_clean_2.json",
    "april_17_alpha": "alpha_thurs_clean_2.json",
}

# (experiment_set, robot) pairs whose UTM trajectory CSV should be read from
# `--hydra-dir` instead of `--fused-dir` (GPS was unusable for these runs).
HYDRA_OVERRIDE = {("april_17_alpha", "hamilton")}


# ---- CSV loading --------------------------------------------------------


def load_master(path: Path) -> list[dict]:
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def row_robots(row: dict) -> list[str]:
    return [r for r in row.get("robots", "").split(bu.LIST_SEP) if r]


def row_bags(row: dict) -> list[Path]:
    return [Path(p) for p in row.get("bag_paths", "").split(bu.LIST_SEP) if p]


def row_candidate_end_times(row: dict) -> list[float]:
    """Return sorted list of candidate end times as unix timestamps."""
    raw = row.get("candidate_end_times", "").strip()
    if not raw:
        return []
    out: list[float] = []
    for s in raw.split(bu.LIST_SEP):
        s = s.strip()
        if not s:
            continue
        try:
            dt = datetime.datetime.fromisoformat(s.replace("Z", "+00:00"))
            out.append(dt.timestamp())
        except ValueError:
            pass
    return sorted(out)


def row_bags_for_robot(row: dict, robot_ns: str) -> list[Path]:
    """Return the bag mcaps that actually carry <robot_ns>'s local data.

    Prefers the per-robot `<robot>_bag_paths` column (populated when the
    robot has its own sibling folder). Falls back to the flat `bag_paths`
    for backward compatibility.
    """
    specific = row.get(f"{robot_ns}_bag_paths", "")
    if specific:
        return [Path(p) for p in specific.split(bu.LIST_SEP) if p]
    return row_bags(row)


# ---- trajectory plotting ------------------------------------------------


def _gps_from_bag(bag_file: Path, robot_ns: str, t_lo: float, t_hi: float):
    """Load (times, lat_lon_alts) filtered to [t_lo, t_hi] for one bag.

    Drops GPS readings with status < 0 (NO_FIX / stale frozen coordinates).
    """
    from mcap.reader import make_reader as _make_reader
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    MsgType = get_message("sensor_msgs/msg/NavSatFix")
    topic = f"/{robot_ns}/fix"
    start_ns = int(t_lo * 1e9)
    end_ns = int(t_hi * 1e9)

    times, llas = [], []
    try:
        with open(bag_file, "rb") as fh:
            reader = _make_reader(fh, validate_crcs=False)
            for _schema, channel, message in reader.iter_messages(
                topics=[topic], start_time=start_ns, end_time=end_ns
            ):
                msg = deserialize_message(message.data, MsgType)
                if msg.status.status < 0:
                    continue
                if not (np.isfinite(msg.latitude) and np.isfinite(msg.longitude)):
                    continue
                times.append(message.log_time * 1e-9)
                llas.append([msg.latitude, msg.longitude, msg.altitude])
    except Exception as e:
        print(f"    [gps] {bag_file.name}: {e}", file=sys.stderr)

    if not times:
        return np.array([]), np.zeros((0, 3))
    return np.array(times), np.array(llas)


def _collect_gps_trajectory(bag_files: list[Path], robot_ns: str, t_lo: float, t_hi: float):
    parts = []
    for bf in bag_files:
        t, lla = _gps_from_bag(bf, robot_ns, t_lo, t_hi)
        if len(t) > 0:
            parts.append((t, lla))
    if not parts:
        return np.array([]), np.zeros((0, 3))
    times = np.concatenate([t for t, _ in parts])
    lla = np.concatenate([x for _, x in parts], axis=0)
    order = np.argsort(times)
    return times[order], lla[order]


def _fused_csv_path(bag_file: Path, fused_dir: Path) -> Optional[Path]:
    """Return path to precomputed fused trajectory CSV for this bag, or None."""
    session = bag_file.parent.parent.name   # e.g. april_16_alpha_hamilton
    bag_dir_name = bag_file.parent.name     # e.g. recorded_data
    p = fused_dir / f"{session}__{bag_dir_name}.csv"
    return p if p.exists() else None


def _resolve_traj_csv(
    bag_file: Path,
    exp_set: str,
    robot_ns: str,
    fused_dir: Optional[Path],
    hydra_dir: Optional[Path],
) -> Optional[Path]:
    """Pick UTM CSV path for (exp_set, robot), falling back to fused_dir."""
    if (exp_set, robot_ns) in HYDRA_OVERRIDE and hydra_dir is not None:
        p = _fused_csv_path(bag_file, hydra_dir)
        if p is not None:
            return p
    if fused_dir is None:
        return None
    return _fused_csv_path(bag_file, fused_dir)


def _collect_fused_trajectory(
    bag_files: list[Path],
    fused_dir: Optional[Path],
    t_lo: float,
    t_hi: float,
    exp_set: str = "",
    robot_ns: str = "",
    hydra_dir: Optional[Path] = None,
):
    """Load fused GPS+odom trajectory from precomputed CSVs.

    Positions in the CSV are UTM easting/northing (zone 18N, EPSG:32618 —
    West Point dataset). Converts to lat/lon for plotting.
    Returns (times, lat_arr, lon_arr); empty arrays if unavailable.
    """
    if fused_dir is None and hydra_dir is None:
        return np.array([]), np.array([]), np.array([])

    from pyproj import Transformer
    from robotdatapy.data import PoseData
    from robotdatapy.data.pose_data import KIMERA_MULTI_GT_CSV_OPTIONS

    utm_to_wgs84 = Transformer.from_crs("EPSG:32618", "EPSG:4326", always_xy=True)

    all_t, all_lat, all_lon = [], [], []
    for bf in bag_files:
        csv_path = _resolve_traj_csv(bf, exp_set, robot_ns, fused_dir, hydra_dir)
        if csv_path is None:
            continue
        try:
            pd_obj = PoseData.from_csv(
                str(csv_path), csv_options=KIMERA_MULTI_GT_CSV_OPTIONS, time_tol=30.0
            )
        except Exception as e:
            print(f"    [fused] {csv_path.name}: {e}", file=sys.stderr)
            continue
        mask = (pd_obj.times >= t_lo) & (pd_obj.times <= t_hi)
        if not np.any(mask):
            continue
        poses = pd_obj.all_poses()[mask]
        east = poses[:, 0, 3]
        north = poses[:, 1, 3]
        lon_arr, lat_arr = utm_to_wgs84.transform(east, north)
        all_t.append(pd_obj.times[mask])
        all_lat.append(lat_arr)
        all_lon.append(lon_arr)

    if not all_t:
        return np.array([]), np.array([]), np.array([])
    times = np.concatenate(all_t)
    lats = np.concatenate(all_lat)
    lons = np.concatenate(all_lon)
    order = np.argsort(times)
    return times[order], lats[order], lons[order]


def _xy_from_map(xy_map, anchor, to_merc):
    """Map-frame XY → (x, y) in basemap coords (Web Mercator) or lat/lon."""
    if len(xy_map) == 0:
        return np.array([]), np.array([])
    lat, lon = sgu.project_to_latlon(xy_map, anchor)
    if to_merc is None:
        return np.asarray(lon), np.asarray(lat)
    x, y = to_merc.transform(lon, lat)
    return np.asarray(x), np.asarray(y)


def _draw_overlay(ax, overlay: dict, to_merc, all_xy: list) -> None:
    import matplotlib.patches as mpatches

    graph = overlay["graph"]
    anchor = overlay["anchor"]
    mode = overlay.get("mode", "points")
    label_names: dict = overlay.get("label_names", {}) or {}

    def _lbl_text(lab: int) -> str:
        return label_names.get(int(lab), f"class {int(lab)}")

    # ---- traversability places (underlay) ----
    pc = graph.get("place_centers")
    if pc is not None and len(pc) > 0:
        px, py = _xy_from_map(pc, anchor, to_merc)
        ax.scatter(px, py, s=4, c="#6aa2ff", alpha=0.35, linewidths=0,
                   zorder=1, label="traversability places")
        all_xy.append(np.column_stack([px, py]))

    centers = graph.get("object_centers")
    if centers is None or len(centers) == 0:
        return

    cx_, cy_ = _xy_from_map(centers, anchor, to_merc)
    labels = graph["object_labels"]
    seen_in_legend: set[int] = set()

    if mode == "points":
        # Deterministic per-class color; one legend entry per class.
        colors_arr = np.array([sgu.class_color(int(l)) for l in labels])
        for lab in np.unique(labels):
            m = labels == lab
            lbl = None if int(lab) in seen_in_legend else _lbl_text(lab)
            seen_in_legend.add(int(lab))
            ax.scatter(cx_[m], cy_[m], s=30, c=colors_arr[m],
                       edgecolor="white", linewidth=0.5, zorder=3, label=lbl)
    else:  # bbox
        dims = graph["object_dims"]
        yaws = graph["object_yaws"]
        for i in range(len(centers)):
            lab = int(labels[i])
            color = sgu.class_color(lab)
            corners_map = sgu.bbox_corners(
                centers[i, 0], centers[i, 1],
                float(dims[i, 0]), float(dims[i, 1]), float(yaws[i]),
            )
            cx_poly, cy_poly = _xy_from_map(corners_map, anchor, to_merc)
            poly = np.column_stack([cx_poly, cy_poly])
            lbl = None if lab in seen_in_legend else _lbl_text(lab)
            seen_in_legend.add(lab)
            ax.add_patch(mpatches.Polygon(
                poly, closed=True, fill=False, edgecolor=color, linewidth=1.4,
                zorder=3, label=lbl,
            ))
        # Keep extents anchored on object centers too so axes fit the overlay
        # even when no robot trajectory is drawn.
        all_xy.append(np.column_stack([cx_, cy_]))

    if mode == "points":
        all_xy.append(np.column_stack([cx_, cy_]))


def _plot_trajectory_inner(
    row: dict,
    output_path: Path,
    use_basemap: bool,
    t_lo: float,
    t_hi: float,
    t_hi_override: Optional[float],
    get_latlon,   # callable(robot_ns) -> (lat_arr, lon_arr) or empty arrays
    label_suffix: str = "",
    overlay: Optional[dict] = None,   # {"mode": "bbox"|"points", "graph": dict, "anchor": dict}
) -> None:
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(10, 10))
    colors = ["#e41a1c", "#377eb8", "#4daf4a", "#984ea3"]
    plotted = False
    all_xy = []

    to_merc = None
    if use_basemap:
        try:
            import contextily as cx  # noqa: F401
            from pyproj import Transformer

            to_merc = Transformer.from_crs("EPSG:4326", "EPSG:3857", always_xy=True)
        except ImportError:
            print("contextily/pyproj missing; falling back to plain lat/lon.", file=sys.stderr)
            use_basemap = False

    # Scene-graph underlay drawn before trajectories so paths sit on top.
    if overlay is not None:
        _draw_overlay(ax, overlay, to_merc if use_basemap else None, all_xy)

    for i, rns in enumerate(row_robots(row)):
        lat, lon = get_latlon(rns)
        if len(lat) < 2:
            print(f"  {rns}: no trajectory points in window", file=sys.stderr)
            continue
        if use_basemap:
            x, y = to_merc.transform(lon, lat)
        else:
            x, y = lon, lat
        color = colors[i % len(colors)]
        ax.plot(x, y, "-", color=color, linewidth=2.0, label=rns, alpha=0.9, zorder=6)
        ax.scatter(x[0], y[0], marker="o", s=80, color=color, edgecolor="white", zorder=7,
                   label=f"{rns} start")
        ax.scatter(x[-1], y[-1], marker="s", s=80, color=color, edgecolor="white", zorder=7,
                   label=f"{rns} end")
        all_xy.append(np.column_stack([x, y]))
        plotted = True

    # overlay-only plots (no robot trajectory data) should still render.
    if not plotted and not all_xy:
        print(f"  no trajectory data to plot{label_suffix}; writing placeholder", file=sys.stderr)
        ax.text(0.5, 0.5, f"no trajectory data{label_suffix}", transform=ax.transAxes, ha="center")
        output_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(output_path, dpi=200)
        plt.close(fig)
        return

    ax.set_aspect("equal", adjustable="datalim")
    if use_basemap:
        stacked = np.concatenate(all_xy, axis=0)
        pad_x = max(30.0, (stacked[:, 0].max() - stacked[:, 0].min()) * 0.15)
        pad_y = max(30.0, (stacked[:, 1].max() - stacked[:, 1].min()) * 0.15)
        ax.set_xlim(stacked[:, 0].min() - pad_x, stacked[:, 0].max() + pad_x)
        ax.set_ylim(stacked[:, 1].min() - pad_y, stacked[:, 1].max() + pad_y)
        try:
            import contextily as cx

            cx.add_basemap(ax, source=cx.providers.Esri.WorldImagery, crs="EPSG:3857")
            ax.set_xlabel("x (m, Web Mercator)")
            ax.set_ylabel("y (m, Web Mercator)")
        except Exception as e:
            print(f"  basemap fetch failed ({e}); plot has no tiles", file=sys.stderr)
    else:
        ax.set_xlabel("longitude")
        ax.set_ylabel("latitude")

    dur = t_hi - t_lo
    end_iso = datetime.datetime.fromtimestamp(t_hi, tz=datetime.timezone.utc).strftime("%H:%M:%S")
    end_label = f"end {end_iso} UTC  ({dur:.0f} s)"
    if t_hi_override is None:
        end_label += f"  [{row['end_reason']}]"
    ax.set_title(f"{row['experiment_id']}{label_suffix}\n{end_label}")
    ax.legend(loc="best", fontsize=9)
    ax.grid(True, linestyle=":", alpha=0.5)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"  wrote {output_path}")


def plot_trajectory(
    row: dict,
    output_path: Path,
    use_basemap: bool = True,
    t_hi_override: Optional[float] = None,
    fused_dir: Optional[Path] = None,
    hydra_dir: Optional[Path] = None,
    overlay: Optional[dict] = None,
    raw_gps: bool = True,
    fused: bool = True,
    fused_suffix: str = "_fused",
) -> None:
    """Plot trajectory from raw GPS (status>=0 only) and/or fused, with optional overlay."""
    t_lo = float(row["start_time_unix"])
    t_hi = t_hi_override if t_hi_override is not None else float(row["end_time_unix"])
    exp_set = row.get("experiment_set", "")

    # --- raw GPS plot ---
    if raw_gps:
        def gps_latlon(rns):
            _, lla = _collect_gps_trajectory(row_bags_for_robot(row, rns), rns, t_lo, t_hi)
            if len(lla) < 2:
                return np.array([]), np.array([])
            return lla[:, 0], lla[:, 1]

        _plot_trajectory_inner(
            row, output_path, use_basemap, t_lo, t_hi, t_hi_override,
            gps_latlon, overlay=overlay,
        )

    # --- fused plot (alongside raw GPS, not replacing it) ---
    if fused and (fused_dir is not None or hydra_dir is not None):
        fused_path = (
            output_path
            if not raw_gps
            else output_path.with_name(output_path.stem + fused_suffix + output_path.suffix)
        )

        def fused_latlon(rns):
            _, lat, lon = _collect_fused_trajectory(
                row_bags_for_robot(row, rns), fused_dir, t_lo, t_hi,
                exp_set=exp_set, robot_ns=rns, hydra_dir=hydra_dir,
            )
            return lat, lon

        _plot_trajectory_inner(
            row, fused_path, use_basemap, t_lo, t_hi, t_hi_override,
            fused_latlon, label_suffix=" [fused]", overlay=overlay,
        )


# ---- video extraction ---------------------------------------------------


def _ffmpeg_exe() -> str:
    try:
        import imageio_ffmpeg

        return imageio_ffmpeg.get_ffmpeg_exe()
    except ImportError:
        import shutil

        exe = shutil.which("ffmpeg")
        if exe is None:
            raise RuntimeError("ffmpeg not found: `pip install imageio-ffmpeg` or install it")
        return exe


class FfmpegWriter:
    """Stream raw BGR24 frames to ffmpeg via stdin."""

    def __init__(self, output_path: Path, width: int, height: int, fps: float):
        self.output_path = output_path
        output_path.parent.mkdir(parents=True, exist_ok=True)
        cmd = [
            _ffmpeg_exe(),
            "-y",
            "-f", "rawvideo",
            "-vcodec", "rawvideo",
            "-pix_fmt", "bgr24",
            "-s", f"{width}x{height}",
            "-r", f"{fps}",
            "-i", "-",
            "-an",
            "-c:v", "libx264",
            "-preset", "medium",
            "-crf", "23",
            "-pix_fmt", "yuv420p",
            str(output_path),
        ]
        self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stderr=subprocess.DEVNULL)
        self.width = width
        self.height = height

    def write(self, frame_bgr) -> None:
        if frame_bgr.shape[:2] != (self.height, self.width):
            import cv2

            frame_bgr = cv2.resize(frame_bgr, (self.width, self.height))
        self.proc.stdin.write(frame_bgr.tobytes())

    def close(self) -> None:
        try:
            self.proc.stdin.close()
        except BrokenPipeError:
            pass
        self.proc.wait()


def _decode_rgb(msg) -> np.ndarray:
    from cv_bridge import CvBridge

    bridge = CvBridge()
    return bridge.imgmsg_to_cv2(msg, "bgr8")


def _decode_depth_colormap(msg, dmin: float = 0.3, dmax: float = 10.0) -> np.ndarray:
    import cv2
    from cv_bridge import CvBridge

    bridge = CvBridge()
    raw = bridge.imgmsg_to_cv2(msg, "32FC1")  # meters, float32
    nan = ~np.isfinite(raw)
    filled = np.where(nan, dmin, raw)
    clipped = np.clip(filled, dmin, dmax)
    normed = ((clipped - dmin) / (dmax - dmin) * 255.0).astype(np.uint8)
    colored = cv2.applyColorMap(normed, cv2.COLORMAP_TURBO)
    colored[nan] = 0
    return colored


def _t_from_header_or_bag(msg, t_ns: int) -> float:
    try:
        hsec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if hsec > 0:
            return hsec
    except Exception:
        pass
    return t_ns * 1e-9


def extract_videos_for_robot(
    bag_files: list[Path],
    robot_ns: str,
    t_lo: float,
    t_hi: float,
    output_dir: Path,
    rgb_fps: float,
    depth_fps: float,
) -> None:
    rgb_topic = f"/{robot_ns}/{robot_ns}_zed/rgb/image_rect_color"
    depth_topic = f"/{robot_ns}/{robot_ns}_zed/depth/depth_registered"
    topics = [rgb_topic, depth_topic]

    rgb_writer: Optional[FfmpegWriter] = None
    depth_writer: Optional[FfmpegWriter] = None
    rgb_period = 1.0 / rgb_fps
    depth_period = 1.0 / depth_fps
    next_rgb_t: Optional[float] = None
    next_depth_t: Optional[float] = None
    rgb_count = 0
    depth_count = 0

    output_dir.mkdir(parents=True, exist_ok=True)

    for bag_file in bag_files:
        bag_dir = bag_file.parent
        t_lo_bag, t_hi_bag = bu.bag_time_range(bag_dir)
        if t_hi_bag < t_lo or t_lo_bag > t_hi:
            continue  # no overlap
        print(f"    scanning {bag_dir.name} for {robot_ns} images …")
        for topic, t_ns, msg in bu.iter_messages(bag_dir, topics=topics):
            t = _t_from_header_or_bag(msg, t_ns)
            if t < t_lo:
                continue
            if t > t_hi:
                break  # topics filtered; once past window we're done for this bag
            if topic == rgb_topic:
                if next_rgb_t is not None and t < next_rgb_t:
                    continue
                frame = _decode_rgb(msg)
                if rgb_writer is None:
                    h, w = frame.shape[:2]
                    rgb_writer = FfmpegWriter(output_dir / "rgb.mp4", w, h, rgb_fps)
                rgb_writer.write(frame)
                rgb_count += 1
                next_rgb_t = (t if next_rgb_t is None else next_rgb_t) + rgb_period
                while next_rgb_t <= t:
                    next_rgb_t += rgb_period
            elif topic == depth_topic:
                if next_depth_t is not None and t < next_depth_t:
                    continue
                frame = _decode_depth_colormap(msg)
                if depth_writer is None:
                    h, w = frame.shape[:2]
                    depth_writer = FfmpegWriter(output_dir / "depth.mp4", w, h, depth_fps)
                depth_writer.write(frame)
                depth_count += 1
                next_depth_t = (t if next_depth_t is None else next_depth_t) + depth_period
                while next_depth_t <= t:
                    next_depth_t += depth_period

    if rgb_writer is not None:
        rgb_writer.close()
        print(f"    {robot_ns}: wrote {rgb_count} RGB frames → {rgb_writer.output_path}")
    else:
        print(f"    {robot_ns}: no RGB frames in window", file=sys.stderr)

    if depth_writer is not None:
        depth_writer.close()
        print(f"    {robot_ns}: wrote {depth_count} depth frames → {depth_writer.output_path}")
    else:
        print(f"    {robot_ns}: no depth frames in window", file=sys.stderr)


# ---- main ---------------------------------------------------------------


def _load_overlay(row: dict, args, cache: dict):
    """Return (graph_dict, anchor_dict) for this row, or (None, None).

    Cached per experiment_set so we parse the JSON once per run.
    """
    exp_set = row.get("experiment_set", "")
    if exp_set not in SCENE_GRAPHS:
        return None, None
    if exp_set in cache:
        return cache[exp_set]

    graph = anchor = None
    sg_dir = getattr(args, "scene_graph_dir", None)
    anchor_dir = getattr(args, "anchor_dir", None)
    if sg_dir is not None:
        graph_path = sg_dir / SCENE_GRAPHS[exp_set]
        if graph_path.exists():
            try:
                graph = sgu.load_graph_overlay(graph_path)
            except Exception as e:
                print(f"  [overlay] failed to load graph {graph_path}: {e}", file=sys.stderr)
                graph = None
        else:
            print(f"  [overlay] missing scene graph JSON: {graph_path}", file=sys.stderr)
    if anchor_dir is not None:
        graph_name = SCENE_GRAPHS.get(exp_set, "")
        anchor_key = Path(graph_name).stem if graph_name else exp_set
        anchor = sgu.load_anchor(anchor_dir, anchor_key)
        if anchor is None:
            print(f"  [overlay] missing anchor {anchor_key!r} in {anchor_dir}",
                  file=sys.stderr)

    cache[exp_set] = (graph, anchor)
    return graph, anchor


def process_row(row: dict, out_root: Path, args, overlay_cache: dict) -> None:
    exp_id = row["experiment_id"]
    out_dir = out_root / exp_id
    print(f"\n=== {exp_id} ===")
    print(f"  window: {row['start_time_iso']} .. {row['end_time_iso']}  "
          f"({row['duration_s']}s, {row['end_reason']})")

    candidates = row_candidate_end_times(row)
    current_end = float(row["end_time_unix"])
    # Videos run to the latest candidate (or current end if no candidates).
    video_end = max(candidates) if candidates else current_end

    if not args.skip_trajectory:
        fused_dir = getattr(args, "fused_dir", None)
        hydra_dir = getattr(args, "hydra_dir", None)
        draw_raw = not args.skip_raw_gps
        # Main trajectory uses the stored (selected) end time. When raw GPS is
        # on, plot_trajectory writes trajectory.png + trajectory_fused.png;
        # when skipped, it writes only the fused variant to the given path.
        main_path = (out_dir / "trajectory.png") if draw_raw else (out_dir / "trajectory_fused.png")
        plot_trajectory(row, main_path,
                        use_basemap=not args.no_basemap,
                        fused_dir=fused_dir, hydra_dir=hydra_dir,
                        raw_gps=draw_raw)
        for i, t_hi in enumerate(candidates):
            cand_path = (
                out_dir / f"trajectory_cand_{i:02d}.png" if draw_raw
                else out_dir / f"trajectory_cand_{i:02d}_fused.png"
            )
            plot_trajectory(
                row, cand_path,
                use_basemap=not args.no_basemap,
                t_hi_override=t_hi,
                fused_dir=fused_dir,
                hydra_dir=hydra_dir,
                raw_gps=draw_raw,
            )

        # Scene-graph overlay plots (bbox + points), emitted on the fused
        # trajectory only. Skipped when the experiment_set has no graph
        # configured or no anchor JSON is available.
        if not args.skip_scene_graph:
            ov_graph, ov_anchor = _load_overlay(row, args, overlay_cache)
            if ov_graph is not None and ov_anchor is not None:
                # Prefer names embedded in the graph JSON; fall back to the
                # global YAML only when the JSON has no labelspace.
                label_names = ov_graph.get("label_names") or overlay_cache.get(
                    "__label_names__", {}
                )
                for mode in ("bbox", "points"):
                    overlay = {"mode": mode, "graph": ov_graph,
                               "anchor": ov_anchor, "label_names": label_names}
                    plot_trajectory(
                        row,
                        out_dir / f"trajectory_fused_sg_{mode}.png",
                        use_basemap=not args.no_basemap,
                        fused_dir=fused_dir,
                        hydra_dir=hydra_dir,
                        overlay=overlay,
                        raw_gps=False,
                        fused=True,
                        fused_suffix="",
                    )

    if not args.skip_video:
        for rns in row_robots(row):
            extract_videos_for_robot(
                bag_files=row_bags_for_robot(row, rns),
                robot_ns=rns,
                t_lo=float(row["start_time_unix"]),
                t_hi=video_end,
                output_dir=out_dir / rns,
                rgb_fps=args.rgb_fps,
                depth_fps=args.depth_fps,
            )


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--master", type=Path, required=True, help="experiments.csv path")
    ap.add_argument("--experiment-id", type=str, default=None,
                    help="experiment_id to process (omit when using --all)")
    ap.add_argument("--all", action="store_true", help="process every row in the CSV")
    ap.add_argument("--output-root", type=Path, default=Path("viz_out"))
    ap.add_argument("--rgb-fps", type=float, default=10.0)
    ap.add_argument("--depth-fps", type=float, default=10.0)
    ap.add_argument("--no-basemap", action="store_true",
                    help="skip satellite basemap (plot plain lat/lon)")
    ap.add_argument("--skip-trajectory", action="store_true")
    ap.add_argument("--skip-video", action="store_true")
    ap.add_argument("--skip-raw-gps", action="store_true",
                    help="skip the raw-GPS plots (trajectory.png, "
                         "trajectory_cand_*.png) which scan /fix from the mcap")
    ap.add_argument("--skip-scene-graph", action="store_true",
                    help="skip the two scene-graph overlay plots")
    ap.add_argument(
        "--fused-dir", type=Path, default=None,
        help="folder with precomputed fused trajectory CSVs; generates trajectory_fused.png alongside trajectory.png",
    )
    ap.add_argument(
        "--hydra-dir", type=Path,
        default=Path("/home/harel/data/west_point_2026/processed/hydra_merged"),
        help="hydra-anchored CSV folder; used instead of --fused-dir for "
             "(experiment_set, robot) pairs listed in HYDRA_OVERRIDE (e.g. "
             "april_17_alpha hamilton).",
    )
    ap.add_argument(
        "--scene-graph-dir", type=Path,
        default=Path("/data/dcist/west_point_2026/clean_graphs"),
        help="folder with clean spark_dsg JSON scene graphs",
    )
    ap.add_argument(
        "--anchor-dir", type=Path,
        default=Path("/home/harel/data/west_point_2026/processed/scene_graph_anchors"),
        help="folder with <experiment_set>.json anchor files (run anchor_scene_graphs.py first)",
    )
    ap.add_argument(
        "--labelspace-yaml", type=Path, default=None,
        help="hydra label_space YAML for class names (default: ade20k_mit)",
    )
    args = ap.parse_args()

    rows = load_master(args.master)
    if args.all:
        selected = rows
    else:
        if not args.experiment_id:
            ap.error("provide --experiment-id or --all")
        selected = [r for r in rows if r["experiment_id"] == args.experiment_id]
        if not selected:
            ap.error(f"experiment_id {args.experiment_id!r} not found in {args.master}")

    overlay_cache: dict = {}
    label_yaml = getattr(args, "labelspace_yaml", None)
    if label_yaml is not None:
        overlay_cache["__label_names__"] = sgu.load_label_names(label_yaml)
    else:
        overlay_cache["__label_names__"] = sgu.load_label_names()
    for row in selected:
        process_row(row, args.output_root, args, overlay_cache)
    return 0


if __name__ == "__main__":
    sys.exit(main())
