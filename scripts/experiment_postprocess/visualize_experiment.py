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
    """Load (lat, lon, altitudes, times) filtered to [t_lo, t_hi] for one bag.

    Uses robotdatapy.GPSData for convenience; returns numpy arrays.
    """
    from robotdatapy.data import GPSData

    topic = f"/{robot_ns}/fix"
    bag_dir = bag_file.parent
    try:
        g = GPSData.from_bag(str(bag_dir), topic, time_tol=30.0)
    except Exception as e:
        print(f"    [gps] {bag_dir.name}: {e}", file=sys.stderr)
        return np.array([]), np.zeros((0, 3))
    g.rm_nans()
    mask = (g.times >= t_lo) & (g.times <= t_hi)
    return g.times[mask], g.lat_lon_alts[mask]


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


def plot_trajectory(
    row: dict,
    output_path: Path,
    use_basemap: bool = True,
    t_hi_override: Optional[float] = None,
) -> None:
    import matplotlib.pyplot as plt

    t_lo = float(row["start_time_unix"])
    t_hi = t_hi_override if t_hi_override is not None else float(row["end_time_unix"])
    robots = row_robots(row)

    fig, ax = plt.subplots(figsize=(10, 10))
    colors = ["#e41a1c", "#377eb8", "#4daf4a", "#984ea3"]
    plotted = False
    all_xy = []

    if use_basemap:
        try:
            import contextily as cx
            from pyproj import Transformer

            to_merc = Transformer.from_crs("EPSG:4326", "EPSG:3857", always_xy=True)
        except ImportError:
            print("contextily/pyproj missing; falling back to plain lat/lon.", file=sys.stderr)
            use_basemap = False

    for i, rns in enumerate(robots):
        times, lla = _collect_gps_trajectory(row_bags_for_robot(row, rns), rns, t_lo, t_hi)
        if len(times) < 2:
            print(f"  {rns}: no GPS points in window", file=sys.stderr)
            continue
        lat = lla[:, 0]
        lon = lla[:, 1]
        if use_basemap:
            x, y = to_merc.transform(lon, lat)
        else:
            x, y = lon, lat
        color = colors[i % len(colors)]
        ax.plot(x, y, "-", color=color, linewidth=2.0, label=rns, alpha=0.9)
        ax.scatter(x[0], y[0], marker="o", s=80, color=color, edgecolor="white", zorder=5,
                   label=f"{rns} start")
        ax.scatter(x[-1], y[-1], marker="s", s=80, color=color, edgecolor="white", zorder=5,
                   label=f"{rns} end")
        all_xy.append(np.column_stack([x, y]))
        plotted = True

    if not plotted:
        print("  no GPS data to plot; writing empty placeholder", file=sys.stderr)
        ax.text(0.5, 0.5, "no GPS data in window", transform=ax.transAxes, ha="center")
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
    ax.set_title(f"{row['experiment_id']}\n{end_label}")
    ax.legend(loc="best", fontsize=9)
    ax.grid(True, linestyle=":", alpha=0.5)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"  wrote {output_path}")


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


def process_row(row: dict, out_root: Path, args) -> None:
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
        # Main trajectory uses the stored (selected) end time.
        plot_trajectory(row, out_dir / "trajectory.png", use_basemap=not args.no_basemap)
        # One image per candidate end time.
        for i, t_hi in enumerate(candidates):
            plot_trajectory(
                row,
                out_dir / f"trajectory_cand_{i:02d}.png",
                use_basemap=not args.no_basemap,
                t_hi_override=t_hi,
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

    for row in selected:
        process_row(row, args.output_root, args)
    return 0


if __name__ == "__main__":
    sys.exit(main())
