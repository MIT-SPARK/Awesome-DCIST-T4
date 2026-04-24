#!/usr/bin/env python3
"""Compare GPS quality and radio usage between sessions.

Generates a multi-panel figure with rolling-window statistics and summary
tables comparing GPS fix rate, position uncertainty, TX/RX throughput,
and RTCM correction stream rate across two or more sessions.

Usage:
    python compare_gps_radio.py --output /path/to/output.png

Run with ROS 2 Jazzy workspace and spark_env sourced.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.table import Table

_THIS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(_THIS_DIR))
import _bag_utils as bu


# ---- configuration -------------------------------------------------------

# Sessions to compare: label -> (bag file path, robot namespace)
SESSIONS = {
    "Apr 16 hamilton": ("/data/dcist/west_point_2026/april_16_alpha_hamilton/recorded_data/recorded_data_0.mcap", "hamilton"),
    "Apr 17 hamilton": ("/data/dcist/west_point_2026/april_17_alpha_hamilton/recorded_data/recorded_data_0.mcap", "hamilton"),
    "Apr 16 euclid":   ("/data/dcist/west_point_2026/april_16_alpha_euclid/recorded_data/recorded_data_0.mcap",   "euclid"),
    "Apr 17 euclid":   ("/data/dcist/west_point_2026/april_17_alpha_euclid/recorded_data/recorded_data_0.mcap",   "euclid"),
}

WINDOW_MIN = 2.0       # rolling window width in minutes
COLORS = ["#377eb8", "#e41a1c", "#4daf4a", "#984ea3"]


# ---- data loading --------------------------------------------------------

def load_session(bag_path: str, robot: str) -> dict:
    from mcap.reader import make_reader
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    NavSatFix = get_message("sensor_msgs/msg/NavSatFix")
    String    = get_message("std_msgs/msg/String")

    topics = [f"/{robot}/fix", f"/{robot}/tx_rx_usage", f"/{robot}/rtcm"]

    d: dict = {
        "fix_t": [], "fix_status": [], "fix_cov": [],
        "txrx_t": [], "tx_rate": [], "rx_rate": [],
        "rtcm_t": [],
    }

    with open(bag_path, "rb") as f:
        reader = make_reader(f, validate_crcs=False)
        for schema, channel, msg in reader.iter_messages(topics=topics):
            t = msg.log_time * 1e-9
            topic = channel.topic

            if topic == f"/{robot}/fix":
                m = deserialize_message(msg.data, NavSatFix)
                d["fix_t"].append(t)
                d["fix_status"].append(m.status.status)
                cov = np.array(m.position_covariance).reshape(3, 3)
                # Horizontal position sigma: max of x,y std devs
                d["fix_cov"].append(np.sqrt(max(cov[0, 0], cov[1, 1])))

            elif topic == f"/{robot}/tx_rx_usage":
                m = deserialize_message(msg.data, String)
                try:
                    j = json.loads(m.data)
                    dt = j["delta_t"]
                    if dt > 0:
                        d["txrx_t"].append(t)
                        d["tx_rate"].append(j["tx_bytes"] / dt / 1e6)  # MB/s
                        d["rx_rate"].append(j["rx_bytes"] / dt / 1e6)
                except Exception:
                    pass

            elif topic == f"/{robot}/rtcm":
                d["rtcm_t"].append(t)

    # Normalise timestamps to minutes since session start
    all_t = d["fix_t"] + d["txrx_t"] + d["rtcm_t"]
    t0 = min(all_t) if all_t else 0.0
    for k in ("fix_t", "txrx_t", "rtcm_t"):
        d[k] = np.array([(x - t0) / 60.0 for x in d[k]])
    for k in ("fix_status", "fix_cov", "tx_rate", "rx_rate"):
        d[k] = np.array(d[k])

    return d


# ---- rolling statistics --------------------------------------------------

def rolling_frac(t: np.ndarray, vals: np.ndarray, match_val, window: float,
                 n_pts: int = 500) -> tuple[np.ndarray, np.ndarray]:
    """Fraction of vals == match_val in a sliding window."""
    idx = np.round(np.linspace(0, len(t) - 1, min(n_pts, len(t)))).astype(int)
    out_t, out_v = [], []
    for i in idx:
        ti = t[i]
        mask = (t >= ti - window / 2) & (t <= ti + window / 2)
        if mask.sum() > 0:
            out_t.append(ti)
            out_v.append(np.mean(vals[mask] == match_val))
    return np.array(out_t), np.array(out_v)


def rolling_mean(t: np.ndarray, vals: np.ndarray, window: float,
                 n_pts: int = 500) -> tuple[np.ndarray, np.ndarray]:
    idx = np.round(np.linspace(0, len(t) - 1, min(n_pts, len(t)))).astype(int)
    out_t, out_v = [], []
    for i in idx:
        ti = t[i]
        mask = (t >= ti - window / 2) & (t <= ti + window / 2)
        if mask.sum() > 0:
            out_t.append(ti)
            out_v.append(np.mean(vals[mask]))
    return np.array(out_t), np.array(out_v)


def rolling_rate(t: np.ndarray, window: float,
                 n_pts: int = 500) -> tuple[np.ndarray, np.ndarray]:
    """Events per second in a sliding window."""
    if len(t) == 0:
        return np.array([]), np.array([])
    idx = np.round(np.linspace(0, len(t) - 1, min(n_pts, len(t)))).astype(int)
    out_t, out_v = [], []
    for i in idx:
        ti = t[i]
        mask = (t >= ti - window / 2) & (t <= ti + window / 2)
        n = mask.sum()
        if n > 0:
            out_t.append(ti)
            out_v.append(n / (window * 60.0))  # window is in minutes
    return np.array(out_t), np.array(out_v)


# ---- summary statistics --------------------------------------------------

def summary_stats(d: dict) -> dict:
    status = d["fix_status"]
    cov    = d["fix_cov"]
    tx     = d["tx_rate"]
    rx     = d["rx_rate"]

    valid_mask = status >= 0
    valid_cov  = cov[valid_mask] if valid_mask.any() else np.array([np.nan])

    rtcm_gaps = np.diff(d["rtcm_t"]) * 60.0  # seconds between RTCM msgs
    max_rtcm_gap = rtcm_gaps.max() if len(rtcm_gaps) else float("nan")

    return {
        "Duration (min)":         f"{d['fix_t'][-1]:.1f}" if len(d["fix_t"]) else "?",
        "GPS messages":           str(len(status)),
        "NO_FIX (%)":             f"{100 * np.mean(status < 0):.1f}",
        "Fix rate (%)":           f"{100 * np.mean(status == 0):.1f}",
        "Pos σ median (m)":       f"{np.nanmedian(valid_cov):.2f}",
        "Pos σ 95th pct (m)":     f"{np.nanpercentile(valid_cov, 95):.2f}",
        "TX mean (MB/s)":         f"{np.mean(tx):.3f}" if len(tx) else "?",
        "RX mean (MB/s)":         f"{np.mean(rx):.3f}" if len(rx) else "?",
        "RX peak (MB/s)":         f"{np.max(rx):.3f}" if len(rx) else "?",
        "RTCM msgs":              str(len(d["rtcm_t"])),
        "RTCM rate (msgs/s)":     f"{len(d['rtcm_t']) / (d['rtcm_t'][-1] * 60):.2f}" if len(d["rtcm_t"]) > 1 else "?",
        "Max RTCM gap (s)":       f"{max_rtcm_gap:.1f}",
    }


# ---- plotting ------------------------------------------------------------

def add_summary_table(ax, labels: list[str], stats_list: list[dict], colors: list[str]):
    """Replace axis content with a colour-coded summary table."""
    ax.axis("off")
    keys = list(stats_list[0].keys())
    col_labels = ["Metric"] + labels
    cell_text = [[k] + [s[k] for s in stats_list] for k in keys]

    tbl = ax.table(
        cellText=cell_text,
        colLabels=col_labels,
        loc="center",
        cellLoc="center",
    )
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(9)
    tbl.scale(1.0, 1.4)

    # Header row styling
    for j, col in enumerate(col_labels):
        cell = tbl[(0, j)]
        cell.set_facecolor("#dddddd")
        cell.set_text_props(fontweight="bold")

    # Colour-tint data columns
    for j, color in enumerate(colors[:len(labels)]):
        import matplotlib.colors as mcolors
        light = mcolors.to_rgba(color, alpha=0.15)
        for i in range(1, len(keys) + 1):
            tbl[(i, j + 1)].set_facecolor(light)

    # Alternate row shading on metric column
    for i in range(1, len(keys) + 1):
        if i % 2 == 0:
            tbl[(i, 0)].set_facecolor("#f5f5f5")


def make_plot(session_data: dict[str, dict], output_path: Path) -> None:
    labels = list(session_data.keys())
    palette = COLORS[:len(labels)]

    fig = plt.figure(figsize=(15, 18))
    gs = gridspec.GridSpec(5, 1, figure=fig, hspace=0.45,
                           height_ratios=[2, 2, 2, 2, 2.5])

    axes = [fig.add_subplot(gs[i]) for i in range(5)]
    fig.suptitle(f"GPS quality & radio comparison\n({', '.join(labels)})",
                 fontsize=13, fontweight="bold")

    for label, d, color in zip(labels, session_data.values(), palette):
        # Panel 0: NO_FIX fraction
        t, v = rolling_frac(d["fix_t"], d["fix_status"], -1, WINDOW_MIN)
        axes[0].plot(t, v * 100, color=color, label=label, linewidth=1.5)

        # Panel 1: GPS position sigma
        valid = d["fix_status"] >= 0
        if valid.any():
            t, v = rolling_mean(d["fix_t"][valid], d["fix_cov"][valid], WINDOW_MIN)
            axes[1].plot(t, v, color=color, label=label, linewidth=1.5)

        # Panel 2: TX / RX rates
        axes[2].plot(d["txrx_t"], d["tx_rate"], color=color,
                     label=f"{label} TX", linewidth=1.0, alpha=0.85)
        axes[2].plot(d["txrx_t"], d["rx_rate"], color=color,
                     label=f"{label} RX", linewidth=1.0, alpha=0.45, linestyle="--")

        # Panel 3: RTCM correction rate
        t, v = rolling_rate(d["rtcm_t"], WINDOW_MIN)
        axes[3].plot(t, v, color=color, label=label, linewidth=1.5)

    axes[0].set_ylabel("NO_FIX (%)")
    axes[0].set_title(f"GPS NO_FIX rate  ({WINDOW_MIN}-min rolling window)")
    axes[0].set_ylim(0, 105)
    axes[0].legend(fontsize=9); axes[0].grid(True, alpha=0.3)

    axes[1].set_ylabel("σ_horiz (m)")
    axes[1].set_title("GPS horizontal position uncertainty (valid fixes only)")
    axes[1].legend(fontsize=9); axes[1].grid(True, alpha=0.3)

    axes[2].set_ylabel("MB/s")
    axes[2].set_title("Radio throughput — TX (solid) / RX (dashed)")
    axes[2].legend(ncol=2, fontsize=8); axes[2].grid(True, alpha=0.3)

    axes[3].set_ylabel("RTCM msgs/s")
    axes[3].set_title("RTCM correction stream rate")
    axes[3].legend(fontsize=9); axes[3].grid(True, alpha=0.3)

    for ax in axes[:4]:
        ax.set_xlabel("minutes since session start")

    # Panel 4: summary table
    stats_list = [summary_stats(d) for d in session_data.values()]
    add_summary_table(axes[4], labels, stats_list, palette)
    axes[4].set_title("Summary statistics", fontweight="bold", pad=8)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved → {output_path}")


# ---- main ----------------------------------------------------------------

def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--output", type=Path,
        default=Path("/home/harel/data/west_point_2026/viz_out/gps_radio_comparison.png"),
        help="Output PNG path",
    )
    ap.add_argument("--window", type=float, default=WINDOW_MIN,
                    help="Rolling window width in minutes")
    args = ap.parse_args()

    session_data = {}
    for label, (bag_path, robot) in SESSIONS.items():
        print(f"Loading {label} ...", flush=True)
        session_data[label] = load_session(bag_path, robot)
        d = session_data[label]
        print(f"  fix={len(d['fix_t'])}  txrx={len(d['txrx_t'])}  rtcm={len(d['rtcm_t'])}")

    print("Generating plot...", flush=True)
    make_plot(session_data, args.output)
    return 0


if __name__ == "__main__":
    sys.exit(main())
