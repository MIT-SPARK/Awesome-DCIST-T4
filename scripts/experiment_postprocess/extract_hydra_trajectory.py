#!/usr/bin/env python3
"""Extract Hydra-Multi optimized trajectory from TF and anchor it to GPS.

For each configured bag:
  1. Read `map -> <robot>/odom` from /tf at ~10 Hz (Hydra-Multi loop-closure output).
  2. Read high-rate odom poses (`<robot>/odom` topic) and compose with the TF
     to get the robot body position in the map frame.
  3. Use valid GPS fixes (status >= 0) to fit a 2-D rigid transform
     (rotation + translation) from the map frame to UTM, anchoring the
     trajectory globally.
  4. Save a CSV in the same KIMERA_MULTI_GT_CSV_OPTIONS format as the fused/odom
     CSVs (positions in UTM easting/northing, zone 18N).

Usage:
    python extract_hydra_trajectory.py [--output-dir DIR] [--fix-dir DIR]

Then re-run visualize_experiment.py with --fused-dir pointing at the output dir
to see the Hydra trajectory overlaid on the satellite basemap.

Run with ROS 2 Jazzy workspace and spark_env sourced.
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as Rot
from scipy.spatial.transform import Slerp

_THIS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(_THIS_DIR))
import _bag_utils as bu

# ---- configuration -------------------------------------------------------

# Sessions to process: session_folder -> robot namespace
SESSIONS = {
    "april_17_alpha_hamilton": "hamilton",
    "april_17_alpha_euclid":   "euclid",
    "april_16_alpha_hamilton": "hamilton",
    "april_16_alpha_euclid":   "euclid",
    "april_15_bravo_hamilton": "hamilton",
    "april_15_bravo_euclid":   "euclid",
}

DATA_ROOT  = Path("/data/dcist/west_point_2026")
OUTPUT_DIR = Path("/home/harel/data/west_point_2026/processed/hydra")
FIX_DIR    = Path("/home/harel/data/west_point_2026/processed/fix")

# UTM zone for West Point, NY
UTM_EPSG = "EPSG:32618"   # zone 18N

# Minimum valid GPS fixes needed to anchor the map frame
MIN_GPS_ANCHORS = 10
MAX_GPS_SIGMA_M = 5.0   # reject fixes with reported horizontal sigma above this


# ---- TF reading ----------------------------------------------------------

def read_map_T_odom(bag_file: Path, robot: str) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Read `map -> <robot>/odom` transforms from /tf.

    Returns (times_s, translations (N,3), quaternions (N,4) xyzw).
    """
    from mcap.reader import make_reader
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    TFMsg = get_message("tf2_msgs/msg/TFMessage")
    child = f"{robot}/odom"

    times, trans, quats = [], [], []
    with open(bag_file, "rb") as f:
        reader = make_reader(f, validate_crcs=False)
        for _, channel, msg in reader.iter_messages(topics=["/tf"]):
            m = deserialize_message(msg.data, TFMsg)
            for tf in m.transforms:
                if tf.child_frame_id != child:
                    continue
                t = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
                tr = tf.transform.translation
                ro = tf.transform.rotation
                times.append(t)
                trans.append([tr.x, tr.y, tr.z])
                quats.append([ro.x, ro.y, ro.z, ro.w])

    if not times:
        return np.array([]), np.zeros((0, 3)), np.zeros((0, 4))

    times = np.array(times)
    order = np.argsort(times)
    return times[order], np.array(trans)[order], np.array(quats)[order]


def read_odom_poses(bag_file: Path, robot: str) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Read `<robot>/odom` topic (PoseStamped/Odometry) — robot in odom frame.

    Returns (times_s, positions (N,3), quaternions (N,4) xyzw).
    """
    from mcap.reader import make_reader
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    OdomMsg = get_message("nav_msgs/msg/Odometry")
    topic = f"/{robot}/odom"

    times, positions, quats = [], [], []
    with open(bag_file, "rb") as f:
        reader = make_reader(f, validate_crcs=False)
        for _, channel, msg in reader.iter_messages(topics=[topic]):
            m = deserialize_message(msg.data, OdomMsg)
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            p = m.pose.pose.position
            o = m.pose.pose.orientation
            times.append(t)
            positions.append([p.x, p.y, p.z])
            quats.append([o.x, o.y, o.z, o.w])

    if not times:
        return np.array([]), np.zeros((0, 3)), np.zeros((0, 4))

    times = np.array(times)
    order = np.argsort(times)
    return times[order], np.array(positions)[order], np.array(quats)[order]


# ---- pose interpolation --------------------------------------------------

def interp_poses(
    query_times: np.ndarray,
    ref_times: np.ndarray,
    ref_trans: np.ndarray,
    ref_quats: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Linear-interpolate translations, slerp quaternions at query_times."""
    trans = np.column_stack([
        np.interp(query_times, ref_times, ref_trans[:, i]) for i in range(3)
    ])
    slerp = Slerp(ref_times, Rot.from_quat(ref_quats))
    # Clamp to valid range
    clamped = np.clip(query_times, ref_times[0], ref_times[-1])
    quats = slerp(clamped).as_quat()
    return trans, quats


def compose_poses(
    t_map_odom: np.ndarray, q_map_odom: np.ndarray,
    t_odom_body: np.ndarray, q_odom_body: np.ndarray,
) -> np.ndarray:
    """Return body positions in map frame: p_map = R_map_odom @ p_odom_body + t_map_odom."""
    R = Rot.from_quat(q_map_odom).as_matrix()   # (N, 3, 3)
    p = np.einsum("nij,nj->ni", R, t_odom_body) + t_map_odom
    return p


# ---- GPS anchoring -------------------------------------------------------

def load_valid_gps_utm(fix_csv: Path,
                       max_sigma_m: float = MAX_GPS_SIGMA_M
                       ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Load high-quality GPS fixes from fix CSV, return (times_s, easting, northing).

    Keeps only fixes with status>=0 AND horizontal sigma <= max_sigma_m.
    """
    from pyproj import Transformer
    to_utm = Transformer.from_crs("EPSG:4326", UTM_EPSG, always_xy=True)

    times, east, north = [], [], []
    total = rejected_status = rejected_cov = 0
    with open(fix_csv, newline="") as f:
        for row in csv.DictReader(f):
            total += 1
            if int(row["status"]) < 0:
                rejected_status += 1
                continue
            sigma = np.sqrt(max(float(row["cov_xx"]), float(row["cov_yy"])))
            if sigma > max_sigma_m:
                rejected_cov += 1
                continue
            lat, lon = float(row["latitude"]), float(row["longitude"])
            e, n = to_utm.transform(lon, lat)
            times.append(int(row["timestamp_ns"]) * 1e-9)
            east.append(e)
            north.append(n)

    print(f"    GPS: {total} total, {rejected_status} NO_FIX, "
          f"{rejected_cov} high-cov (>{max_sigma_m}m), {len(times)} kept")
    return np.array(times), np.array(east), np.array(north)


def fit_map_to_utm(
    map_xy: np.ndarray, utm_xy: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """Fit 2-D rigid transform (R, t) such that utm ≈ R @ map + t.

    Uses SVD (Arun's method) on the XY plane.
    Returns (R_2x2, t_2d).
    """
    mu_m = map_xy.mean(axis=0)
    mu_u = utm_xy.mean(axis=0)
    H = (map_xy - mu_m).T @ (utm_xy - mu_u)
    U, _, Vt = np.linalg.svd(H)
    R2 = Vt.T @ U.T
    if np.linalg.det(R2) < 0:
        Vt[-1, :] *= -1
        R2 = Vt.T @ U.T
    t2 = mu_u - R2 @ mu_m
    return R2, t2


def fit_map_to_utm_ransac(
    map_xy: np.ndarray, utm_xy: np.ndarray,
    inlier_threshold_m: float = 10.0, iterations: int = 200
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """RANSAC robust fit of map→UTM 2-D rigid transform.

    Returns (R_2x2, t_2d, inlier_mask).
    """
    rng = np.random.default_rng(42)
    n = len(map_xy)
    best_mask = np.zeros(n, dtype=bool)

    for _ in range(iterations):
        idx = rng.choice(n, size=min(10, n), replace=False)
        R2, t2 = fit_map_to_utm(map_xy[idx], utm_xy[idx])
        pred = (R2 @ map_xy.T).T + t2
        resid = np.linalg.norm(pred - utm_xy, axis=1)
        mask = resid < inlier_threshold_m
        if mask.sum() > best_mask.sum():
            best_mask = mask

    # Final fit on all inliers
    if best_mask.sum() < 3:
        best_mask = np.ones(n, dtype=bool)
    R2, t2 = fit_map_to_utm(map_xy[best_mask], utm_xy[best_mask])
    return R2, t2, best_mask


# ---- CSV output ----------------------------------------------------------

CSV_HEADER = ["#timestamp_kf", "x", "y", "z", "qw", "qx", "qy", "qz"]


def save_csv(path: Path, times_s: np.ndarray, positions: np.ndarray,
             quats_xyzw: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    times_ns = (times_s * 1e9).astype(np.int64)
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(CSV_HEADER)
        for t, p, q in zip(times_ns, positions, quats_xyzw):
            writer.writerow([t, p[0], p[1], p[2], q[3], q[0], q[1], q[2]])
    print(f"    saved {len(times_s)} poses → {path.name}")


# ---- per-bag processing --------------------------------------------------

def process_bag(bag_file: Path, robot: str, fix_csv: Path, out_path: Path,
                anchor_fix_csv: Path | None = None) -> bool:
    """anchor_fix_csv: use a different robot's fix CSV for GPS anchoring (cross-robot).
    Both robots must share the same Hydra map frame."""
    print(f"  Reading map->odom TF ...", flush=True)
    tf_t, tf_trans, tf_quats = read_map_T_odom(bag_file, robot)
    if len(tf_t) == 0:
        print(f"  [skip] no map->{robot}/odom TF found", file=sys.stderr)
        return False
    print(f"    {len(tf_t)} TF poses at ~{len(tf_t)/(tf_t[-1]-tf_t[0]):.1f} Hz")

    print(f"  Reading odom ...", flush=True)
    od_t, od_pos, od_quat = read_odom_poses(bag_file, robot)
    if len(od_t) == 0:
        print(f"  [skip] no odom found", file=sys.stderr)
        return False
    print(f"    {len(od_t)} odom poses")

    # Clip to TF time range (Hydra may start after bag start)
    mask = (od_t >= tf_t[0]) & (od_t <= tf_t[-1])
    od_t, od_pos, od_quat = od_t[mask], od_pos[mask], od_quat[mask]
    if len(od_t) == 0:
        print("  [skip] no odom in TF time range", file=sys.stderr)
        return False

    # Subsample odom to a manageable rate (~10 Hz to match TF)
    odom_hz = len(od_t) / (od_t[-1] - od_t[0])
    stride = max(1, int(odom_hz / 10))
    od_t, od_pos, od_quat = od_t[::stride], od_pos[::stride], od_quat[::stride]
    print(f"    subsampled to {len(od_t)} odom poses (stride {stride})")

    # Interpolate map_T_odom at each odom timestamp
    tf_trans_at_odom, tf_quats_at_odom = interp_poses(od_t, tf_t, tf_trans, tf_quats)

    # Compose: map_T_body
    map_positions = compose_poses(tf_trans_at_odom, tf_quats_at_odom, od_pos, od_quat)

    # Choose which fix CSV to use for GPS anchoring
    gps_fix_csv = anchor_fix_csv if (anchor_fix_csv and anchor_fix_csv.exists()) else fix_csv
    if anchor_fix_csv and anchor_fix_csv.exists():
        print(f"  Using cross-robot GPS anchor: {anchor_fix_csv.name}")
        # For cross-robot: read the anchor robot's map-frame trajectory too
        anchor_robot = anchor_fix_csv.stem.split("__")[0].split("_")[-1]  # e.g. "euclid"
        anchor_tf_t, anchor_tf_trans, anchor_tf_quats = read_map_T_odom(bag_file, anchor_robot)
        if len(anchor_tf_t) == 0:
            print(f"  [warn] no map->{anchor_robot}/odom TF in this bag; falling back to own GPS")
            gps_fix_csv = fix_csv
            anchor_od_t = od_t
            anchor_map_positions = map_positions
        else:
            anchor_od_t2, anchor_od_pos, anchor_od_quat = read_odom_poses(bag_file, anchor_robot)
            if len(anchor_od_t2) > 0:
                mask2 = (anchor_od_t2 >= anchor_tf_t[0]) & (anchor_od_t2 <= anchor_tf_t[-1])
                anchor_od_t2 = anchor_od_t2[mask2]
                anchor_od_pos = anchor_od_pos[mask2]
                anchor_od_quat = anchor_od_quat[mask2]
                stride2 = max(1, int(len(anchor_od_t2) / (anchor_tf_t[-1] - anchor_tf_t[0]) / 10))
                anchor_od_t2 = anchor_od_t2[::stride2]
                anchor_od_pos = anchor_od_pos[::stride2]
                anchor_od_quat = anchor_od_quat[::stride2]
                tf_at_a, tfq_at_a = interp_poses(anchor_od_t2, anchor_tf_t, anchor_tf_trans, anchor_tf_quats)
                anchor_map_positions = compose_poses(tf_at_a, tfq_at_a, anchor_od_pos, anchor_od_quat)
                anchor_od_t = anchor_od_t2
            else:
                anchor_od_t = od_t
                anchor_map_positions = map_positions
    else:
        anchor_od_t = od_t
        anchor_map_positions = map_positions

    if not gps_fix_csv.exists():
        print(f"  [warn] no fix CSV at {gps_fix_csv}; saving map-frame coords (not UTM)", file=sys.stderr)
        save_csv(out_path, od_t, map_positions, od_quat)
        return True

    gps_t, gps_east, gps_north = load_valid_gps_utm(gps_fix_csv)

    if len(gps_t) < MIN_GPS_ANCHORS:
        print(f"  [warn] only {len(gps_t)} quality GPS fixes (<{MIN_GPS_ANCHORS}); "
              f"saving map-frame coords (not UTM)", file=sys.stderr)
        save_csv(out_path, od_t, map_positions, od_quat)
        return True

    # Find anchor robot's map-frame positions at GPS times
    t_lo = max(anchor_od_t[0], gps_t[0])
    t_hi = min(anchor_od_t[-1], gps_t[-1])
    gps_mask = (gps_t >= t_lo) & (gps_t <= t_hi)
    gps_t = gps_t[gps_mask]
    gps_east = gps_east[gps_mask]
    gps_north = gps_north[gps_mask]

    map_at_gps = np.column_stack([
        np.interp(gps_t, anchor_od_t, anchor_map_positions[:, i]) for i in range(3)
    ])

    R2, t2, inliers = fit_map_to_utm_ransac(
        map_at_gps[:, :2], np.column_stack([gps_east, gps_north])
    )
    pred = (R2 @ map_at_gps[:, :2].T).T + t2
    resid = np.linalg.norm(pred - np.column_stack([gps_east, gps_north]), axis=1)
    print(f"  GPS alignment: {inliers.sum()}/{len(inliers)} inliers  "
          f"residual mean={resid[inliers].mean():.2f}m  max={resid[inliers].max():.2f}m")

    # Apply transform to robot's map-frame trajectory → UTM
    utm_xy = (R2 @ map_positions[:, :2].T).T + t2
    utm_positions = np.column_stack([utm_xy, map_positions[:, 2]])

    save_csv(out_path, od_t, utm_positions, od_quat)
    return True


# ---- main ----------------------------------------------------------------

def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--output-dir", type=Path, default=OUTPUT_DIR)
    ap.add_argument("--fix-dir", type=Path, default=FIX_DIR)
    ap.add_argument("--data-root", type=Path, default=DATA_ROOT)
    ap.add_argument("--session", type=str, default=None,
                    help="Process only this session (default: all in SESSIONS)")
    ap.add_argument("--anchor-robot", type=str, default=None,
                    help="Use this robot's GPS fixes for map anchoring instead of the "
                         "bag's own robot (e.g. 'euclid' when hamilton GPS is bad)")
    ap.add_argument("--force", action="store_true", help="Overwrite existing CSVs")
    args = ap.parse_args()

    sessions = SESSIONS
    if args.session:
        if args.session not in SESSIONS:
            ap.error(f"{args.session!r} not in SESSIONS")
        sessions = {args.session: SESSIONS[args.session]}

    for session, robot in sessions.items():
        session_dir = args.data_root / session
        for bag_dir in sorted(session_dir.glob("recorded_data*")):
            if not bag_dir.is_dir():
                continue
            try:
                bag_file = bu.find_mcap(bag_dir)
            except RuntimeError as e:
                print(f"  [skip] {e}", file=sys.stderr)
                continue

            out_path = args.output_dir / f"{session}__{bag_dir.name}.csv"
            if out_path.exists() and not args.force:
                print(f"skip (exists): {out_path.name}")
                continue

            fix_csv = args.fix_dir / f"{session}__{bag_dir.name}__fix.csv"

            # Cross-robot anchor: look for the anchor robot's fix CSV in the same session
            anchor_fix_csv = None
            if args.anchor_robot and args.anchor_robot != robot:
                # Find the sibling session folder for the anchor robot
                session_base = "_".join(session.split("_")[:-1])  # strip last token (robot name)
                anchor_session = f"{session_base}_{args.anchor_robot}"
                anchor_fix_csv = args.fix_dir / f"{anchor_session}__{bag_dir.name}__fix.csv"
                if not anchor_fix_csv.exists():
                    print(f"  [warn] anchor fix CSV not found: {anchor_fix_csv.name}")
                    anchor_fix_csv = None

            print(f"\n=== {session} / {bag_dir.name} ===")
            process_bag(bag_file, robot, fix_csv, out_path, anchor_fix_csv=anchor_fix_csv)

    return 0


if __name__ == "__main__":
    sys.exit(main())
