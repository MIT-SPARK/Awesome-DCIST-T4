#!/usr/bin/env python3
"""Fit a 2-D rigid map->UTM transform from bag GPS fixes and a Hydra trajectory.

Run with the spark_env python (no ROS needed).
"""

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from geo_common import lonlat_to_utm, utm_to_lonlat  # noqa: E402


def find_mcap(p):
    p = Path(p)
    if p.is_dir():
        files = sorted(p.glob("*.mcap"))
        if not files:
            raise SystemExit(f"no .mcap in {p}")
        return files[0]
    return p


def extract_fixes(mcap_path, topic, cache):
    """Return array [t_ns, lat, lon, alt, status] (header stamp)."""
    if cache.exists():
        return np.loadtxt(cache, delimiter=",", skiprows=1, ndmin=2)
    from mcap.reader import make_reader
    from rosbags.typesys import Stores, get_typestore

    ts = get_typestore(Stores.ROS2_JAZZY)
    rows = []
    with open(mcap_path, "rb") as f:
        reader = make_reader(f, validate_crcs=False)
        for _, ch, msg in reader.iter_messages(topics=[topic]):
            m = ts.deserialize_cdr(msg.data, "sensor_msgs/msg/NavSatFix")
            stamp = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
            if stamp == 0:
                stamp = msg.log_time
            rows.append([stamp, m.latitude, m.longitude, m.altitude, m.status.status])
    arr = np.array(rows, dtype=float)
    with open(cache, "w") as f:
        f.write("timestamp_ns,lat,lon,alt,status\n")
        for r in rows:
            f.write(f"{int(r[0])},{r[1]:.9f},{r[2]:.9f},{r[3]:.4f},{int(r[4])}\n")
    return arr


def load_traj(path):
    d = np.loadtxt(path, delimiter=",", skiprows=1)
    return d[np.argsort(d[:, 0])]


def quat_to_R(q):
    w, x, y, z = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def pair_up(fixes, traj, lever, max_dt, min_status, shift_s=0.0):
    """Interpolate trajectory at fix times; return (map_xy of antenna, utm_xy, z)."""
    tt = traj[:, 0]
    mp, up, zs = [], [], []
    for t_ns, lat, lon, alt, st in fixes:
        if st < min_status:
            continue
        t = t_ns + shift_s * 1e9
        i = np.searchsorted(tt, t)
        if i == 0 or i >= len(tt):
            continue
        dt = (tt[i] - tt[i - 1]) / 1e9
        if dt > max_dt and min(t - tt[i - 1], tt[i] - t) / 1e9 > max_dt:
            continue
        a = (t - tt[i - 1]) / (tt[i] - tt[i - 1])
        p = (1 - a) * traj[i - 1, 1:4] + a * traj[i, 1:4]
        q = (1 - a) * traj[i - 1, 4:8] + a * traj[i, 4:8]
        if np.dot(traj[i - 1, 4:8], traj[i, 4:8]) < 0:
            q = (1 - a) * traj[i - 1, 4:8] - a * traj[i, 4:8]
        q /= np.linalg.norm(q)
        ant = p + quat_to_R(q) @ lever
        e, n = lonlat_to_utm(lon, lat)
        mp.append(ant)
        up.append([e, n])
        zs.append(p[2])
    return np.array(mp), np.array(up), np.array(zs)


def umeyama2d(src, dst, with_scale=False):
    mu_s, mu_d = src.mean(0), dst.mean(0)
    s, d = src - mu_s, dst - mu_d
    H = s.T @ d / len(src)
    U, S, Vt = np.linalg.svd(H)
    D = np.diag([1, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    sc = (S * np.diag(D)).sum() / (s**2).sum(1).mean() if with_scale else 1.0
    t = mu_d - sc * R @ mu_s
    return R, t, sc


def residuals(R, t, sc, src, dst):
    return np.linalg.norm((sc * (R @ src.T)).T + t - dst, axis=1)


def fit(mp, up, reject=True):
    R, t, sc = umeyama2d(mp[:, :2], up)
    r = residuals(R, t, sc, mp[:, :2], up)
    keep = np.ones(len(r), bool)
    if reject:
        keep = r <= max(3 * np.median(r), 0.5)
        R, t, sc = umeyama2d(mp[keep, :2], up[keep])
        r = residuals(R, t, sc, mp[keep, :2], up[keep])
    return R, t, r, keep


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--bag", required=True)
    ap.add_argument("--topic", default="/euclid/fix")
    ap.add_argument("--trajectory", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--lever-arm", type=float, nargs=3, default=[-0.22, 0.0, 0.54])
    ap.add_argument("--max-dt", type=float, default=0.2)
    ap.add_argument("--epsg", type=int, default=32618)
    ap.add_argument(
        "--min-status",
        type=int,
        default=2,
        help="lowest NavSatFix status used (2 = RTK-corrected; plain fixes, 0, are metres off)",
    )
    ap.add_argument(
        "--body-height",
        type=float,
        default=0.5,
        help="trajectory z minus this = ground z in map frame",
    )
    ap.add_argument(
        "--time-search",
        type=float,
        default=2.0,
        help="+- seconds of time shift searched as a diagnostic",
    )
    ap.add_argument("--no-plot", action="store_true")
    a = ap.parse_args()
    if a.epsg != 32618:
        print("note: geo_common default EPSG is 32618; conversions use that")
    out = Path(a.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    mcap = find_mcap(a.bag)
    cache = out.with_name(out.stem + "_fixes.csv")
    t0 = time.time()
    fixes = extract_fixes(mcap, a.topic, cache)
    print(f"fixes: {len(fixes)} ({time.time() - t0:.1f}s, cache {cache})")
    traj = load_traj(a.trajectory)
    lever = np.array(a.lever_arm)
    print(
        f"trajectory: {len(traj)} poses, span {(traj[-1, 0] - traj[0, 0]) / 1e9:.0f}s; "
        f"fix span {(fixes[-1, 0] - fixes[0, 0]) / 1e9:.0f}s; "
        f"overlap {max(traj[0, 0], fixes[0, 0]) / 1e9:.0f}..{min(traj[-1, 0], fixes[-1, 0]) / 1e9:.0f}"
    )
    print(
        "status counts:",
        {int(s): int((fixes[:, 4] == s).sum()) for s in np.unique(fixes[:, 4])},
    )

    mp, up, zs = pair_up(fixes, traj, lever, a.max_dt, a.min_status)
    if len(mp) < 3:
        raise SystemExit(f"only {len(mp)} pairs")
    R, t, r, keep = fit(mp, up)
    yaw = float(np.arctan2(R[1, 0], R[0, 0]))
    # diagnostics
    _, _, sc = umeyama2d(mp[keep, :2], up[keep], with_scale=True)
    Rs, ts, scs = umeyama2d(mp[keep, :2], up[keep], with_scale=True)
    rs = residuals(Rs, ts, scs, mp[keep, :2], up[keep])
    print(f"pairs {len(mp)} -> kept {int(keep.sum())}")
    print(
        f"rigid: yaw {np.degrees(yaw):.3f} deg, t {t}, RMS {np.sqrt((r**2).mean()):.3f} "
        f"median {np.median(r):.3f} max {r.max():.3f}"
    )
    print(f"with-scale diag: scale {scs:.5f} RMS {np.sqrt((rs**2).mean()):.3f}")
    best = (0.0, np.sqrt((r**2).mean()))
    for sh in np.arange(-a.time_search, a.time_search + 1e-9, 0.25):
        m2, u2, _ = pair_up(fixes, traj, lever, a.max_dt, a.min_status, sh)
        if len(m2) < 3:
            continue
        _, _, r2, _ = fit(m2, u2)
        rms2 = np.sqrt((r2**2).mean())
        if rms2 < best[1] - 1e-9:
            best = (float(sh), float(rms2))
    print(f"time-shift search best: shift {best[0]:+.2f}s RMS {best[1]:.3f}")

    z0 = float(np.median(zs[keep]) - a.body_height)
    c_map = mp[keep, :2].mean(0)
    cu = R @ c_map + t
    lon_c, lat_c = utm_to_lonlat(cu[0], cu[1])
    origin = t
    res = {
        "epsg": a.epsg,
        "yaw": yaw,
        "t": [float(t[0]), float(t[1])],
        "z0": z0,
        "z0_note": f"median trajectory z minus {a.body_height} m (Spot body height above ground)",
        "rms": float(np.sqrt((r**2).mean())),
        "median_residual": float(np.median(r)),
        "max_residual": float(r.max()),
        "n": int(keep.sum()),
        "scale_diagnostic": float(scs),
        "time_shift_best": best[0],
        "source": {
            "bag": str(mcap),
            "topic": a.topic,
            "trajectory": str(a.trajectory),
            "lever_arm": list(a.lever_arm),
            "max_dt": a.max_dt,
            "time": time.strftime("%Y-%m-%dT%H:%M:%S"),
        },
        "lonlat_center": [float(lon_c), float(lat_c)],
    }
    out.write_text(json.dumps(res, indent=2))
    print(f"map origin -> UTM ({origin[0]:.2f}, {origin[1]:.2f}); wrote {out}")

    if not a.no_plot:
        plot(out.with_suffix(".png"), R, t, traj, up, a.epsg)


def plot(path, R, t, traj, up, epsg):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    tu = (R @ traj[:, 1:3].T).T + t
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.plot(up[:, 0], up[:, 1], "b.", ms=3, label="GPS")
    ax.plot(tu[:, 0], tu[:, 1], "r-", lw=1, label="Hydra trajectory (anchored)")
    ax.set_aspect("equal")
    ax.legend()
    try:
        import contextily as cx

        cx.add_basemap(ax, crs=f"EPSG:{epsg}", source=cx.providers.Esri.WorldImagery)
    except Exception as e:
        print("basemap unavailable:", e)
    fig.savefig(path, dpi=110, bbox_inches="tight")
    print("wrote", path)


if __name__ == "__main__":
    main()
