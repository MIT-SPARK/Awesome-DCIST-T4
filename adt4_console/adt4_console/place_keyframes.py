"""Which keyframes observed a traversability ("mesh") place (API.md §7, place keyframes).

The visibility test is the one ``scripts/gps_fusion/regions/place_features.py`` uses to
give places their region features: the place centre plus 8 points at 0.6 of its
region-growing boundary radius are projected into the keyframe (world_T_body from the
graph's agent node the frame is anchored to, then body_T_sensor). A point counts as seen
when it is in front of the camera (z > 0.3 m), within 12 m, inside the image, and not
occluded: the median of the valid depth pixels in a 5x5 window around it is at least
``z - max(0.5, 0.1 z)``; a point without valid depth is accepted only within 5 m. A
keyframe's weight is its fraction of seen points (``visible``).

:class:`PlaceKeyframeIndex` holds every keyframe of a map version with its camera pose
and answers per place. The full place x keyframe table (one depth decode per keyframe,
Penn: ~4000 x ~4000 in tens of seconds) is built once in a background thread and kept on
disk, keyed by the content (places, keyframe poses, parameters), so a console restart
reuses it. Until it is ready a query tests that one place against every keyframe:
frustum + range for all of them at once, then the depth test on the shortlist only.
"""

from __future__ import annotations

import datetime
import hashlib
import logging
import math
import pathlib
import threading
import time
from collections import Counter, OrderedDict
from dataclasses import dataclass

import numpy as np

log = logging.getLogger(__name__)

MESH_PLACES = (3, 1)
MAX_RANGE = 12.0  # m, camera to point
MAX_RANGE_NO_DEPTH = 5.0  # m, points without valid depth
DEPTH_TOL, DEPTH_TOL_REL = 0.5, 0.1  # occluded when depth < z - max(tol, rel * z)
MIN_Z = 0.3
N_RING, RING_FRAC = 8, 0.6  # boundary samples: count, fraction of the ray length
CAND_PAD = (
    5.0  # places whose centre is within MAX_RANGE + this of the camera are tested
)
WIN = 2  # depth window half size (5x5)
DEPTH_CACHE = 48  # decoded depth images kept for single-place queries
PARAMS = (
    MAX_RANGE,
    MAX_RANGE_NO_DEPTH,
    DEPTH_TOL,
    DEPTH_TOL_REL,
    MIN_Z,
    N_RING,
    RING_FRAC,
    WIN,
)
SORTS = ("newest", "oldest", "visible")


def is_place_record(rec) -> bool:
    """A traversability place with (possibly) a ray boundary: MESH_PLACES (3p1)."""
    return rec is not None and (rec.layer, rec.part) == MESH_PLACES


def place_samples(
    pos, radii=None, n_ring: int = N_RING, frac: float = RING_FRAC
) -> np.ndarray:
    """Place centre plus ``n_ring`` points at ``frac`` of the boundary ray lengths
    (just the centre without a boundary). Same sampling as place_features.py."""
    p = np.asarray(pos, float)
    pts = [p]
    if radii:
        nr = len(radii)
        for k in range(n_ring):
            j = int(round(k * nr / n_ring)) % nr
            a = 2.0 * np.pi * j / nr
            r = frac * float(radii[j])
            pts.append(p + np.array([r * np.cos(a), r * np.sin(a), 0.0]))
    return np.array(pts)


@dataclass(frozen=True)
class Camera:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    depth_scale: float


def project(pts_w: np.ndarray, T_ws: np.ndarray, cam: Camera):
    """World points (..., 3) into a camera: (z, range, u, v, in view without occlusion)."""
    R, t = T_ws[:3, :3], T_ws[:3, 3]
    pc = (pts_w - t) @ R  # world -> camera (R^T (p - t))
    z = pc[..., 2]
    rng = np.linalg.norm(pc, axis=-1)
    zs = np.maximum(z, 1e-6)
    u = cam.fx * pc[..., 0] / zs + cam.cx
    v = cam.fy * pc[..., 1] / zs + cam.cy
    ok = (
        (z > MIN_Z)
        & (rng < MAX_RANGE)
        & (u >= 0)
        & (u < cam.width)
        & (v >= 0)
        & (v < cam.height)
    )
    return z, rng, u, v, ok


def seen_points(z, rng, u, v, ok, cam: Camera, depth) -> np.ndarray:
    """Which in-view points are not occluded in ``depth`` (metres, any resolution)."""
    seen = np.zeros(ok.shape, bool)
    idx = np.nonzero(ok)
    if not len(idx[0]):
        return seen
    zi, ri = z[idx], rng[idx]
    if depth is None:
        seen[idx] = ri < MAX_RANGE_NO_DEPTH
        return seen
    dh, dw = depth.shape
    ui = (u[idx] * dw / cam.width).astype(int)
    vi = (v[idx] * dh / cam.height).astype(int)
    off = np.arange(-WIN, WIN + 1)
    rows = vi[:, None, None] + off[None, :, None]  # (S, 5, 1)
    cols = ui[:, None, None] + off[None, None, :]  # (S, 1, 5)
    inside = (rows >= 0) & (rows < dh) & (cols >= 0) & (cols < dw)
    win = depth[np.clip(rows, 0, dh - 1), np.clip(cols, 0, dw - 1)]
    win = np.where(inside & (win > 0), win, np.nan).reshape(len(zi), -1)
    valid = ~np.isnan(win).all(axis=1)
    med = np.full(len(zi), np.nan)
    if valid.any():
        med[valid] = np.nanmedian(win[valid], axis=1)
    tol = np.maximum(DEPTH_TOL, DEPTH_TOL_REL * zi)
    seen[idx] = np.where(valid, med >= zi - tol, ri < MAX_RANGE_NO_DEPTH)
    return seen


def load_depth(path, depth_scale: float):
    """A depth PNG in metres, or None (missing / unreadable)."""
    if path is None:
        return None
    try:
        from PIL import Image

        with Image.open(path) as im:
            return np.asarray(im, dtype=np.float32) * float(depth_scale)
    except Exception:  # noqa: BLE001
        return None


def bag_short(bag: str | None) -> str | None:
    """Short name of a bag path: the last component that is not a generic leaf."""
    if not bag:
        return None
    parts = [p for p in str(bag).rstrip("/").split("/") if p]
    for p in reversed(parts):
        if p not in ("recorded_data", "bags", "bag", "rosbag", "rosbag2"):
            return p
    return parts[-1] if parts else None


class PlaceKeyframeIndex:
    """Keyframes of one map version (camera poses, intrinsics, depth) against its places."""

    def __init__(self, places: dict, frames: list, sources: dict | None = None,
                 cache_dir=None, label: str = ""):  # fmt: skip
        """``places``: sym -> (pos, radii | None). ``frames``: dicts with ``frame_id,
        t_ns, robot_id, T_ws (4x4 world_T_sensor), cam (Camera), rgb, depth, anchor (sym |
        None)``. ``sources``: robot prefix -> {session, robot, bag} (trajectory robots)."""
        self.label = label
        self.place_syms = list(places)
        self.place_of = {s: i for i, s in enumerate(self.place_syms)}
        self.centres = np.array([places[s][0] for s in self.place_syms], float).reshape(
            -1, 3
        )
        self.samples = [place_samples(*places[s]) for s in self.place_syms]
        self.frames = frames
        self._by_fid = {int(f["frame_id"]): f for f in frames}
        self.T = np.array([f["T_ws"] for f in frames], float).reshape(-1, 4, 4)
        self.cam_pos = self.T[:, :3, 3]
        self.robot_keys = self._robot_keys(frames)
        self.sessions = self._sessions(sources or {})
        self.key = self._content_key()
        self.cache_dir = pathlib.Path(cache_dir).expanduser() if cache_dir else None
        self._table: dict[int, np.ndarray] | None = (
            None  # place index -> rows (kf, visible, dist)
        )
        self._per_place: dict[int, np.ndarray] = {}
        self._depth: OrderedDict = OrderedDict()
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self.build_seconds: float | None = None
        self._load_cached()

    def __len__(self) -> int:
        return len(self.frames)

    @property
    def ready(self) -> bool:
        return self._table is not None

    # -- identity ---------------------------------------------------------------
    @staticmethod
    def _robot_keys(frames) -> list[str]:
        """Each frame's robot key: its anchor's symbol category (the trajectory robot
        prefix); unanchored frames take the commonest category of their robot_id."""
        by_robot: dict[int, Counter] = {}
        for f in frames:
            if f["anchor"]:
                by_robot.setdefault(f["robot_id"], Counter())[f["anchor"][0]] += 1
        out = []
        for f in frames:
            if f["anchor"]:
                out.append(f["anchor"][0])
            elif by_robot.get(f["robot_id"]):
                out.append(by_robot[f["robot_id"]].most_common(1)[0][0])
            else:
                out.append(f"r{f['robot_id']}")
        return out

    def _sessions(self, sources: dict) -> dict:
        out: dict[str, dict] = {}
        for f, k in zip(self.frames, self.robot_keys):
            s = out.get(k)
            if s is None:
                info = sources.get(k) or {}
                s = out[k] = {
                    "key": k, "session": info.get("session"), "robot": info.get("robot"),
                    "bag": info.get("bag"), "bag_short": bag_short(info.get("bag")),
                    "first_ns": f["t_ns"], "last_ns": f["t_ns"],
                }  # fmt: skip
            s["first_ns"] = min(s["first_ns"], f["t_ns"])
            s["last_ns"] = max(s["last_ns"], f["t_ns"])
        return out

    def _content_key(self) -> str:
        h = hashlib.sha1(repr(PARAMS).encode())
        h.update(np.ascontiguousarray(self.centres).round(3).tobytes())
        for s in self.samples:
            h.update(np.ascontiguousarray(s).round(3).tobytes())
        h.update(np.array([f["frame_id"] for f in self.frames], np.int64).tobytes())
        h.update(np.ascontiguousarray(self.T).round(4).tobytes())
        for f in self.frames:
            h.update(repr(f["cam"]).encode())
        return h.hexdigest()[:20]

    # -- disk cache ---------------------------------------------------------------
    def _cache_file(self):
        return self.cache_dir / f"{self.key}.npz" if self.cache_dir else None

    def _load_cached(self) -> None:
        path = self._cache_file()
        if path is None or not path.is_file():
            return
        try:
            d = np.load(path)
            self._table = self._split(d["place"], d["rows"])
            self.build_seconds = 0.0
            log.info("%s: place keyframes from %s", self.label, path)
        except Exception as e:  # noqa: BLE001
            log.warning("place keyframe cache %s unreadable: %s", path, e)

    def _save(self, place: np.ndarray, rows: np.ndarray) -> None:
        path = self._cache_file()
        if path is None:
            return
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            tmp = path.with_suffix(".tmp.npz")
            np.savez(tmp, place=place, rows=rows)
            tmp.replace(path)
            # keep the newest few tables (a table per map version)
            old = sorted(
                path.parent.glob("*.npz"), key=lambda p: p.stat().st_mtime, reverse=True
            )
            for p in old[8:]:
                p.unlink(missing_ok=True)
        except OSError as e:
            log.warning("place keyframe cache %s not written: %s", path, e)

    @staticmethod
    def _split(place: np.ndarray, rows: np.ndarray) -> dict[int, np.ndarray]:
        out: dict[int, np.ndarray] = {}
        if not len(place):
            return out
        order = np.argsort(place, kind="stable")
        place, rows = place[order], rows[order]
        cuts = np.flatnonzero(np.diff(place)) + 1
        for chunk_p, chunk in zip(np.split(place, cuts), np.split(rows, cuts)):
            out[int(chunk_p[0])] = chunk
        return out

    # -- visibility ----------------------------------------------------------------
    def _depth_of(self, k: int):
        f = self.frames[k]
        with self._lock:
            if k in self._depth:
                self._depth.move_to_end(k)
                return self._depth[k]
        d = load_depth(f["depth"], f["cam"].depth_scale)
        with self._lock:
            self._depth[k] = d
            while len(self._depth) > DEPTH_CACHE:
                self._depth.popitem(last=False)
        return d

    def _frame_rows(self, k: int, cand: list[int], depth) -> list[tuple]:
        """(place, visible, dist) for the candidate places keyframe ``k`` sees."""
        if not cand:
            return []
        cam = self.frames[k]["cam"]
        n = np.array([len(self.samples[i]) for i in cand])
        pts = np.concatenate([self.samples[i] for i in cand])
        z, rng, u, v, ok = project(pts, self.T[k], cam)
        if not ok.any():
            return []
        seen = seen_points(z, rng, u, v, ok, cam, depth)
        counts = np.add.reduceat(seen.astype(int), np.r_[0, np.cumsum(n)[:-1]])
        out = []
        for j in np.flatnonzero(counts):
            i = cand[j]
            out.append(
                (
                    i,
                    counts[j] / n[j],
                    float(np.linalg.norm(self.centres[i] - self.cam_pos[k])),
                )
            )
        return out

    def _compute_place(self, i: int) -> np.ndarray:
        """One place against every keyframe: frustum + range first, depth on the shortlist."""
        if not len(self.frames):
            return np.zeros((0, 3))
        near = np.flatnonzero(
            np.linalg.norm(self.cam_pos - self.centres[i], axis=1)
            < MAX_RANGE + CAND_PAD
        )
        pts = self.samples[i]
        rows = []
        for k in near:
            *_, ok = project(pts, self.T[k], self.frames[k]["cam"])
            if not ok.any():
                continue
            for _, vis, dist in self._frame_rows(int(k), [i], self._depth_of(int(k))):
                rows.append((k, vis, dist))
        return np.array(rows, float).reshape(-1, 3)

    def build(self) -> None:
        """The full place x keyframe table (one depth decode per keyframe)."""
        t0 = time.time()
        if len(self.centres):
            from scipy.spatial import cKDTree

            tree = cKDTree(self.centres)
        place, rows = [], []
        for k, f in enumerate(self.frames):
            if not len(self.centres):
                break
            cand = tree.query_ball_point(self.cam_pos[k], MAX_RANGE + CAND_PAD)
            if not cand:
                continue
            # frustum + range before decoding the depth image
            cand = [
                i
                for i in cand
                if project(self.samples[i], self.T[k], f["cam"])[4].any()
            ]
            if not cand:
                continue
            for i, vis, dist in self._frame_rows(
                k, cand, load_depth(f["depth"], f["cam"].depth_scale)
            ):
                place.append(i)
                rows.append((k, vis, dist))
        place_a = np.array(place, np.int64)
        rows_a = np.array(rows, float).reshape(-1, 3)
        self._table = self._split(place_a, rows_a)
        self.build_seconds = round(time.time() - t0, 2)
        log.info("%s: place keyframes for %d places x %d keyframes (%d pairs) in %.1fs", self.label,
                 len(self.place_syms), len(self.frames), len(place), self.build_seconds)  # fmt: skip
        self._save(place_a, rows_a)
        with self._lock:
            self._per_place.clear()
            self._depth.clear()

    def start_build(self) -> None:
        """Build the full table in a background thread (once)."""
        with self._lock:
            if self._table is not None or self._thread is not None:
                return
            self._thread = threading.Thread(
                target=self._build_safe, name="place-keyframes", daemon=True
            )
            self._thread.start()

    def _build_safe(self) -> None:
        try:
            self.build()
        except Exception as e:  # noqa: BLE001
            log.warning("%s: place keyframe table failed: %s", self.label, e)

    def wait(self, timeout: float | None = None) -> bool:
        t = self._thread
        if t is not None:
            t.join(timeout)
        return self.ready

    def rows(self, sym: str) -> np.ndarray:
        """(keyframe index, visible, dist) rows of place ``sym`` (KeyError: no such place)."""
        i = self.place_of[sym]
        if self._table is not None:
            return self._table.get(i, np.zeros((0, 3)))
        with self._lock:
            cur = self._per_place.get(i)
        if cur is None:
            cur = self._compute_place(i)
            with self._lock:
                self._per_place[i] = cur
        return cur

    def query(self, sym: str, sort: str = "newest") -> dict:
        """Every keyframe that observed place ``sym``, sorted (``SORTS``)."""
        rows = self.rows(sym)
        kfs = []
        for k, vis, dist in rows:
            k = int(k)
            f = self.frames[k]
            kfs.append({
                "frame_id": int(f["frame_id"]), "agent": f["anchor"], "robot_key": self.robot_keys[k],
                "t_ns": int(f["t_ns"]), "visible": round(float(vis), 3), "dist_m": round(float(dist), 2),
            })  # fmt: skip
        if sort == "visible":
            kfs.sort(key=lambda r: (-r["visible"], r["dist_m"], -r["t_ns"]))
        else:
            kfs.sort(key=lambda r: (r["t_ns"], r["frame_id"]), reverse=sort != "oldest")
        used = {r["robot_key"] for r in kfs}
        sessions = []
        for k, s in self.sessions.items():
            if k in used:
                ts = [r["t_ns"] for r in kfs if r["robot_key"] == k]
                sessions.append({**s, "n": len(ts), "first_ns": min(ts), "last_ns": max(ts),
                                 "date": _date(min(ts))})  # fmt: skip
        sessions.sort(key=lambda s: s["first_ns"], reverse=sort == "newest")
        return {"keyframes": kfs, "sessions": sessions}

    def frame_rgb(self, frame_id: int):
        """The rgb path of an indexed keyframe (None: not one of this map's keyframes)."""
        f = self._by_fid.get(int(frame_id))
        return None if f is None else f["rgb"]


def _date(t_ns: int) -> str | None:
    try:
        return (
            datetime.datetime.fromtimestamp(t_ns / 1e9, datetime.timezone.utc)
            .date()
            .isoformat()
        )
    except (OverflowError, OSError, ValueError):
        return None


def store_keyframes(store, lookup=None) -> list[dict]:
    """The keyframes of an ``hydra_python.image_store.ImageStore`` with their camera poses:
    world_T_body from ``lookup`` (node id -> 4x4, the graph's agent poses) via the
    anchor, else the odometric pose (``ImageStore.body_pose``), then body_T_sensor."""
    from hydra_python.image_store import FrameKind, as_pose_lookup

    cams, body_T_sensor = {}, {}
    for c in store.sensors():
        if c is None:
            continue
        cams[c.sensor_id] = Camera(int(c.width), int(c.height), float(c.fx), float(c.fy), float(c.cx),
                                   float(c.cy), float(c.depth_scale or 0.001))  # fmt: skip
        body_T_sensor[c.sensor_id] = np.asarray(c.body_T_sensor, float)
    lk = as_pose_lookup(lookup)
    canon: dict[int, int] = {}
    out = []
    for f in store.frames(kind=int(FrameKind.KEYFRAME)):
        if f.rgb_path is None or f.sensor_id not in cams:
            continue
        world_T_body, _ = store.body_pose(f, lk)
        anchor = None
        if f.anchor_node_id is not None:
            a = int(f.anchor_node_id)
            if a not in canon:
                canon[a] = store.canonical(a)
            anchor = _sym(canon[a])
        out.append({
            "frame_id": int(f.frame_id), "t_ns": int(f.timestamp_ns), "robot_id": int(f.robot_id),
            "T_ws": np.asarray(world_T_body, float) @ body_T_sensor[f.sensor_id], "cam": cams[f.sensor_id],
            "rgb": f.rgb_path, "depth": f.depth_path, "anchor": anchor,
        })  # fmt: skip
    return out


def quat_pose(xyz, w, x, y, z) -> np.ndarray:
    """4x4 from a position and a (w, x, y, z) quaternion."""
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    w, x, y, z = w / n, x / n, y / n, z / n
    T = np.eye(4)
    T[:3, :3] = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]
    T[:3, 3] = xyz
    return T


def _sym(node_id: int) -> str:
    return f"{chr((node_id >> 56) & 0xFF)}({node_id & ((1 << 56) - 1)})"
