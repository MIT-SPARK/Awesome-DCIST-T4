"""Ground-to-ground loop closures between meridian runs: ICP on re-aggregated depth clouds.

1. Candidate pairs: ground submaps (of any two runs, or of one run at least min_intra_path_dist_m of
   travel apart) whose centres are within pair_dist_m in meridian's solutions.
2. ICP: ROMAN's refine_with_icp between point clouds re-aggregated from the runs' depth images
   (depth_clouds.py) around the two submaps' nodes, started from the solutions' relative pose. Closures ICP
   does not accept (fitness below icp_min_fitness, worse than the start, or moved more than
   icp_max_translation_m / icp_max_rotation_deg) are dropped.

A closure is Z, the 3x3 SE(2) pose of run b's body at node_b in run a's body at node_a (gravity aligned).
"""

import itertools
from dataclasses import dataclass, field

import numpy as np
import open3d as o3d

from .depth_clouds import DepthCloudParams, load_or_aggregate
from .runs import gravity_aligned, se2, yaw


@dataclass
class ClosureParams:
    pair_dist_m: float = 20.0  # submap centres within this in the solutions
    min_intra_path_dist_m: float = 40.0  # same-run pairs: travel between the two submaps
    # ICP clouds: depth frames within cloud_window_m of travel of the node, cropped to cloud_radius_m
    # (horizontal) of it
    cloud_window_m: float = 20.0
    cloud_radius_m: float = 20.0
    cloud_voxel_m: float = 0.1
    ground_percentile: float = 5.0  # initial height difference: these percentiles of the clouds' z
    # ROMAN refine_with_icp (ROMAN's SubmapAlignParams defaults, except the rotation limit)
    icp_distances: list = field(default_factory=lambda: [1.0, 0.5, 0.25, 0.1])
    icp_min_fitness: float = 0.2
    icp_max_translation_m: float = 1.5
    icp_max_rotation_deg: float = 10.0


def candidate_pairs(runs, P):
    """(run a, submap i, run b, submap j) for submaps whose centres are within pair_dist_m."""
    entries = [(r, i, r.submap_center_utm(i)) for r in runs for i in range(len(r.submaps))]
    pairs = []
    for (ra, i, ca), (rb, j, cb) in itertools.combinations(entries, 2):
        if np.linalg.norm(ca - cb) > P.pair_dist_m:
            continue
        if ra is rb and abs(ra.path_s[ra.submap_nodes[i]] - ra.path_s[rb.submap_nodes[j]]) < P.min_intra_path_dist_m:
            continue
        pairs.append((ra, i, rb, j))
    return pairs


def _relative(Ta, Tb):
    return np.linalg.inv(se2(Ta)) @ se2(Tb)


def _difference(Z, Z_ref):
    D = np.linalg.inv(Z_ref) @ Z
    return float(np.linalg.norm(D[:2, 2])), float(np.degrees(abs(np.arctan2(D[1, 0], D[0, 0]))))


def _row(ra, i, rb, j):
    na, nb = ra.submap_nodes[i], rb.submap_nodes[j]
    row = dict(run_a=ra.name, submap_a=i, node_a=na, t_a=float(ra.times[na]),
               run_b=rb.name, submap_b=j, node_b=nb, t_b=float(rb.times[nb]),
               Z_est=_relative(ra.est[na], rb.est[nb]), status="")
    if ra.has_gt(na) and rb.has_gt(nb):
        row["Z_gt"] = _relative(ra.gt_body[na], rb.gt_body[nb])
    return row


class CloudBuilder:
    """ICP clouds of a run's re-aggregated depth frames around a node, in the node's gravity-aligned body frame."""

    def __init__(self, P, depth_params=DepthCloudParams(), cache_dir=None, log=print):
        self.P = P
        self.depth_params = depth_params
        self.cache_dir = cache_dir
        self.log = log
        self._clouds = {}
        self._depth = {}

    def cloud(self, run, k):
        key = (run.name, k)
        if key not in self._clouds:
            self._clouds[key] = self._build(run, k)
        return self._clouds[key]

    def _build(self, run, k):
        if run.name not in self._depth:
            cache = (run.dir if self.cache_dir is None else self.cache_dir / run.name) / "depth_clouds.npz"
            self._depth[run.name] = load_or_aggregate(run, cache, self.depth_params, self.log)
        times, ptr, points = self._depth[run.name]
        near = np.abs(run.path_s - run.path_s[k]) <= self.P.cloud_window_m
        a = np.searchsorted(times, run.times[near].min())
        b = np.searchsorted(times, run.times[near].max(), side="right")
        if b <= a:
            return np.empty((0, 3))
        P = points[ptr[a]:ptr[b]].astype(float)
        P = P[np.linalg.norm(P[:, :2] - run.T_odom_body[k, :2, 3], axis=1) <= self.P.cloud_radius_m]
        C = gravity_aligned(run.T_odom_body[k])
        P = (P - C[:3, 3]) @ C[:3, :3]
        pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(P))
        return np.asarray(pcd.voxel_down_sample(self.P.cloud_voxel_m).points)


class IcpRefinement:
    """ROMAN's submap point-cloud ICP (roman.align.object_registration.refine_with_icp)."""

    def __init__(self, P):
        from roman.align.object_registration import ObjectRegistration
        from roman.params.submap_align_params import SubmapAlignParams

        defaults = SubmapAlignParams()
        self.registration = ObjectRegistration(dim=3)
        self.registration.set_icp_params(
            correspondence_distances=P.icp_distances,
            max_iterations=defaults.icp_max_iterations,
            min_fitness=P.icp_min_fitness,
            max_source_points=defaults.icp_max_source_points,
            normal_radius=defaults.icp_normal_radius,
            normal_max_nn=defaults.icp_normal_max_nn,
            max_translation=P.icp_max_translation_m,
            max_rotation_deg=P.icp_max_rotation_deg,
        )
        self.P = P
        self._targets = {}

    def refine(self, key_a, cloud_a, cloud_b, Z0):
        """ICP of cloud b onto cloud a from Z0 (3x3), the height difference taken from the clouds' ground.
        Returns (Z, accepted, info)."""
        if len(cloud_a) < 3 or len(cloud_b) < 3:
            return Z0, False, dict(icp_note="empty cloud")
        if key_a not in self._targets:
            self._targets[key_a] = self.registration.prepare_icp_cloud(cloud_a)
        T0 = np.eye(4)
        T0[:2, :2], T0[:2, 3] = Z0[:2, :2], Z0[:2, 2]
        q = self.P.ground_percentile
        T0[2, 3] = np.percentile(cloud_a[:, 2], q) - np.percentile(cloud_b[:, 2], q)
        T1 = self.registration.refine_with_icp(T0, cloud_a, cloud_b, xyz_yaw_only=True,
                                               target_pcd=self._targets[key_a])
        accepted = not np.allclose(T1, T0)
        corr = T1 @ np.linalg.inv(T0)
        info = dict(icp_moved_m=float(np.linalg.norm(corr[:2, 3])),
                    icp_turned_deg=float(np.degrees(abs(yaw(corr)))), icp_dz_m=float(T1[2, 3]))
        Z = np.array([[T1[0, 0], T1[0, 1], T1[0, 3]], [T1[1, 0], T1[1, 1], T1[1, 3]], [0.0, 0.0, 1.0]])
        return Z, accepted, info


def find_closures(runs, P, depth_params=DepthCloudParams(), cache_dir=None, log=print):
    """Every candidate pair's row (dict); rows with status 'accepted' carry the closure Z. Depth clouds are
    cached as <run dir>/depth_clouds.npz, or <cache_dir>/<run name>/depth_clouds.npz."""
    pairs = candidate_pairs(runs, P)
    log(f"{len(pairs)} candidate submap pairs within {P.pair_dist_m} m "
        f"({sum(a is not b for a, _, b, _ in pairs)} between runs)")
    clouds, icp = CloudBuilder(P, depth_params, cache_dir, log), IcpRefinement(P)
    for r in runs:  # aggregate (or load) every run's depth clouds up front
        clouds.cloud(r, 0)
    rows = []
    for n, (ra, i, rb, j) in enumerate(pairs):
        if n % 200 == 0 and n:
            log(f"  {n}/{len(pairs)} pairs, {sum(r['status'] == 'accepted' for r in rows)} accepted")
        row = _row(ra, i, rb, j)
        rows.append(row)
        key_a = (ra.name, row["node_a"])
        Z, accepted, info = icp.refine(key_a, clouds.cloud(ra, row["node_a"]), clouds.cloud(rb, row["node_b"]),
                                       row["Z_est"])
        row.update(info)
        if not accepted:
            row["status"] = "icp_rejected"
            continue
        row["Z"], row["status"] = Z, "accepted"
    return rows


def closure_errors(row, key):
    """(m, deg) error of row[key] vs ground truth, or None."""
    if "Z_gt" not in row or key not in row:
        return None
    return _difference(row[key], row["Z_gt"])
