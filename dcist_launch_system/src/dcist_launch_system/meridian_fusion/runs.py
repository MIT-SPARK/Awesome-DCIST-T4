"""Loading a run-meridian output directory (one robot run localized by meridian).

Frames:
    utm     the aerial image's frame, which meridian's solution is in
    odom    the run's odometry frame, which its segment map and ground submaps are in
    body    the robot body frame of the odometry (T_camera_flu of meridian's params)
"""

import pathlib
import pickle

import numpy as np
from robotdatapy.data import PoseData
from scipy.spatial.transform import Rotation as Rot
from scipy.spatial.transform import Slerp


def yaw(T):
    return np.arctan2(T[..., 1, 0], T[..., 0, 0])


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def se2(T):
    """3x3 SE(2) (x, y, yaw) of a 4x4 pose."""
    c, s = np.cos(yaw(T)), np.sin(yaw(T))
    return np.array([[c, -s, T[0, 3]], [s, c, T[1, 3]], [0.0, 0.0, 1.0]])


def gravity_aligned(T):
    """4x4 pose with T's position and yaw only."""
    out = np.eye(4)
    out[:2, :2] = se2(T)[:2, :2]
    out[:3, 3] = T[:3, 3]
    return out


def path_length(xy):
    return np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))])


def interp_poses(times, poses, query):
    """Poses at query times: positions interpolated linearly, rotations slerped; NaN outside times."""
    out = np.full((len(query), 4, 4), np.nan)
    inside = (query >= times[0]) & (query <= times[-1])
    if inside.sum() == 0:
        return out
    q = query[inside]
    T = np.tile(np.eye(4), (len(q), 1, 1))
    for k in range(3):
        T[:, k, 3] = np.interp(q, times, poses[:, k, 3])
    T[:, :3, :3] = Slerp(times, Rot.from_matrix(poses[:, :3, :3]))(q).as_matrix()
    out[inside] = T
    return out


def _unique_times(times, poses):
    times, idx = np.unique(times, return_index=True)
    return times, poses[idx]


class Run:
    """One run-meridian output directory.

    Attributes:
        times, T_odom_body: meridian's mapping nodes (segment map trajectory) and their odometry
        est: (n, 4, 4) meridian's 2D solution, T_utm_body at the nodes
        aerial: [(node, 3x3 T_utm_body)] meridian's accepted (inlier) aerial loop closures
        dense_times, dense: meridian's full-rate T_utm_body trajectory of the same solution
        submaps: meridian's ground submaps (odom frame)
        gt_body: (n, 4, 4) ground truth T_utm_body at the nodes (NaN where unknown / no GT)
    """

    def __init__(self, run_dir, name, solution="localization/rerun"):
        from meridian.map3d.submap import Submap
        from meridian.params.data_params import CrossViewLocalizationDataParams

        self.name = name
        self.dir = pathlib.Path(run_dir)
        self.params_file = self.dir / "params.yaml"
        data_params = CrossViewLocalizationDataParams.load(str(self.params_file))
        self.T_camera_flu = (np.eye(4) if data_params.T_camera_flu is None
                             else np.asarray(data_params.T_camera_flu, dtype=float))

        with open(self.dir / "segment_map.pkl", "rb") as f:
            segment_map = pickle.load(f)
        self.times = np.asarray(segment_map.times, dtype=float)
        self.T_odom_body = np.asarray(segment_map.trajectory) @ self.T_camera_flu
        self.path_s = path_length(self.T_odom_body[:, :2, 3])
        del segment_map

        sol = self.dir / solution
        with open(sol / "optimized_trajectory.pkl", "rb") as f:
            self.est = np.asarray(pickle.load(f), dtype=float)
        if len(self.est) != len(self.times):
            raise ValueError(f"{sol}: {len(self.est)} solution poses for {len(self.times)} nodes")
        with open(sol / "candidates.pkl", "rb") as f:
            candidates = pickle.load(f)
        inliers = np.load(sol / "inlier_indices.npy").astype(int)
        self.aerial = [(self.node(candidates[i]["ground_submap_time"]),
                        np.asarray(candidates[i]["T_utm_body_se2"], dtype=float)) for i in inliers]
        dense = PoseData.from_csv(str(sol / "dense_trajectory_3d.csv"))
        self.dense_times, self.dense = _unique_times(np.asarray(dense.times), dense.all_poses())

        self.submaps = [Submap.load(str(f)) for f in sorted((self.dir / "ground" / "segments").glob("*.pkl"),
                                                             key=lambda f: int(f.stem))]
        self.submap_nodes = [self.node(sm.time) for sm in self.submaps]

        self.gt_body = np.full((len(self.times), 4, 4), np.nan)
        self.gt_times, self.gt = None, None
        if data_params.gt_pose_data:
            gt = PoseData.from_dict(data_params.gt_pose_data)  # T_utm_camera (meridian's convention)
            self.gt_times, self.gt = _unique_times(np.asarray(gt.times), gt.all_poses() @ self.T_camera_flu)
            self.gt_body = interp_poses(self.gt_times, self.gt, self.times)

    def node(self, t):
        return int(np.argmin(np.abs(self.times - t)))

    def T_utm_odom(self, est, k):
        """T_utm_odom (4x4) implied by the 2D solution at node k."""
        return est[k] @ np.linalg.inv(self.T_odom_body[k])

    def submap_center_utm(self, i):
        sm = self.submaps[i]
        return (self.T_utm_odom(self.est, self.submap_nodes[i]) @ sm.camera_pose)[:2, 3]

    def has_gt(self, k):
        return bool(np.isfinite(self.gt_body[k]).all())
