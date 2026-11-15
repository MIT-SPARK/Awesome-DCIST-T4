"""Raw point clouds re-aggregated from a run's depth images, for ground-to-ground ICP.

Depth frames (meridian's segment_mapping_data.depth_data topic) are taken every frame_spacing_m of travel,
back-projected up to max_depth_m, placed in the odometry frame with meridian's camera trajectory
(interpolated to the frame stamp) and voxelized. Unlike meridian's segments, these clouds keep everything
the camera saw (ground, walls, vegetation), as ROMAN's submap clouds do, and ICP closures on them are
both more numerous and more accurate (Pennovation, 8 runs: 420 vs 300 accepted, median 0.23 vs 0.32 m).
"""

import pathlib
from dataclasses import dataclass

import numpy as np
import open3d as o3d
import yaml

from .runs import interp_poses


@dataclass
class DepthCloudParams:
    frame_spacing_m: float = 0.25  # one depth frame per this much travel
    max_depth_m: float = 5.0  # ROMAN's pcd_max_depth
    min_depth_m: float = 0.3
    pixel_stride: int = 4
    frame_voxel_m: float = 0.1


def _depth_image(msg):
    if msg.encoding == "32FC1":
        d = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.step // 4)[:, :msg.width]
    elif msg.encoding in ("16UC1", "mono16"):
        d = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.step // 2)[:, :msg.width] / 1000.0
    else:
        raise ValueError(f"unsupported depth encoding {msg.encoding}")
    return d.astype(np.float32)


def aggregate(run, P=DepthCloudParams(), log=print):
    """(frame_times, frame_ptr, points): per kept depth frame, its points (float32, odom frame)."""
    from rosbags.highlevel import AnyReader
    from robotdatapy.bag_reader import iter_raw_messages

    with open(run.params_file) as f:
        depth = yaml.safe_load(f)["segment_mapping_data"]["depth_data"]
    if depth.get("type") != "bag" or depth.get("compressed"):
        raise ValueError(f"{run.name}: only uncompressed bag depth images are supported ({depth})")
    T_odom_cam = run.T_odom_body @ np.linalg.inv(run.T_camera_flu)
    times, points, K = [], [], None
    last_s = -np.inf
    with AnyReader([pathlib.Path(depth["path"])]) as reader:
        info = [c for c in reader.connections if c.topic == depth["camera_info_topic"]]
        for con, _, raw in iter_raw_messages(reader, info):
            K = np.asarray(reader.deserialize(raw, con.msgtype).k, dtype=float).reshape(3, 3)
            break
        if K is None:
            raise ValueError(f"{run.name}: no {depth['camera_info_topic']} in {depth['path']}")
        conns = [c for c in reader.connections if c.topic == depth["topic"]]
        t0, t1 = run.times[0], run.times[-1]
        for n, (con, _, raw) in enumerate(iter_raw_messages(reader, conns)):
            msg = reader.deserialize(raw, con.msgtype)
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if t < t0 or t > t1:
                continue
            s = np.interp(t, run.times, run.path_s)
            if s - last_s < P.frame_spacing_m:
                continue
            T = interp_poses(run.times, T_odom_cam, np.array([t]))[0]
            d = _depth_image(msg)[::P.pixel_stride, ::P.pixel_stride]
            v, u = np.mgrid[0:msg.height:P.pixel_stride, 0:msg.width:P.pixel_stride]
            ok = np.isfinite(d) & (d >= P.min_depth_m) & (d <= P.max_depth_m)
            z = d[ok]
            pc = np.stack([(u[ok] - K[0, 2]) * z / K[0, 0], (v[ok] - K[1, 2]) * z / K[1, 1], z], axis=1)
            pw = pc @ T[:3, :3].T + T[:3, 3]
            pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pw))
            points.append(np.asarray(pcd.voxel_down_sample(P.frame_voxel_m).points, dtype=np.float32))
            times.append(t)
            last_s = s
            if len(times) % 500 == 0:
                log(f"  {run.name}: {len(times)} depth frames ({s:.0f} of {run.path_s[-1]:.0f} m)")
    ptr = np.concatenate([[0], np.cumsum([len(p) for p in points])])
    return np.array(times), ptr, np.concatenate(points)


def load_or_aggregate(run, cache, P=DepthCloudParams(), log=print):
    """aggregate(), cached in the .npz file cache (rebuilt if the params or the run's nodes differ)."""
    cache = pathlib.Path(cache)
    key = np.array([P.frame_spacing_m, P.max_depth_m, P.min_depth_m, P.pixel_stride, P.frame_voxel_m])
    if cache.exists():
        z = np.load(cache)
        if np.allclose(z["params"], key) and len(z["run_times"]) == len(run.times):
            return z["times"], z["ptr"], z["points"]
    log(f"aggregating {run.name} depth clouds -> {cache}")
    times, ptr, points = aggregate(run, P, log)
    cache.parent.mkdir(parents=True, exist_ok=True)
    np.savez(cache, times=times, ptr=ptr, points=points, params=key, run_times=run.times)
    return times, ptr, points
