"""Joint pose graph over several meridian runs, then a joint height lift.

2D (GTSAM Pose2), one node per meridian mapping node of every run:
    odometry     between consecutive nodes of a run (relative pose of their gravity-aligned projections),
                 with meridian's cross_view_rpgo odometry sigmas
    aerial       a prior at each of meridian's accepted aerial loop closures, cross_view_rpgo prior sigmas
    closures     between factors for the accepted ground-to-ground closures (Huber)
    initial      meridian's solutions
3D: multi_lift (heights from odometry increments plus equal heights for nodes of any runs within
cross_paths_3d_constraint_dist_m, then a Pose3 roll / pitch refinement) on nodes every lift_node_spacing_m of
travel, densified onto meridian's full-rate trajectory of each run (meridian densify_trajectory).
"""

from dataclasses import dataclass

import gtsam
import numpy as np

from . import multi_lift
from .runs import interp_poses, path_length, se2


@dataclass
class GraphParams:
    closure_trans_sigma_m: float = 0.3
    closure_rot_sigma_deg: float = 2.0
    closure_huber_k: float = 1.345  # 0: no robust kernel
    lift_node_spacing_m: float = 1.0
    reference_run: int = 0  # whose first height and tilt the lift keeps
    chain_roll_pitch_sigma_deg: float = 2.0


def _pose2(T3):
    return gtsam.Pose2(T3[0, 2], T3[1, 2], np.arctan2(T3[1, 0], T3[0, 0]))


def solve_2d(runs, rpgo_params, closures, G, use_closures=True):
    """{run name: (n, 4, 4) flat T_utm_body at the run's nodes}."""
    rp = rpgo_params
    graph, values = gtsam.NonlinearFactorGraph(), gtsam.Values()
    odom = gtsam.noiseModel.Diagonal.Sigmas(np.array([rp.odom_trans_sigma_m, rp.odom_trans_sigma_m,
                                                      rp.odom_rot_sigma_rad]))
    prior = gtsam.noiseModel.Diagonal.Sigmas(np.array([rp.prior_trans_sigma_m, rp.prior_trans_sigma_m,
                                                       np.deg2rad(rp.prior_rot_sigma_deg)]))
    lc = gtsam.noiseModel.Diagonal.Sigmas(np.array([G.closure_trans_sigma_m, G.closure_trans_sigma_m,
                                                    np.deg2rad(G.closure_rot_sigma_deg)]))
    if G.closure_huber_k > 0:
        lc = gtsam.noiseModel.Robust.Create(gtsam.noiseModel.mEstimator.Huber.Create(G.closure_huber_k), lc)
    offset, total = {}, 0
    for r in runs:
        offset[r.name] = total
        total += len(r.times)
        for i in range(len(r.times)):
            values.insert(offset[r.name] + i, _pose2(se2(r.est[i])))
        for i in range(len(r.times) - 1):
            Z = np.linalg.inv(se2(r.T_odom_body[i])) @ se2(r.T_odom_body[i + 1])
            graph.add(gtsam.BetweenFactorPose2(offset[r.name] + i, offset[r.name] + i + 1, _pose2(Z), odom))
        for k, T in r.aerial:
            graph.add(gtsam.PriorFactorPose2(offset[r.name] + k, _pose2(T), prior))
    if use_closures:
        for c in closures:
            graph.add(gtsam.BetweenFactorPose2(offset[c["run_a"]] + c["node_a"], offset[c["run_b"]] + c["node_b"],
                                               _pose2(c["Z"]), lc))
    result = gtsam.LevenbergMarquardtOptimizer(graph, values, gtsam.LevenbergMarquardtParams()).optimize()
    out = {}
    for r in runs:
        T = np.tile(np.eye(4), (len(r.times), 1, 1))
        for i in range(len(r.times)):
            p = result.atPose2(offset[r.name] + i)
            c, s = np.cos(p.theta()), np.sin(p.theta())
            T[i, :2, :2] = [[c, -s], [s, c]]
            T[i, :2, 3] = [p.x(), p.y()]
        out[r.name] = T
    return out


def _node_indices(xy, spacing):
    s = path_length(xy)
    idx = np.searchsorted(s, np.arange(0.0, s[-1], spacing))
    return np.unique(np.concatenate([idx, [len(xy) - 1]]))


def lift_and_densify(runs, est_2d, rpgo_params, G):
    """Joint 3D lift of the 2D solutions; returns ({run name: (m, 4, 4) dense T_utm_body}, height pairs info)."""
    from meridian.cross_view.rpgo import densify_trajectory

    idx = {r.name: _node_indices(est_2d[r.name][:, :2, 3], G.lift_node_spacing_m) for r in runs}
    nodes_2d = np.concatenate([est_2d[r.name][idx[r.name]] for r in runs])
    nodes_odom = np.concatenate([r.T_odom_body[idx[r.name]] for r in runs])
    chain_ids = np.concatenate([np.full(len(idx[r.name]), k) for k, r in enumerate(runs)])
    lifted, pairs = multi_lift.lift_to_3d(nodes_2d, nodes_odom, chain_ids, rpgo_params, G.reference_run,
                                          np.deg2rad(G.chain_roll_pitch_sigma_deg))
    dense, z_meridian = {}, []
    for k, r in enumerate(runs):
        t_nodes = r.times[idx[r.name]]
        # meridian's dense trajectory carries the odometry's full-rate motion; correct it at the lifted nodes
        at_nodes = interp_poses(r.dense_times, r.dense, t_nodes)
        z_meridian.append(at_nodes[:, 2, 3])
        ok = np.isfinite(at_nodes[:, 0, 0])
        # densify in a frame near the trajectory: the correction is interpolated as a rigid transform, and
        # about the UTM origin a fraction of a degree of tilt change is kilometres of translation
        origin = np.eye(4)
        origin[:3, 3] = np.round(np.nanmean(at_nodes[ok, :3, 3], axis=0))
        to_local = np.linalg.inv(origin)
        dense[r.name] = origin @ densify_trajectory(t_nodes[ok], to_local @ lifted[chain_ids == k][ok],
                                                    to_local @ at_nodes[ok], r.dense_times, to_local @ r.dense)
    z_meridian = np.concatenate(z_meridian)
    cross = chain_ids[pairs[:, 0]] != chain_ids[pairs[:, 1]]
    dz = np.abs(lifted[pairs[:, 0], 2, 3] - lifted[pairs[:, 1], 2, 3])
    dz_meridian = np.abs(z_meridian[pairs[:, 0]] - z_meridian[pairs[:, 1]])
    info = dict(n_pairs=len(pairs), n_cross=int(cross.sum()),
                dz_cross_after=float(np.nanmedian(dz[cross])) if cross.any() else np.nan,
                dz_cross_meridian=float(np.nanmedian(dz_meridian[cross])) if cross.any() else np.nan)
    return dense, info


def xy_errors(run, est):
    """xy error (m) and |yaw| error (deg) at the run's nodes with ground truth."""
    ok = np.isfinite(run.gt_body[:, 0, 3])
    e = np.linalg.norm(est[ok, :2, 3] - run.gt_body[ok, :2, 3], axis=1)
    dyaw = np.arctan2(np.sin(np.arctan2(est[ok, 1, 0], est[ok, 0, 0]) - np.arctan2(run.gt_body[ok, 1, 0],
                                                                                   run.gt_body[ok, 0, 0])),
                      np.cos(np.arctan2(est[ok, 1, 0], est[ok, 0, 0]) - np.arctan2(run.gt_body[ok, 1, 0],
                                                                                   run.gt_body[ok, 0, 0])))
    return e, np.degrees(np.abs(dyaw))


def dense_xy_errors(run, dense):
    """xy error (m) of a full-rate trajectory against ground truth (where the ground truth covers it)."""
    if run.gt is None:
        return np.zeros(0)
    gt = interp_poses(run.gt_times, run.gt, run.dense_times)
    ok = np.isfinite(gt[:, 0, 3])
    return np.linalg.norm(dense[ok, :2, 3] - gt[ok, :2, 3], axis=1)
