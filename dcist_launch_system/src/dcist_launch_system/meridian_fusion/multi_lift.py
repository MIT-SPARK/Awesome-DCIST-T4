"""(Copied from dcist_experiments.meridian_multi_trajectory_optimization.multi_lift.)

Multi-trajectory version of meridian's 3D lift (meridian.cross_view.rpgo.CrossViewRPGO.lift_to_3d).

Adapted from meridian's cross_path_pairs, _solve_heights and _refine_3d so that several trajectories can
be lifted jointly. Nodes of all trajectories are concatenated, with chain_ids giving each node's trajectory
(each trajectory's nodes contiguous and in time order). Compared to meridian:
    - there are no odometry factors between trajectories
    - nodes of different trajectories within cross_paths_3d_constraint_dist_m share a height (no minimum
      path distance); same-trajectory pairs work as in meridian
    - height is anchored at the reference trajectory's first node, plus one node for each group of
      trajectories that no pairs connect to the reference (see height_anchor_nodes)
    - roll / pitch are anchored at those same nodes; the first roll / pitch of every other trajectory gets
      a soft prior (chain_roll_pitch_sigma_rad) instead, since its tilt relative to the reference is
      observable through the cross-trajectory pairs
With a single trajectory this reduces to meridian's lift exactly.
"""

import logging

import numpy as np
import scipy.sparse as sp
import scipy.sparse.linalg as spla
from meridian.params.cross_view_params import CrossViewRPGOParams
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as Rot

logger = logging.getLogger(__name__)

# Anchor sigma (rad / m) for anchored roll, pitch and height (meridian rpgo _LIFT_ANCHOR_SIGMA)
ANCHOR_SIGMA = 1e-3


def chain_starts(chain_ids: np.ndarray) -> np.ndarray:
    """Index of the first node of each trajectory."""
    return np.flatnonzero(np.r_[True, chain_ids[1:] != chain_ids[:-1]])


def cross_path_pairs(
    xy: np.ndarray, chain_ids: np.ndarray, dist_m: float, min_path_dist_m: float
) -> np.ndarray:
    """Index pairs (i, j), i < j, of nodes within dist_m of each other in 2D that should share a height:
    nodes of different trajectories, or of the same trajectory at least min_path_dist_m apart along its
    path. Each node j keeps only its nearest such partner i (meridian rpgo.cross_path_pairs)."""
    if len(xy) < 2:
        return np.zeros((0, 2), dtype=int)
    pairs = cKDTree(xy).query_pairs(dist_m, output_type="ndarray")
    if len(pairs) == 0:
        return np.zeros((0, 2), dtype=int)
    step = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    step[chain_ids[1:] != chain_ids[:-1]] = 0.0
    path_s = np.concatenate([[0.0], np.cumsum(step)])
    pairs = np.sort(pairs, axis=1)
    revisit = path_s[pairs[:, 1]] - path_s[pairs[:, 0]] >= min_path_dist_m
    revisit |= chain_ids[pairs[:, 0]] != chain_ids[pairs[:, 1]]
    pairs = pairs[revisit]
    if len(pairs) == 0:
        return np.zeros((0, 2), dtype=int)
    dist = np.linalg.norm(xy[pairs[:, 0]] - xy[pairs[:, 1]], axis=1)
    pairs = pairs[np.argsort(dist, kind="stable")]
    _, first = np.unique(pairs[:, 1], return_index=True)
    return pairs[np.sort(first)]


def height_anchor_nodes(chain_ids: np.ndarray, pairs: np.ndarray, reference_chain: int) -> np.ndarray:
    """First node of the reference trajectory, plus the first node of the lowest-index trajectory in each
    group that no pairs connect to the reference (its height relative to the reference is unobservable)."""
    starts = chain_starts(chain_ids)
    chains = chain_ids[starts]
    lookup = {c: k for k, c in enumerate(chains)}
    a = np.array([lookup[c] for c in chain_ids[pairs[:, 0]]], dtype=int)
    b = np.array([lookup[c] for c in chain_ids[pairs[:, 1]]], dtype=int)
    links = sp.coo_matrix((np.ones(len(pairs)), (a, b)), shape=(len(chains),) * 2)
    _, component = connected_components(links, directed=False)
    ref = lookup[reference_chain]
    anchors = [starts[ref]]
    for c in np.unique(component):
        if c == component[ref]:
            continue
        members = np.flatnonzero(component == c)
        logger.warning(f"Trajectories {chains[members].tolist()} share no height pairs with trajectory "
                       f"{reference_chain}; keeping their own height and tilt")
        anchors.append(starts[members[0]])
    return np.array(anchors)


def lift_to_3d(
    optimized_trajectory: np.ndarray,
    trajectory: np.ndarray,
    chain_ids: np.ndarray,
    params: CrossViewRPGOParams,
    reference_chain: int = 0,
    chain_roll_pitch_sigma_rad: float = np.deg2rad(2.0),
):
    """Lift several trajectories to 3D jointly (meridian CrossViewRPGO.lift_to_3d).

    Args:
        optimized_trajectory: (n, 4, 4) T_world_body; x, y and yaw are used.
        trajectory: (n, 4, 4) T_odom_body odometry at the same nodes (relative motion, roll / pitch).
        chain_ids: (n,) trajectory id per node.
        params: meridian CrossViewRPGOParams (cross_paths_3d_*, odom_*, lift_3d_* fields are used).
        reference_chain: trajectory id whose first height (and tilt) is kept.
        chain_roll_pitch_sigma_rad: prior on the first roll / pitch of non-anchored trajectories.

    Returns:
        (n, 4, 4) lifted T_world_body, and the (k, 2) same-height pairs used.
    """
    chain_ids = np.asarray(chain_ids)
    T_odom_body = np.asarray(trajectory)
    T_2d = np.asarray(optimized_trajectory)
    n = len(T_odom_body)
    xy = T_2d[:, :2, 3]
    yaw_2d = np.arctan2(T_2d[:, 1, 0], T_2d[:, 0, 0])
    yaw_odom = np.arctan2(T_odom_body[:, 1, 0], T_odom_body[:, 0, 0])

    pairs = cross_path_pairs(
        xy, chain_ids, params.cross_paths_3d_constraint_dist_m, params.cross_paths_3d_constraint_path_dist_m
    )
    starts = chain_starts(chain_ids)
    anchors = height_anchor_nodes(chain_ids, pairs, reference_chain)
    logger.info(f"3D lift: {n} nodes, {len(starts)} trajectories, {len(pairs)} height constraints")

    z = solve_heights(T_odom_body[:, 2, 3], pairs, starts, anchors, params)

    lifted = np.tile(np.eye(4), (n, 1, 1))
    lifted[:, :3, :3] = Rot.from_euler("z", (yaw_2d - yaw_odom)[:, None]).as_matrix() @ T_odom_body[:, :3, :3]
    lifted[:, :2, 3] = xy
    lifted[:, 2, 3] = z

    if params.lift_3d_refine_rotation and n > 1:
        rp_sigma = np.where(np.isin(starts, anchors), ANCHOR_SIGMA, chain_roll_pitch_sigma_rad)
        lifted = refine_3d(lifted, T_odom_body, xy, pairs, starts, rp_sigma, anchors, params)
    return lifted, pairs


def solve_heights(z_odom, pairs, starts, anchors, params: CrossViewRPGOParams) -> np.ndarray:
    """Weighted linear least squares for node heights (meridian CrossViewRPGO._solve_heights)."""
    n = len(z_odom)
    if n == 1:
        return z_odom.copy()
    w_odom = 1.0 / params.odom_trans_sigma_m
    w_pair = 1.0 / params.cross_paths_3d_constraint_sigma_m
    w_anchor = 1e3

    new_chain = np.zeros(n, dtype=bool)
    new_chain[starts] = True
    idx = np.flatnonzero(~new_chain[1:])
    na = len(anchors)
    rows = np.concatenate([np.arange(na), na + np.repeat(np.arange(len(idx)), 2)])
    cols = np.concatenate([anchors, np.stack([idx, idx + 1], 1).ravel()])
    vals = np.concatenate([np.full(na, w_anchor), np.tile([-w_odom, w_odom], len(idx))])
    b = np.concatenate([w_anchor * z_odom[anchors], w_odom * (z_odom[idx + 1] - z_odom[idx])])
    if len(pairs):
        r0 = na + len(idx)
        prow = r0 + np.repeat(np.arange(len(pairs)), 2)
        rows = np.concatenate([rows, prow])
        cols = np.concatenate([cols, pairs.ravel()])
        vals = np.concatenate([vals, np.tile([w_pair, -w_pair], len(pairs))])
        b = np.concatenate([b, np.zeros(len(pairs))])
    A = sp.csr_matrix((vals, (rows, cols)), shape=(len(b), n))
    return spla.spsolve((A.T @ A).tocsc(), A.T @ b)


def refine_3d(lifted, T_odom_body, xy, pairs, starts, rp_sigma, anchors, params: CrossViewRPGOParams,
              max_iterations: int = 50) -> np.ndarray:
    """Pose3 graph refining the lifted poses (meridian CrossViewRPGO._refine_3d), solved with
    Levenberg-Marquardt on SE(3).

    Factors: 3D odometry between consecutive poses of each trajectory; x, y, yaw priors from the 2D
    solution; equal world-frame height for pairs; roll / pitch priors (rp_sigma) on each trajectory's
    first pose; and height anchors at the anchors nodes.
    """
    n = len(lifted)
    R = lifted[:, :3, :3].copy()
    t = lifted[:, :3, 3].copy()
    yaw_prior = np.arctan2(R[:, 1, 0], R[:, 0, 0])
    R_anchor, z_anchor = R[starts].copy(), t[anchors, 2].copy()
    new_chain = np.zeros(n, dtype=bool)
    new_chain[starts] = True
    i0 = np.flatnonzero(~new_chain[1:])
    i1 = i0 + 1
    m = len(i0)
    T_rel = np.linalg.inv(T_odom_body[i0]) @ T_odom_body[i1]
    Rm, tm = T_rel[:, :3, :3], T_rel[:, :3, 3]
    w_rot = 1.0 / params.odom_rot_sigma_rad
    w_tran = 1.0 / params.odom_trans_sigma_m
    w_xy = 1.0 / params.lift_3d_prior_xy_sigma_m
    w_yaw = 1.0 / params.lift_3d_prior_yaw_sigma_rad
    w_h = 1.0 / params.cross_paths_3d_constraint_sigma_m
    w_rp = 1.0 / np.asarray(rp_sigma)
    w_anchor = 1.0 / ANCHOR_SIGMA
    eye3 = np.broadcast_to(np.eye(3), (m, 3, 3))

    def residuals(R, t):
        E = np.swapaxes(Rm, 1, 2) @ np.swapaxes(R[i0], 1, 2) @ R[i1]
        dt = t[i1] - t[i0]
        yaw = np.arctan2(R[:, 1, 0], R[:, 0, 0])
        r_rp = Rot.from_matrix(np.swapaxes(R_anchor, 1, 2) @ R[starts]).as_rotvec()[:, :2]
        return np.concatenate(
            [
                w_rot * Rot.from_matrix(E).as_rotvec().ravel(),
                w_tran * (np.einsum("nji,nj->ni", R[i0], dt) - tm).ravel(),
                w_xy * (t[:, :2] - xy).ravel(),
                w_yaw * ((yaw - yaw_prior + np.pi) % (2 * np.pi) - np.pi),
                w_h * (t[pairs[:, 0], 2] - t[pairs[:, 1], 2]),
                (w_rp[:, None] * r_rp).ravel(),
                w_anchor * (t[anchors, 2] - z_anchor),
            ]
        )

    def jacobian(R, t):
        # Right perturbations: R <- R Exp(dtheta), t <- t + R dt; node k's
        # tangent occupies columns 6k:6k+3 (dtheta) and 6k+3:6k+6 (dt).
        rows, cols, vals = [], [], []

        def block(r0, node, c_off, M):
            # M: (m, a, b) blocks placed at rows r0 + a*m, cols 6*node + c_off
            k, a, b = M.shape
            rr = r0[:, None, None] + np.arange(a)[None, :, None]
            cc = 6 * node[:, None, None] + c_off + np.arange(b)[None, None, :]
            rows.append(np.broadcast_to(rr, M.shape).ravel())
            cols.append(np.broadcast_to(cc, M.shape).ravel())
            vals.append(M.ravel())

        r = 0
        # odometry rotation residual Log(Rm^T Ri^T Rj)
        rr = r + 3 * np.arange(m)
        block(rr, i0, 0, -w_rot * np.swapaxes(R[i1], 1, 2) @ R[i0])
        block(rr, i1, 0, w_rot * eye3)
        r += 3 * m
        # odometry translation residual Ri^T (tj - ti) - tm
        rr = r + 3 * np.arange(m)
        dt_body = np.einsum("nji,nj->ni", R[i0], t[i1] - t[i0])
        skew = np.zeros((m, 3, 3))
        skew[:, 0, 1], skew[:, 0, 2] = -dt_body[:, 2], dt_body[:, 1]
        skew[:, 1, 0], skew[:, 1, 2] = dt_body[:, 2], -dt_body[:, 0]
        skew[:, 2, 0], skew[:, 2, 1] = -dt_body[:, 1], dt_body[:, 0]
        block(rr, i0, 0, w_tran * skew)
        block(rr, i0, 3, -w_tran * eye3)
        block(rr, i1, 3, w_tran * np.swapaxes(R[i0], 1, 2) @ R[i1])
        r += 3 * m
        # x, y priors
        nodes = np.arange(n)
        block(r + 2 * nodes, nodes, 3, w_xy * R[:, :2, :])
        r += 2 * n
        # yaw prior: d atan2(R10, R00) / d dtheta
        d = R[:, 0, 0] ** 2 + R[:, 1, 0] ** 2
        J_yaw = np.zeros((n, 1, 3))
        J_yaw[:, 0, 1] = (R[:, 1, 0] * R[:, 0, 2] - R[:, 0, 0] * R[:, 1, 2]) / d
        J_yaw[:, 0, 2] = (R[:, 0, 0] * R[:, 1, 1] - R[:, 1, 0] * R[:, 0, 1]) / d
        block(r + nodes, nodes, 0, w_yaw * J_yaw)
        r += n
        # equal heights
        if len(pairs):
            pr = r + np.arange(len(pairs))
            block(pr, pairs[:, 0], 3, w_h * R[pairs[:, 0], 2:3, :])
            block(pr, pairs[:, 1], 3, -w_h * R[pairs[:, 1], 2:3, :])
            r += len(pairs)
        # roll / pitch (local rotation x, y) of each trajectory's first node
        ns = len(starts)
        J_rp = w_rp[:, None, None] * np.broadcast_to(np.eye(3)[:2, :], (ns, 2, 3))
        block(r + 2 * np.arange(ns), starts, 0, J_rp)
        r += 2 * ns
        # height anchors
        block(r + np.arange(len(anchors)), anchors, 3, w_anchor * R[anchors, 2:3, :])
        r += len(anchors)
        return sp.csr_matrix(
            (np.concatenate(vals), (np.concatenate(rows), np.concatenate(cols))),
            shape=(r, 6 * n),
        )

    res = residuals(R, t)
    cost = res @ res
    lam = 1e-4
    for _ in range(max_iterations):
        J = jacobian(R, t)
        H = (J.T @ J).tocsc()
        g = J.T @ res
        while True:
            damp = sp.diags(lam * (H.diagonal() + 1e-9))
            delta = spla.spsolve(H + damp, -g).reshape(n, 6)
            R_new = R @ Rot.from_rotvec(delta[:, :3]).as_matrix()
            t_new = t + np.einsum("nij,nj->ni", R, delta[:, 3:])
            res_new = residuals(R_new, t_new)
            cost_new = res_new @ res_new
            if cost_new < cost or lam > 1e8:
                break
            lam *= 10
        if cost_new >= cost:
            break
        converged = cost - cost_new < 1e-9 * cost
        R, t, res, cost = R_new, t_new, res_new, cost_new
        lam = max(lam / 10, 1e-12)
        if converged:
            break

    out = np.tile(np.eye(4), (n, 1, 1))
    out[:, :3, :3], out[:, :3, 3] = R, t
    return out
