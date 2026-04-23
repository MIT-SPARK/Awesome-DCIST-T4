#!/usr/bin/env python3
"""Streamlit app for browsing and editing experiment results.

Launch with:
    streamlit run review_experiments.py -- \
        --master /path/to/experiments.csv \
        --viz-root /path/to/viz_out

Or set defaults via environment:
    EXPERIMENTS_MASTER=/path/to/experiments.csv
    EXPERIMENTS_VIZ_ROOT=/path/to/viz_out
"""

from __future__ import annotations

import argparse
import csv
import datetime
import os
import subprocess
import sys
from pathlib import Path
from typing import Optional

import streamlit as st

# ---------------------------------------------------------------------------
# Argument parsing (Streamlit passes everything after '--' as sys.argv[1:])
# ---------------------------------------------------------------------------

def _parse_args():
    ap = argparse.ArgumentParser(add_help=False)
    ap.add_argument("--master", type=Path,
                    default=os.environ.get("EXPERIMENTS_MASTER"))
    ap.add_argument("--viz-root", type=Path,
                    default=os.environ.get("EXPERIMENTS_VIZ_ROOT", "viz_out"))
    args, _ = ap.parse_known_args()
    return args


_ARGS = _parse_args()

# ---------------------------------------------------------------------------
# CSV helpers
# ---------------------------------------------------------------------------

_LIST_SEPS = ("|", ";")  # prefer | (new), tolerate ; (old)


def _split_list(val: str) -> list[str]:
    for sep in _LIST_SEPS:
        if sep in val:
            return [x for x in val.split(sep) if x]
    return [val] if val.strip() else []


@st.cache_data(ttl=5)
def _load_rows(path: str) -> list[dict]:
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def _save_rows(path: Path, rows: list[dict]) -> None:
    if not rows:
        return
    fieldnames = list(rows[0].keys())
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        w.writerows(rows)
    _load_rows.clear()  # bust cache


def _update_row(rows: list[dict], exp_id: str, updates: dict) -> list[dict]:
    return [
        {**r, **updates} if r["experiment_id"] == exp_id else r
        for r in rows
    ]


# ---------------------------------------------------------------------------
# Time helpers
# ---------------------------------------------------------------------------

def _unix_to_dt(unix: float) -> datetime.datetime:
    return datetime.datetime.fromtimestamp(unix, tz=datetime.timezone.utc)


def _dt_to_unix(dt: datetime.datetime) -> float:
    return dt.timestamp()


def _iso(unix: float) -> str:
    return _unix_to_dt(unix).strftime("%Y-%m-%dT%H:%M:%S.%fZ")


def _dt_from_date_time(d: datetime.date, t: datetime.time) -> datetime.datetime:
    return datetime.datetime(
        d.year, d.month, d.day, t.hour, t.minute, t.second,
        tzinfo=datetime.timezone.utc,
    )


# ---------------------------------------------------------------------------
# Page config
# ---------------------------------------------------------------------------

st.set_page_config(
    page_title="Experiment Review",
    layout="wide",
    initial_sidebar_state="expanded",
)

st.markdown("""
<style>
  .stVideo { border-radius: 6px; }
  div[data-testid="metric-container"] { background: #1e2130; border-radius: 8px; padding: 8px 12px; }
  .candidate-btn button { font-size: 0.78rem; padding: 2px 8px; }
</style>
""", unsafe_allow_html=True)

# ---------------------------------------------------------------------------
# Sidebar — path config + experiment selector
# ---------------------------------------------------------------------------

with st.sidebar:
    st.title("Experiment Review")

    master_str = st.text_input(
        "Master CSV",
        value=str(_ARGS.master) if _ARGS.master else "",
        placeholder="/path/to/experiments.csv",
    )
    viz_root_str = st.text_input(
        "Viz root",
        value=str(_ARGS.viz_root) if _ARGS.viz_root else "viz_out",
        placeholder="/path/to/viz_out",
    )

    master_path = Path(master_str) if master_str else None
    viz_root = Path(viz_root_str) if viz_root_str else Path("viz_out")

    if not master_path or not master_path.exists():
        st.error("Set a valid --master path above.")
        st.stop()

    rows = _load_rows(str(master_path))
    if not rows:
        st.error("CSV is empty.")
        st.stop()

    sessions = sorted({r.get("experiment_set", "") for r in rows})
    selected_session = st.selectbox("Filter by session", ["All"] + sessions)

    filtered = rows if selected_session == "All" else [
        r for r in rows if r.get("experiment_set") == selected_session
    ]

    exp_labels = [
        f"{r['experiment_id']}  ({float(r.get('duration_s', 0)):.0f}s, {r.get('end_reason', '')})"
        for r in filtered
    ]
    exp_ids = [r["experiment_id"] for r in filtered]

    sel_idx = st.selectbox("Experiment", range(len(exp_ids)),
                           format_func=lambda i: exp_labels[i])

    # Prev/next navigation
    nav_col1, nav_col2 = st.columns(2)
    if nav_col1.button("◀ Prev") and sel_idx > 0:
        sel_idx -= 1
    if nav_col2.button("Next ▶") and sel_idx < len(exp_ids) - 1:
        sel_idx += 1

    row = filtered[sel_idx]
    exp_id = row["experiment_id"]

    st.divider()
    st.caption(f"CSV: {master_path.name}  •  {len(rows)} experiments")

# ---------------------------------------------------------------------------
# Helpers to read out per-robot columns
# ---------------------------------------------------------------------------

KNOWN_ROBOTS = ("euclid", "hamilton", "gauss", "lewis", "pascal", "newton")
PER_ROBOT_SUFFIXES = ["path_length_m", "num_actions", "action_type_counts",
                      "object_classes", "plan_id", "bag_paths"]


def _robots(r: dict) -> list[str]:
    return _split_list(r.get("robots", ""))


def _robot_cols(r: dict) -> dict[str, dict[str, str]]:
    """Return {robot: {suffix: value}} for all per-robot columns."""
    out: dict[str, dict[str, str]] = {}
    for robot in _robots(r):
        out[robot] = {}
        for sfx in PER_ROBOT_SUFFIXES:
            val = r.get(f"{robot}_{sfx}", "")
            if val:
                out[robot][sfx] = val
    return out


def _video_path(robot: str, kind: str) -> Path:
    return viz_root / exp_id / robot / f"{kind}.mp4"


def _traj_path() -> Path:
    return viz_root / exp_id / "trajectory.png"


# ---------------------------------------------------------------------------
# Main content — tabs
# ---------------------------------------------------------------------------

st.title(exp_id)

tab_overview, tab_videos, tab_edit = st.tabs(["Overview", "Videos", "Edit"])

# ---- Overview tab ----------------------------------------------------------

with tab_overview:
    meta_col, traj_col = st.columns([1, 1.8])

    with meta_col:
        st.subheader("Mission")
        t_lo = float(row.get("start_time_unix", 0))
        t_hi = float(row.get("end_time_unix", 0))
        dur = float(row.get("duration_s", 0))

        m1, m2 = st.columns(2)
        m1.metric("Duration", f"{dur:.1f} s")
        m2.metric("End reason", row.get("end_reason", "—"))

        st.write(f"**Start:** {row.get('start_time_iso', '—')}")
        st.write(f"**End:**   {row.get('end_time_iso', '—')}")

        robots = _robots(row)
        st.write(f"**Robots:** {', '.join(robots) if robots else '—'}")
        st.write(f"**Multi-agent:** {'Yes' if row.get('is_multi_agent', '').lower() in ('true', '1') else 'No'}")

        n_int = row.get("num_manual_interventions", "0")
        st.write(f"**Manual interventions:** {n_int}")

        lang = row.get("language_command", "").strip()
        if lang:
            st.info(f"**Language command:** {lang}")

        notes = row.get("notes", "").strip()
        if notes:
            st.warning(f"**Notes:** {notes}")

        # Per-robot metrics
        rdata = _robot_cols(row)
        if rdata:
            st.subheader("Per-robot")
            for robot, vals in rdata.items():
                with st.expander(robot, expanded=True):
                    pl = vals.get("path_length_m", "")
                    na = vals.get("num_actions", "")
                    ac = vals.get("action_type_counts", "")
                    oc = vals.get("object_classes", "")
                    if pl:
                        st.metric("Path length", f"{float(pl):.1f} m")
                    if na:
                        st.write(f"**Actions:** {na}  ({ac})")
                    if oc:
                        st.write(f"**Objects:** {oc}")

        # Candidate end times
        cands_raw = row.get("candidate_end_times", "").strip()
        if cands_raw:
            st.subheader("Candidate end times")
            st.caption("Stops detected during the mission window.")
            for c in _split_list(cands_raw):
                st.write(f"• {c}")

        # Lease events
        lease_raw = row.get("lease_events_in_window", "").strip()
        if lease_raw:
            st.subheader("Lease events")
            for ev in _split_list(lease_raw):
                st.write(f"• {ev}")

    with traj_col:
        st.subheader("Trajectory")
        tp = _traj_path()
        if tp.exists():
            st.image(str(tp), use_container_width=True)
        else:
            st.info(f"trajectory.png not found at {tp}\nRun visualize_experiment.py first.")

# ---- Videos tab ------------------------------------------------------------

with tab_videos:
    robots = _robots(row)
    if not robots:
        st.info("No robots listed for this experiment.")
    else:
        for robot in robots:
            st.subheader(robot)
            vc1, vc2 = st.columns(2)
            with vc1:
                st.caption("RGB")
                rp = _video_path(robot, "rgb")
                if rp.exists():
                    st.video(str(rp))
                else:
                    st.info(f"rgb.mp4 not found\n{rp}")
            with vc2:
                st.caption("Depth")
                dp = _video_path(robot, "depth")
                if dp.exists():
                    st.video(str(dp))
                else:
                    st.info(f"depth.mp4 not found\n{dp}")

    st.divider()
    regen_col, _ = st.columns([1, 3])
    if regen_col.button("Regenerate trajectory + videos"):
        script = Path(__file__).parent / "visualize_experiment.py"
        cmd = [
            sys.executable, str(script),
            "--master", str(master_path),
            "--experiment-id", exp_id,
            "--output-root", str(viz_root),
        ]
        with st.spinner(f"Running visualize_experiment.py for {exp_id} …"):
            result = subprocess.run(cmd, capture_output=True, text=True)
        if result.returncode == 0:
            st.success("Done.")
            if result.stdout:
                st.code(result.stdout)
        else:
            st.error("Failed.")
            st.code(result.stderr or result.stdout)

# ---- Edit tab --------------------------------------------------------------

with tab_edit:
    st.subheader("Edit mission times")

    t_lo = float(row.get("start_time_unix", 0))
    t_hi = float(row.get("end_time_unix", 0))
    start_dt = _unix_to_dt(t_lo)
    end_dt = _unix_to_dt(t_hi)

    # Candidate end times as quick-select buttons
    cands_raw = row.get("candidate_end_times", "").strip()
    if cands_raw:
        st.caption("Click a candidate to use it as the end time:")
        cand_cols = st.columns(min(len(_split_list(cands_raw)), 4))
        for i, c in enumerate(_split_list(cands_raw)):
            with cand_cols[i % 4]:
                if st.button(c, key=f"cand_{i}"):
                    try:
                        cand_dt = datetime.datetime.fromisoformat(
                            c.replace("Z", "+00:00")
                        )
                        end_dt = cand_dt
                    except ValueError:
                        pass

    ecol1, ecol2 = st.columns(2)
    with ecol1:
        st.write("**Start time (UTC)**")
        sd = st.date_input("Start date", value=start_dt.date(), key="start_date")
        st_time = st.time_input("Start time", value=start_dt.time().replace(microsecond=0), key="start_time")
        new_start_dt = _dt_from_date_time(sd, st_time)

    with ecol2:
        st.write("**End time (UTC)**")
        ed = st.date_input("End date", value=end_dt.date(), key="end_date")
        et_time = st.time_input("End time", value=end_dt.time().replace(microsecond=0), key="end_time")
        new_end_dt = _dt_from_date_time(ed, et_time)

    new_dur = (new_end_dt - new_start_dt).total_seconds()
    st.metric("New duration", f"{new_dur:.1f} s",
              delta=f"{new_dur - (t_hi - t_lo):+.1f} s vs current")

    st.divider()
    st.subheader("Labels")
    new_lang = st.text_input(
        "Language command",
        value=row.get("language_command", "") or "",
    )
    new_notes = st.text_area(
        "Notes",
        value=row.get("notes", "") or "",
        height=100,
    )

    st.divider()
    save_col, regen_col2, _ = st.columns([1, 1, 2])

    if save_col.button("💾 Save to CSV", type="primary"):
        new_start_unix = _dt_to_unix(new_start_dt)
        new_end_unix = _dt_to_unix(new_end_dt)
        updates = {
            "start_time_unix": new_start_unix,
            "start_time_iso": _iso(new_start_unix),
            "end_time_unix": new_end_unix,
            "end_time_iso": _iso(new_end_unix),
            "duration_s": round(new_end_unix - new_start_unix, 3),
            "language_command": new_lang,
            "notes": new_notes,
        }
        all_rows = _load_rows(str(master_path))
        updated = _update_row(all_rows, exp_id, updates)
        _save_rows(master_path, updated)
        st.success(f"Saved {exp_id}.")
        st.rerun()

    if regen_col2.button("▶ Regenerate viz"):
        script = Path(__file__).parent / "visualize_experiment.py"
        # Save updated times first
        new_start_unix = _dt_to_unix(new_start_dt)
        new_end_unix = _dt_to_unix(new_end_dt)
        updates = {
            "start_time_unix": new_start_unix,
            "start_time_iso": _iso(new_start_unix),
            "end_time_unix": new_end_unix,
            "end_time_iso": _iso(new_end_unix),
            "duration_s": round(new_end_unix - new_start_unix, 3),
            "language_command": new_lang,
            "notes": new_notes,
        }
        all_rows = _load_rows(str(master_path))
        updated = _update_row(all_rows, exp_id, updates)
        _save_rows(master_path, updated)
        cmd = [
            sys.executable, str(script),
            "--master", str(master_path),
            "--experiment-id", exp_id,
            "--output-root", str(viz_root),
        ]
        with st.spinner("Regenerating trajectory + videos …"):
            result = subprocess.run(cmd, capture_output=True, text=True)
        if result.returncode == 0:
            st.success("Done. Switch to Overview / Videos tabs to see results.")
            if result.stdout:
                st.code(result.stdout)
        else:
            st.error("visualize_experiment.py failed.")
            st.code(result.stderr or result.stdout)
        st.rerun()
