#!/usr/bin/env python3
"""Associate each experiment in experiments.csv with the corresponding trial
row in `experiment_notes.xlsx`, using planning-problem PDDL files as the
bridge.

The base station logs every planning request as
    omniplanner_problem_<YYYY-MM-DD>_<HH>_<MM>_<SS>_<HASH>.pddl
under `planning_problems/`. Each file contains a `(:goal ...)` form. We:

  1. Parse every problem PDDL and extract (local timestamp, UTC timestamp, goal).
  2. Assign each problem to the experiment whose start time is the closest one
     within a small window of the problem's timestamp.
  3. For each day with experiments, parse the matching xlsx sheet and score
     each (experiment, trial) pair on PDDL similarity (predicate-verb match +
     Jaccard over the object/place IDs referenced).
  4. Emit a review CSV with the candidates so the user can confirm or edit.

Run with the ROS 2 Jazzy workspace sourced and `spark_env` python on PATH (for
pandas + openpyxl).
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import glob
import re
import sys
from pathlib import Path

import pandas as pd


EDT = dt.timezone(dt.timedelta(hours=-4))   # West Point, NY in April

# experiment_set -> day prefix to look up in the xlsx sheet names + the
# YYYY-MM-DD date stamp on the problem PDDLs we should consider.
DAYS = {
    "april_15_bravo": ("Wednesday", "2026-04-15"),
    "april_16_alpha": ("Thursday",  "2026-04-16"),
    "april_17_alpha": ("Friday",    "2026-04-17"),
}

# Matching window for problem -> experiment.
# We pick the experiment with the closest start_time within
# [start - LEAD_S, end + LAG_S]. The closest-start tiebreak keeps boundary
# problems on the run they actually triggered, so LEAD_S can be generous.
LEAD_S = 1200.0  # 20 min before start
LAG_S  = 60.0    # 1 min grace after end (re-issue mid-run)

# Minimum score below which we report no trial match instead of forcing one.
MIN_TRIAL_SCORE = 0.05


# ---- problem PDDL parsing ----------------------------------------------


_PROBLEM_RE = re.compile(
    r"omniplanner_problem_(\d{4}-\d{2}-\d{2})_(\d{2})_(\d{2})_(\d{2})_(\w+)\.pddl"
)


def parse_problem(path: Path) -> dict | None:
    m = _PROBLEM_RE.match(path.name)
    if not m:
        return None
    date, h, mn, s, hsh = m.groups()
    local = dt.datetime.fromisoformat(f"{date}T{h}:{mn}:{s}").replace(tzinfo=EDT)
    utc = local.astimezone(dt.timezone.utc)
    txt = path.read_text()
    goal_m = re.search(r"\(:goal\s+(.+?)\)\s*\(:metric", txt, flags=re.S)
    goal = re.sub(r"\s+", " ", goal_m.group(1).strip()) if goal_m else "?"
    return {
        "file": path.name,
        "date": date,
        "local": local,
        "utc": utc,
        "goal": goal,
    }


def load_problems(problem_dir: Path) -> list[dict]:
    out: list[dict] = []
    for p in sorted(problem_dir.glob("omniplanner_problem_*.pddl")):
        rec = parse_problem(p)
        if rec is not None:
            out.append(rec)
    out.sort(key=lambda r: r["utc"])
    return out


# ---- problem -> experiment grouping ------------------------------------


def assign_problems(experiments: pd.DataFrame, problems: list[dict]) -> dict[str, list[dict]]:
    """Return {experiment_id: [problem_rec, ...]}.

    A problem is assigned to the experiment whose start_time it is closest to
    in a window [start - LEAD_S, end + LAG_S]. Ties broken by the smallest
    |problem - start| so that boundary problems land on the run they
    triggered, not the previous one.
    """
    groups: dict[str, list[dict]] = {eid: [] for eid in experiments["experiment_id"]}
    starts = experiments["start_time_unix"].to_numpy()
    ends = experiments["end_time_unix"].to_numpy()
    eids = experiments["experiment_id"].tolist()

    for p in problems:
        t = p["utc"].timestamp()
        best = None
        best_score = float("inf")
        for s, e, eid in zip(starts, ends, eids):
            if t < s - LEAD_S or t > e + LAG_S:
                continue
            # Score = absolute distance from start_time. Problems are usually
            # issued just before the experiment kicks off, so closest start
            # wins for boundary situations.
            score = abs(t - s)
            if score < best_score:
                best_score = score
                best = eid
        if best is not None:
            groups[best].append(p)
    return groups


# ---- xlsx parsing ------------------------------------------------------


_OBJ_RE   = re.compile(r"O\d+", flags=re.IGNORECASE)
_PLACE_RE = re.compile(r"\bt\d+", flags=re.IGNORECASE)
_VERB_RE  = re.compile(r"\(([a-z\-]+)\s")


def normalize_goal(s: str) -> str:
    if not isinstance(s, str):
        return ""
    return re.sub(r"\s+", " ", s).strip().lower()


def goal_features(s: str) -> tuple[set[str], set[str], set[str]]:
    s = normalize_goal(s)
    objs = {m.upper() for m in _OBJ_RE.findall(s)}
    places = {m.lower() for m in _PLACE_RE.findall(s)}
    verbs = set(_VERB_RE.findall(s)) - {"and", "or", "not"}
    return objs, places, verbs


def parse_trial_sheet(df: pd.DataFrame) -> list[dict]:
    """Group sheet rows into trials.

    A trial starts on a row with a non-empty Trial #. Subsequent NaN-trial
    rows are follow-up clarifications belonging to the same trial.
    """
    trials: list[dict] = []
    cur: dict | None = None
    for _, r in df.iterrows():
        tn = r.get("Trial #")
        if pd.notna(tn):
            if cur is not None:
                trials.append(cur)
            cur = {
                "trial": str(tn).strip(),
                "instruction": "",
                "expected_pddl": "",
                "generated_pddl": "",
                "exec_success": "",
                "execution_notes": "",
                "grounding_notes": "",
            }
        if cur is None:
            continue
        for col_csv, col_xlsx in [
            ("instruction", "Instruction"),
            ("expected_pddl", "Expected PDDL"),
            ("generated_pddl", "Generated PDDL"),
            ("exec_success", "Execution Success"),
            ("execution_notes", "Execution Notes"),
            ("grounding_notes", "Grounding Notes"),
        ]:
            v = r.get(col_xlsx)
            if pd.notna(v):
                cur[col_csv] = (cur[col_csv] + "\n" + str(v)).strip()
    if cur is not None:
        trials.append(cur)
    return trials


def load_trials(xlsx_path: Path) -> dict[str, list[dict]]:
    """Return {day_keyword (Wednesday/Thursday/Friday/...): [trial dicts]}."""
    xl = pd.ExcelFile(xlsx_path)
    out: dict[str, list[dict]] = {}
    for sheet in xl.sheet_names:
        df = xl.parse(sheet)
        # Day keyword: split on " - " and take the left side of " ("
        # e.g. "ARL Missions - Wednesday (April 15"  ->  "Wednesday"
        m = re.match(r"ARL Missions - (\w+)", sheet)
        if not m:
            continue
        day = m.group(1)
        out[day] = parse_trial_sheet(df)
    return out


# ---- scoring -----------------------------------------------------------


def score_experiment_against_trial(exp_problems: list[dict], trial: dict) -> float:
    """Higher = better match.

    Combines:
      - Jaccard over object IDs (referenced in the experiment's problems vs.
        the trial's expected/generated PDDLs).
      - Jaccard over place IDs.
      - Verb-set match (e.g. visited-object, object-in-place, at-object).
    """
    if not exp_problems:
        return 0.0
    exp_text = " ".join(p["goal"] for p in exp_problems)
    e_objs, e_places, e_verbs = goal_features(exp_text)

    trial_text = " ".join([trial.get("expected_pddl", ""),
                           trial.get("generated_pddl", "")])
    t_objs, t_places, t_verbs = goal_features(trial_text)

    if not (e_objs | t_objs) and not (e_places | t_places):
        return 0.0

    def jacc(a: set, b: set) -> float:
        if not a and not b:
            return 0.0
        return len(a & b) / max(1, len(a | b))

    s_obj = jacc(e_objs, t_objs)
    s_place = jacc(e_places, t_places)
    s_verb = jacc(e_verbs, t_verbs)
    return 0.6 * s_obj + 0.3 * s_place + 0.1 * s_verb


# ---- driver ------------------------------------------------------------


def build_associations(
    experiments: pd.DataFrame,
    problems_by_eid: dict[str, list[dict]],
    trials_by_day: dict[str, list[dict]],
) -> list[dict]:
    """Per-day greedy chronological assignment.

    Within a day:
      - Sort experiments by start_time (already chronological in the CSV).
      - For each experiment, pick the best-scoring trial that (a) clears the
        minimum score threshold, (b) has index >= the last assigned trial
        index (so retries map onto consecutive trial rows naturally).
      - Trials are NOT mutually exclusive: a trial may match more than one
        experiment if no later trial scores higher (the chronological floor
        prevents an earlier-trial match from overwriting a later one).
    """
    rows: list[dict] = []
    for exp_set, (day_key, _) in DAYS.items():
        sub = experiments[experiments["experiment_set"] == exp_set].copy()
        sub = sub.sort_values("start_time_unix").reset_index(drop=True)
        trials = trials_by_day.get(day_key, [])
        last_idx = -1                       # chronological floor
        consumed: set[int] = set()           # trials already assigned

        # Sort key for two candidate scored entries:
        #   1) higher score wins
        #   2) tied scores → earliest trial wins (lower idx)
        def _key(s):
            return (round(s[0], 4), -s[1])

        for _, r in sub.iterrows():
            eid = r["experiment_id"]
            probs = problems_by_eid.get(eid, [])
            scored = [
                (score_experiment_against_trial(probs, t), idx, t)
                for idx, t in enumerate(trials)
            ]
            best_overall = max(scored, key=lambda x: x[0]) if scored else (0.0, -1, None)

            # Eligible: chronologically allowed (>= last_idx) and clears MIN.
            eligible = [s for s in scored if s[1] >= last_idx and s[0] >= MIN_TRIAL_SCORE]
            if not eligible:
                top1 = (0.0, -1, None)
            else:
                best_any = max(eligible, key=_key)
                cand_uncons = [s for s in eligible if s[1] not in consumed]
                if cand_uncons:
                    best_uncons = max(cand_uncons, key=_key)
                    # Prefer an unconsumed trial only when it's within 0.05 of
                    # the absolute best — this keeps friday's three identical
                    # `(object-in-place O102 t4987)` retries on trials 19/20/21
                    # without forcing thursday's retry onto a wrong-PDDL trial.
                    if best_uncons[0] >= best_any[0] - 0.05:
                        top1 = best_uncons
                    else:
                        top1 = best_any
                else:
                    top1 = best_any
            if top1[1] >= 0:
                last_idx = top1[1]
                consumed.add(top1[1])

            # Top-2 (best score after top1) for review purposes.
            scored_sorted = sorted(scored, key=lambda x: x[0], reverse=True)
            top2 = next((s for s in scored_sorted if s[2] is not top1[2]), (0.0, -1, None))

            rows.append({
                "experiment_id": eid,
                "experiment_set": exp_set,
                "day": day_key,
                "start_utc": r["start_time_iso"],
                "duration_s": r["duration_s"],
                "n_problems": len(probs),
                "problem_files":  " | ".join(p["file"] for p in probs),
                "problem_local_times": " | ".join(p["local"].strftime("%H:%M:%S") for p in probs),
                "problem_goals": " | ".join(p["goal"] for p in probs),
                "top1_trial":   top1[2]["trial"] if top1[2] else "",
                "top1_score":   round(top1[0], 3),
                "top1_instruction": (top1[2] or {}).get("instruction", "")[:120],
                "top1_expected_pddl": (top1[2] or {}).get("expected_pddl", "").replace("\n", " "),
                "top1_generated_pddl": (top1[2] or {}).get("generated_pddl", "").replace("\n", " "),
                "top1_exec_success": (top1[2] or {}).get("exec_success", ""),
                "top2_trial":   top2[2]["trial"] if top2[2] else "",
                "top2_score":   round(top2[0], 3),
                "top2_expected_pddl": (top2[2] or {}).get("expected_pddl", "").replace("\n", " "),
                "best_overall_trial": best_overall[2]["trial"] if best_overall[2] else "",
                "best_overall_score": round(best_overall[0], 3),
            })
    return rows


def write_csv(rows: list[dict], path: Path) -> None:
    if not rows:
        path.write_text("")
        return
    cols = list(rows[0].keys())
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for r in rows:
            w.writerow(r)


# ---- summary table -----------------------------------------------------


def print_summary(rows: list[dict]) -> None:
    by_set: dict[str, list[dict]] = {}
    for r in rows:
        by_set.setdefault(r["experiment_set"], []).append(r)
    print("\n=================================================================")
    print(" Experiment <-> trial association summary")
    print("=================================================================\n")
    for exp_set in sorted(by_set):
        print(f"### {exp_set} ({DAYS.get(exp_set, ('?',))[0]})")
        print(f"{'experiment':<28} | {'best trial':>10} | {'score':>5} | {'#prob':>5} | goals")
        print("-" * 110)
        for r in by_set[exp_set]:
            trial = r["top1_trial"] or "(none)"
            score = f"{r['top1_score']:.2f}"
            goals = (r["problem_goals"] or "(no problems matched window)")
            if len(goals) > 60:
                goals = goals[:57] + "..."
            warn = ""
            if r["top1_score"] < MIN_TRIAL_SCORE:
                warn = "  ⚠ no good trial match"
            elif r["best_overall_trial"] != r["top1_trial"]:
                warn = f"  (best-overall would be trial {r['best_overall_trial']} score={r['best_overall_score']:.2f})"
            print(f"{r['experiment_id']:<28} | {trial:>10} | {score:>5} | {r['n_problems']:>5} | {goals}{warn}")
        print()


# ---- main --------------------------------------------------------------


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--master", type=Path,
                    default=Path("/home/harel/data/west_point_2026/corrected/experiments.csv"))
    ap.add_argument("--problem-dir", type=Path,
                    default=Path("/data/dcist/west_point_2026/base_station_willow/planning_problems"))
    ap.add_argument("--xlsx", type=Path,
                    default=Path("/data/dcist/west_point_2026/clean_graphs/experiment_notes.xlsx"))
    ap.add_argument("--output", type=Path,
                    default=Path("/home/harel/data/west_point_2026/corrected/experiment_associations.csv"))
    args = ap.parse_args()

    experiments = pd.read_csv(args.master)
    print(f"Loaded {len(experiments)} experiments from {args.master.name}")

    problems = load_problems(args.problem_dir)
    print(f"Loaded {len(problems)} problem PDDL files")

    trials_by_day = load_trials(args.xlsx)
    print(f"Loaded trials from {len(trials_by_day)} sheets: {list(trials_by_day)}")

    problems_by_eid = assign_problems(experiments, problems)
    n_assigned = sum(len(v) for v in problems_by_eid.values())
    n_unassigned = len(problems) - n_assigned
    print(f"Assigned {n_assigned}/{len(problems)} problems to experiments  "
          f"({n_unassigned} fell outside any experiment window)")

    rows = build_associations(experiments, problems_by_eid, trials_by_day)
    write_csv(rows, args.output)
    print(f"Wrote {args.output}  ({len(rows)} rows)")

    print_summary(rows)
    return 0


if __name__ == "__main__":
    sys.exit(main())
