"""Experiment / prior-map helpers shared by the TUI, CLI and web console."""

from __future__ import annotations

from dataclasses import dataclass

# A run directory is usable as a prior map when both of these were saved.
PRIOR_SUBDIRS = ("hydra", "roman")


def is_prior_map(experiment: dict) -> bool:
    """``experiment`` is an entry from ``list_remote_experiments``."""
    subdirs = experiment.get("subdirs", [])
    return all(d in subdirs for d in PRIOR_SUBDIRS)


@dataclass
class FileComparison:
    path: str
    hashes: dict[str, str | None]  # machine -> md5 (None = missing)
    consistent: bool

    def cell(self, machine: str) -> str:
        """Display value: ``MISSING``, ``OK`` or the first 8 hex digits."""
        h = self.hashes[machine]
        if h is None:
            return "MISSING"
        return "OK" if self.consistent else h[:8]


def compare_hashes(machine_hashes: dict[str, dict[str, str]]) -> list[FileComparison]:
    """Compare per-machine ``{relative_path: md5}`` maps file by file.

    A file is consistent only if every machine has it with the same hash.
    """
    machines = sorted(machine_hashes)
    files = sorted({f for h in machine_hashes.values() for f in h})
    rows = []
    for path in files:
        hashes = {mn: machine_hashes[mn].get(path) for mn in machines}
        present = [h for h in hashes.values() if h]
        consistent = len(present) == len(machines) and len(set(present)) == 1
        rows.append(FileComparison(path, hashes, consistent))
    return rows


# ---------------------------------------------------------------------------
# Run listing with contents / sizes / image store (fleet.run_scan over ssh)
# ---------------------------------------------------------------------------

# python on the far side: the workspace env if present, else the system python3
# (run_scan is stdlib only, so any python3 works; no sqlite3 CLI needed).
_REMOTE_PY = (
    'PY="${ADT4_ENV:-$HOME/environments/dcist}/spark_env/bin/python3"; '
    '[ -x "$PY" ] || PY=python3; '
)


def remote_scan_command(
    root: str,
    known: dict | None = None,
    budget: float | None = None,
    only: list[str] | None = None,
    light: bool = False,
    manifest: bool = False,
) -> str:
    """Shell command that runs ``fleet.run_scan`` on a remote machine and prints JSON.

    The module source and the arguments travel base64-encoded inside the command, so
    one ssh round trip lists the machine and nothing needs installing there.
    """
    import base64
    import json
    import pathlib

    src = (pathlib.Path(__file__).with_name("run_scan.py")).read_bytes()
    code = base64.b64encode(src).decode()
    args = base64.b64encode(
        json.dumps(
            {
                "root": root,
                "known": known or {},
                "budget": budget,
                "only": only,
                "light": light,
                "manifest": manifest,
            }
        ).encode()
    ).decode()
    return _REMOTE_PY + f'"$PY" -c "$(echo {code} | base64 -d)" {args}'


def parse_scan_output(stdout: str) -> list[dict]:
    """Runs from ``remote_scan_command``'s stdout (raises ValueError on garbage)."""
    import json

    text = stdout.strip()
    start = text.find("{")  # login shells may print banners first
    if start < 0:
        raise ValueError(f"no JSON in scan output: {text[-200:]!r}")
    data = json.loads(text[start:])
    runs = data.get("runs")
    if not isinstance(runs, list):
        raise ValueError("scan output has no runs list")
    return runs


_NAME_DATE_PATTERNS = None


def name_date(name: str) -> float | None:
    """Epoch seconds (local midnight) of a date embedded in a run name, else None.

    Understands ``2026-10-18`` / ``2026_10_18``, ``20261018`` (YYYYMMDD), ``03192026``
    (MMDDYYYY) and ``2026-07`` (first of the month).
    """
    import datetime
    import re

    global _NAME_DATE_PATTERNS
    if _NAME_DATE_PATTERNS is None:
        _NAME_DATE_PATTERNS = [
            (re.compile(r"(?<!\d)(20\d\d)[-_.](\d\d)[-_.](\d\d)(?!\d)"), "ymd"),
            (re.compile(r"(?<!\d)(20\d\d)(\d\d)(\d\d)(?!\d)"), "ymd"),
            (re.compile(r"(?<!\d)(\d\d)(\d\d)(20\d\d)(?!\d)"), "mdy"),
            (re.compile(r"(?<!\d)(20\d\d)[-_](\d\d)(?![\d])"), "ym"),
        ]
    for pat, kind in _NAME_DATE_PATTERNS:
        for m in pat.finditer(name):
            g = [int(x) for x in m.groups()]
            if kind == "ymd":
                y, mo, d = g
            elif kind == "mdy":
                mo, d, y = g
            else:
                (y, mo), d = g, 1
            try:
                return datetime.datetime(y, mo, d).timestamp()
            except ValueError:
                continue
    return None


def human_size(n: int | None) -> str:
    """``du -sh`` style size (1024 based, one decimal under 10)."""
    if n is None:
        return "?"
    v, units = float(n), "BKMGT"
    i = 0
    while v >= 1024 and i < len(units) - 1:
        v /= 1024
        i += 1
    if i == 0:
        return f"{int(v)}B"
    return f"{v:.1f}{units[i]}" if v < 10 else f"{v:.0f}{units[i]}"


def finish_entry(entry: dict) -> dict:
    """Derived listing fields shared by local and remote runs."""
    entry["size"] = human_size(entry.get("size_bytes"))
    entry["name_date"] = name_date(entry.get("name", ""))
    entry["is_prior"] = is_prior_map(entry)
    return entry
