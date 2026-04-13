"""Shared mutable state passed to all TUI screens.

Replaces closure captures from the original monolithic _launch_tui().
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass
class TuiContext:
    """Carries all mutable state shared between TUI screens."""

    topo: dict
    active_network: str
    topology_path: str | None
    output_root: str
    runtime_config: dict[str, dict] = field(default_factory=dict)
    _deleted_machines: set[str] = field(default_factory=set)
    _local_ips: set[str] = field(default_factory=set)
    _fleet_zenoh: dict[str, Any] = field(
        default_factory=lambda: {"proc": None, "config": None}
    )

    @staticmethod
    def fuzzy_match(query: str, text: str) -> bool:
        """Subsequence fuzzy match — all chars of query appear in order in text."""
        if not query:
            return True
        query = query.lower()
        text = text.lower()
        qi = 0
        for ch in text:
            if ch == query[qi]:
                qi += 1
                if qi == len(query):
                    return True
        return False
