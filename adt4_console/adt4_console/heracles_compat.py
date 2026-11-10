"""Optional heracles APIs.

The console runs against whichever heracles the workspace pins. Some of the
functions it uses for Neo4j maps (version counter, mesh path, map loading with
keyframes and image-store links) are newer than others; each is looked up here
so that a missing one turns its feature off with a single log line instead of
breaking the console. File (run-directory) maps never need heracles.
"""

from __future__ import annotations

import importlib
import logging

log = logging.getLogger(__name__)

_warned: set[str] = set()


def optional(module: str, name: str, feature: str):
    """``module.name`` or None (logged once per name: ``feature`` is off)."""
    try:
        return getattr(importlib.import_module(module), name)
    except (ImportError, AttributeError) as e:
        key = f"{module}.{name}"
        if key not in _warned:
            _warned.add(key)
            log.warning(
                "heracles has no %s (%s): %s is off; update heracles to enable it",
                key,
                e,
                feature,
            )
        return None
