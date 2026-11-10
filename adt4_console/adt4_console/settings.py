"""Server configuration (built by ``cli.py`` or directly by tests)."""

from __future__ import annotations

import os
import pathlib
from dataclasses import dataclass, field

STATIC_DIR = pathlib.Path(__file__).resolve().parent / "static"


def _env(*names: str, default: str) -> str:
    for n in names:
        v = os.environ.get(n)
        if v:
            return v
    return default


@dataclass
class Settings:
    host: str = "0.0.0.0"
    port: int = 8090
    token: str | None = None
    output_root: str = "~/adt4_output"  # run directories listed on the Maps page
    neo4j: dict[str, str] = field(default_factory=dict)  # name -> uri
    # the --neo4j target maps are loaded into (map_loader); None: the first
    live_neo4j: str | None = None
    # heracles' variables, else the ADT4 launch ones (README: ADT4_NEO4J_*)
    neo4j_user: str = field(
        default_factory=lambda: _env(
            "HERACLES_NEO4J_USERNAME", "ADT4_NEO4J_USERNAME", default="neo4j"
        )
    )
    neo4j_password: str = field(
        default_factory=lambda: _env(
            "HERACLES_NEO4J_PASSWORD", "ADT4_NEO4J_PASSWORD", default="neo4j_pw"
        )
    )
    maps: dict[str, str] = field(default_factory=dict)  # name -> run dir
    static_dir: pathlib.Path = STATIC_DIR

    # background work (tests switch the poll off by making it slow)
    neo4j_poll_s: float = 2.0
    confirm_timeout_s: float = 300.0

    # geo (API.md §6)
    geo_anchors: dict[str, str] = field(
        default_factory=dict
    )  # src id/name ("*": all) -> json
    tiles_dir: str = "~/.adt4_console/tiles"
    tiles_live: bool = (
        False  # fetch missing satellite tiles from the internet (and keep them)
    )
    # packed mesh tiles + levels of detail cached per mesh file (mesh_lod.py);
    # None -> memory only (tests); cli: ~/.adt4_console/mesh_cache
    mesh_cache_dir: str | None = None

    @property
    def output_root_path(self) -> pathlib.Path:
        return pathlib.Path(os.path.expanduser(self.output_root))
