"""``adt4-console`` command line (API.md §0)."""

from __future__ import annotations

import argparse
import logging
import os


def _pairs(values, flag) -> dict[str, str]:
    out = {}
    for v in values or []:
        if "=" not in v:
            raise SystemExit(f"{flag} expects NAME=VALUE, got {v!r}")
        k, val = v.split("=", 1)
        out[k.strip()] = val.strip()
    return out


def parse_args(argv=None):
    p = argparse.ArgumentParser(
        prog="adt4-console", description="ADT4 web console: scene graph / map viewer"
    )
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", type=int, default=8090)
    p.add_argument("--token", default=os.environ.get("ADT4_CONSOLE_TOKEN"),
                   help="bearer token for /api (default: $ADT4_CONSOLE_TOKEN)")  # fmt: skip
    p.add_argument("--neo4j", action="append", metavar="NAME=URI",
                   help="Neo4j (heracles) graph source, e.g. live=bolt://localhost:7687 "
                   "(credentials: $HERACLES_NEO4J_USERNAME / $HERACLES_NEO4J_PASSWORD, else "
                   "$ADT4_NEO4J_USERNAME / $ADT4_NEO4J_PASSWORD, else neo4j / neo4j_pw)")  # fmt: skip
    p.add_argument("--live-neo4j", metavar="NAME",
                   help="the --neo4j target that maps are loaded into from the Maps page "
                   "(default: the first --neo4j)")  # fmt: skip
    p.add_argument("--map", action="append", metavar="NAME=RUN_DIR",
                   help="file graph source: a run dir containing hydra/backend/dsg.json")  # fmt: skip
    p.add_argument("--output-root", default="~/adt4_output",
                   help="where the Maps page looks for run directories")  # fmt: skip
    g = p.add_argument_group("satellite underlay")
    g.add_argument("--geo-anchor", action="append", metavar="SRC=JSON",
                   help="map→UTM anchor JSON file for a source; SRC '*' = every "
                   "source (default: <run>/geo_anchor.json)")  # fmt: skip
    g.add_argument("--tiles-dir", default="~/.adt4_console/tiles",
                   help="satellite tile cache")  # fmt: skip
    g.add_argument("--tiles-live", action="store_true",
                   help="fetch missing satellite tiles from the internet (cached for offline use)")  # fmt: skip
    p.add_argument("--log-level", default="info")
    return p.parse_args(argv)


def settings_from_args(args):
    from .settings import Settings

    neo4j = _pairs(args.neo4j, "--neo4j")
    if args.live_neo4j and args.live_neo4j not in neo4j:
        raise SystemExit(f"--live-neo4j {args.live_neo4j} is not a --neo4j target")
    return Settings(
        host=args.host,
        port=args.port,
        token=args.token or None,
        output_root=args.output_root,
        neo4j=neo4j,
        live_neo4j=args.live_neo4j,
        maps=_pairs(args.map, "--map"),
        geo_anchors=_pairs(args.geo_anchor, "--geo-anchor"),
        tiles_dir=args.tiles_dir,
        tiles_live=args.tiles_live,
        mesh_cache_dir="~/.adt4_console/mesh_cache",
    )


def main(argv=None):
    args = parse_args(argv)
    logging.basicConfig(
        level=args.log_level.upper(),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    settings = settings_from_args(args)

    import uvicorn

    from .app import create_app

    app = create_app(settings)
    uvicorn.run(app, host=settings.host, port=settings.port, log_level=args.log_level)


if __name__ == "__main__":
    main()
