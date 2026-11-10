"""FastAPI application factory.

``create_app(settings, services=None)`` — tests inject a :class:`Services` built
with ``build_services(settings, tile_fetch=...)``.
"""

from __future__ import annotations

import contextlib
import logging
import pathlib
from dataclasses import dataclass

from fastapi import FastAPI, Request
from fastapi.middleware.gzip import GZipMiddleware
from fastapi.responses import FileResponse, HTMLResponse, JSONResponse

from . import __version__
from .auth import TokenAuthMiddleware
from .geo import GeoService
from .graph_service import GraphService, GraphUnavailable, NotFound
from .hub import Hub
from .jobs import JobManager
from .map_loader import MapLoader
from .settings import Settings

log = logging.getLogger(__name__)


@dataclass
class Services:
    settings: Settings
    hub: Hub
    jobs: JobManager
    graph: GraphService
    geo: GeoService
    maps: MapLoader  # the live map and switching it

    def start(self) -> None:
        self.graph.start()
        self.graph.warm()

    def stop(self) -> None:
        try:
            self.graph.stop()
        except Exception:  # noqa: BLE001
            log.exception("stopping the graph service")


def build_services(settings: Settings, *, tile_fetch=None) -> Services:
    hub = Hub()
    jobs = JobManager(hub, confirm_timeout=settings.confirm_timeout_s)
    graph = GraphService(settings, hub)
    geo = GeoService(settings, graph, fetch=tile_fetch)
    maps = MapLoader(settings, graph, jobs, hub, geo)
    return Services(settings, hub, jobs, graph, geo, maps)


PLACEHOLDER = """<!doctype html><html><head><meta charset="utf-8"><title>ADT4 Console</title>
<style>body{font-family:system-ui,sans-serif;background:#111;color:#ddd;margin:3rem}
code{color:#8cf}</style></head><body><h1>ADT4 Console</h1>
<p>The web UI is not built yet (<code>adt4_console/adt4_console/static/index.html</code> is missing):
install Node 22 and run <code>npm ci &amp;&amp; npm run build</code> in <code>adt4_console/frontend</code>.</p>
<p>The API is up: try <a href="/api/health">/api/health</a> (add <code>?token=…</code> if a token is set).</p>
</body></html>"""


def create_app(settings: Settings, services: Services | None = None) -> FastAPI:
    services = services or build_services(settings)

    @contextlib.asynccontextmanager
    async def lifespan(app):
        services.start()
        try:
            yield
        finally:
            services.stop()

    app = FastAPI(title="ADT4 Console", version=__version__, lifespan=lifespan)
    app.state.services = services

    @app.exception_handler(NotFound)
    async def _not_found(request: Request, exc: NotFound):
        return JSONResponse({"detail": str(exc)}, status_code=404)

    @app.exception_handler(GraphUnavailable)
    async def _graph_unavailable(request: Request, exc: GraphUnavailable):
        return JSONResponse({"detail": str(exc)}, status_code=503)

    @app.get("/api/health")
    def health():
        """Server up + which Neo4j sources answer ({name: bool})."""
        neo4j = {}
        for s in list(services.graph.sources.values()):
            if s.kind == "neo4j":
                try:
                    s.count_nodes()
                    neo4j[s.name] = True
                except Exception:  # noqa: BLE001
                    neo4j[s.name] = False
        return {"ok": True, "version": __version__, "neo4j": neo4j}

    from .routes import geo, graph, jobs, maps, ws

    for mod in (jobs, maps, graph, geo, ws):
        app.include_router(mod.router)

    _mount_static(app, pathlib.Path(settings.static_dir))

    app.add_middleware(GZipMiddleware, minimum_size=1024)
    app.add_middleware(TokenAuthMiddleware, token=settings.token)
    return app


def _mount_static(app: FastAPI, static_dir: pathlib.Path) -> None:
    """Serve the built SPA with an index.html fallback for client-side routes."""
    root = static_dir.resolve()

    @app.get("/{path:path}", include_in_schema=False)
    async def spa(path: str):
        if path == "api" or path.startswith("api/"):
            return JSONResponse({"detail": "Not Found"}, status_code=404)
        index = root / "index.html"
        if path:
            target = (root / path).resolve()
            if target.is_file() and root in target.parents:
                headers = (
                    {"Cache-Control": "public, max-age=31536000, immutable"}
                    if path.startswith("assets/")
                    else {}
                )
                # not in every Python's mimetypes table
                media = {".webmanifest": "application/manifest+json"}.get(target.suffix)
                return FileResponse(target, headers=headers, media_type=media)
        if index.is_file():
            return FileResponse(index, headers={"Cache-Control": "no-cache"})
        return HTMLResponse(PLACEHOLDER)
