"""Bearer-token auth for ``/api/*`` (HTTP and WebSocket); static files stay public."""

from __future__ import annotations

import hmac
import json
from urllib.parse import parse_qs


def extract_token(scope) -> str | None:
    for name, value in scope.get("headers") or []:
        if name == b"authorization":
            v = value.decode("latin-1")
            if v.lower().startswith("bearer "):
                return v[7:].strip()
    qs = parse_qs((scope.get("query_string") or b"").decode("latin-1"))
    return (qs.get("token") or [None])[0]


def token_ok(expected: str | None, given: str | None) -> bool:
    if not expected:
        return True
    return given is not None and hmac.compare_digest(expected, given)


class TokenAuthMiddleware:
    def __init__(self, app, token: str | None):
        self.app = app
        self.token = token

    async def __call__(self, scope, receive, send):
        typ = scope.get("type")
        path = scope.get("path", "")
        if (
            self.token
            and typ in ("http", "websocket")
            and (path == "/api" or path.startswith("/api/"))
            and not token_ok(self.token, extract_token(scope))
        ):
            if typ == "websocket":
                await send({"type": "websocket.close", "code": 4401})
                return
            body = json.dumps({"detail": "Not authenticated"}).encode()
            await send(
                {
                    "type": "http.response.start",
                    "status": 401,
                    "headers": [
                        (b"content-type", b"application/json"),
                        (b"www-authenticate", b"Bearer"),
                    ],
                }
            )
            await send({"type": "http.response.body", "body": body})
            return
        await self.app(scope, receive, send)
