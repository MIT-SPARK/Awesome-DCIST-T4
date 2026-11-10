"""``/api/ws`` — binary msgpack pub/sub frames from the hub (API.md §2)."""

from __future__ import annotations

import asyncio
import contextlib

import msgpack
from fastapi import APIRouter, WebSocket, WebSocketDisconnect

router = APIRouter()


@router.websocket("/api/ws")
async def ws(websocket: WebSocket):
    hub = websocket.app.state.services.hub
    await websocket.accept()
    client = hub.add_client(asyncio.get_running_loop())
    hub.subscribe(client, None)

    async def reader():
        while True:
            msg = await websocket.receive()
            if msg["type"] == "websocket.disconnect":
                raise WebSocketDisconnect(msg.get("code", 1000))
            raw = msg.get("bytes")
            try:
                req = msgpack.unpackb(raw, raw=False) if raw is not None else None
                if req is None and msg.get("text"):
                    import json

                    req = json.loads(msg["text"])
            except Exception:  # noqa: BLE001
                continue
            if isinstance(req, dict) and "sub" in req:
                subs = req["sub"]
                hub.subscribe(client, list(subs) if subs is not None else None)

    async def writer():
        while True:
            for _topic, frame in await client.wait():
                await websocket.send_bytes(frame)

    tasks = [asyncio.create_task(reader()), asyncio.create_task(writer())]
    try:
        await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
    finally:
        for t in tasks:
            t.cancel()
        hub.remove_client(client)
        with contextlib.suppress(Exception):
            await websocket.close()
