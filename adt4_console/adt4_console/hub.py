"""WebSocket pub/sub hub.

Frames are binary msgpack ``{topic, seq, data}``; ``seq`` counts per topic so a
client can detect gaps (e.g. dropped graph deltas → refetch the snapshot).

``Hub.publish`` is thread-safe: job workers and the graph poller publish from
their own threads.  Each client owns a bounded queue; when it is full the
*oldest frame of the same topic* is dropped (falling back to the oldest frame
overall), and frames of latest-value topics replace any queued frame of that
topic instead of piling up.
"""

from __future__ import annotations

import asyncio
import collections
import itertools
import threading

import msgpack

# Topics whose frames carry the full current value: only the newest matters.
LATEST_VALUE_PREFIXES = ("jobs", "job/")
LATEST_VALUE_SUFFIXES = ("/version",)


def is_latest_value(topic: str) -> bool:
    return topic.startswith(LATEST_VALUE_PREFIXES) or topic.endswith(
        LATEST_VALUE_SUFFIXES
    )


def default_match(topic: str) -> bool:
    """Default subscription: everything except ``graph/*/delta``."""
    return not (topic.startswith("graph/") and topic.endswith("/delta"))


def prefixes_match(prefixes: list[str] | None, topic: str) -> bool:
    if prefixes is None:
        return default_match(topic)
    return any(topic.startswith(p) for p in prefixes)


def encode_frame(topic: str, seq: int, data) -> bytes:
    return msgpack.packb({"topic": topic, "seq": seq, "data": data}, use_bin_type=True)


class Client:
    """One WebSocket connection's outbound queue (sync core, async wakeup)."""

    _ids = itertools.count(1)

    def __init__(self, maxlen: int = 256, loop=None):
        self.id = next(self._ids)
        self.maxlen = maxlen
        self.prefixes: list[str] | None = None  # None -> default_match
        self.queue: collections.deque[tuple[str, bytes]] = collections.deque()
        self.dropped = 0
        self._lock = threading.Lock()
        self._loop = loop
        self._event = asyncio.Event() if loop is not None else None

    def wants(self, topic: str) -> bool:
        return prefixes_match(self.prefixes, topic)

    def push(self, topic: str, frame: bytes) -> None:
        with self._lock:
            if is_latest_value(topic):
                for i, (t, _) in enumerate(self.queue):
                    if t == topic:
                        del self.queue[i]
                        break
            if len(self.queue) >= self.maxlen:
                victim = next(
                    (i for i, (t, _) in enumerate(self.queue) if t == topic), 0
                )
                del self.queue[victim]
                self.dropped += 1
            self.queue.append((topic, frame))
        self._wake()

    def pop_all(self) -> list[tuple[str, bytes]]:
        with self._lock:
            out = list(self.queue)
            self.queue.clear()
            return out

    def _wake(self) -> None:
        if self._event is None or self._loop is None:
            return
        try:
            self._loop.call_soon_threadsafe(self._event.set)
        except RuntimeError:  # loop closed
            pass

    async def wait(self) -> list[tuple[str, bytes]]:
        while True:
            frames = self.pop_all()
            if frames:
                return frames
            self._event.clear()
            frames = self.pop_all()
            if frames:
                return frames
            await self._event.wait()


class Hub:
    def __init__(self, client_queue: int = 256):
        self.client_queue = client_queue
        self._clients: dict[int, Client] = {}
        self._seq: dict[str, int] = collections.defaultdict(int)
        self._latest: dict[str, bytes] = {}
        self._lock = threading.Lock()

    def add_client(self, loop=None) -> Client:
        client = Client(self.client_queue, loop)
        with self._lock:
            self._clients[client.id] = client
        return client

    def remove_client(self, client: Client) -> None:
        with self._lock:
            self._clients.pop(client.id, None)

    def subscribe(self, client: Client, prefixes: list[str] | None) -> None:
        """Set ``client``'s topic prefixes and replay cached latest values."""
        client.prefixes = list(prefixes) if prefixes is not None else None
        with self._lock:
            cached = list(self._latest.items())
        for topic, frame in cached:
            if client.wants(topic):
                client.push(topic, frame)

    def publish(self, topic: str, data) -> int:
        with self._lock:
            self._seq[topic] += 1
            seq = self._seq[topic]
            clients = list(self._clients.values())
        frame = encode_frame(topic, seq, data)
        if is_latest_value(topic):
            with self._lock:
                self._latest[topic] = frame
        for c in clients:
            if c.wants(topic):
                c.push(topic, frame)
        return seq

    def latest(self, topic: str):
        with self._lock:
            frame = self._latest.get(topic)
        return msgpack.unpackb(frame, raw=False)["data"] if frame else None

    @property
    def n_clients(self) -> int:
        return len(self._clients)
