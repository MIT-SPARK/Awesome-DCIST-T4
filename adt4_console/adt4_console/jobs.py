"""Background jobs with logs, progress and blocking user confirmations.

A job runs ``fn(job)`` on a worker thread.  ``job.confirm(...)`` publishes a
confirmation request and blocks the worker until ``POST /api/jobs/{id}/confirm``
answers it, the job is cancelled, or the timeout expires (→ ``None``).
"""

from __future__ import annotations

import itertools
import threading
import time
import traceback
from dataclasses import asdict, is_dataclass

MAX_LOG = 2000
LEVELS = ("heading", "info", "dim", "ok", "warn", "error")


class JobCancelled(Exception):
    pass


class Job:
    def __init__(self, manager: JobManager, job_id: str, kind: str, title: str):
        self._mgr = manager
        self.id = job_id
        self.kind = kind
        self.title = title
        self.state = "running"
        self.created = time.time()
        self.finished: float | None = None
        self.progress: dict[str, float] = {}
        self.log_lines: list[dict] = []
        self.confirm_request: dict | None = None
        self.result = None
        self.cancelled = threading.Event()
        self.done = threading.Event()  # worker thread finished
        self._answer = None
        self._answered = threading.Event()
        self._lock = threading.RLock()
        self._last_progress_pub = 0.0

    # -- worker-side API ---------------------------------------------------
    def log(self, level: str, text: str) -> None:
        level = level if level in LEVELS else "info"
        with self._lock:
            self.log_lines.append({"t": time.time(), "level": level, "text": str(text)})
            if len(self.log_lines) > MAX_LOG:
                del self.log_lines[: len(self.log_lines) - MAX_LOG]
        self._changed()

    def set_progress(self, key: str, pct: float) -> None:
        now = time.time()
        with self._lock:
            self.progress[str(key)] = float(pct)
            # rsync reports progress many times a second: throttle broadcasts
            if pct < 100 and now - self._last_progress_pub < 0.25:
                return
            self._last_progress_pub = now
        self._changed()

    def check_cancelled(self) -> None:
        if self.cancelled.is_set():
            raise JobCancelled()

    def confirm(
        self,
        message: str,
        *,
        kind: str = "confirm",
        required_text: str | None = None,
        yes_label: str = "Yes",
        no_label: str = "No",
        detail: dict | None = None,
        timeout: float | None = None,
    ) -> bool | None:
        """Block until the user answers. True/False, or None on timeout/cancel."""
        if self.cancelled.is_set():
            return None
        timeout = self._mgr.confirm_timeout if timeout is None else timeout
        with self._lock:
            self._answered.clear()
            self._answer = None
            self.confirm_request = {
                "kind": kind,
                "message": message,
                "required_text": required_text,
                "yes_label": yes_label,
                "no_label": no_label,
                "detail": detail or {},
            }
            self.state = "waiting"
        self._changed()
        answered = self._answered.wait(timeout)
        with self._lock:
            answer = self._answer if answered else None
            self.confirm_request = None
            if self.state == "waiting":
                self.state = "running"
        if not answered:
            self.log("warn", "Confirmation timed out.")
        self._changed()
        return answer

    # -- API side ----------------------------------------------------------
    def answer(self, answer: bool, text: str | None = None) -> bool:
        """Answer the pending confirm; returns False if none is pending."""
        with self._lock:
            req = self.confirm_request
            if req is None or self._answered.is_set():
                return False
            if req.get("required_text"):
                answer = bool(answer) and text == req["required_text"]
            self._answer = bool(answer)
            self._answered.set()
        return True

    def cancel(self) -> None:
        self.cancelled.set()
        with self._lock:
            if self.state in ("running", "waiting"):
                self.state = "cancelled"
                self.finished = time.time()
            self._answer = None
            self._answered.set()
        self._changed()

    # -- serialisation -----------------------------------------------------
    def summary(self) -> dict:
        with self._lock:
            return {
                "id": self.id,
                "kind": self.kind,
                "title": self.title,
                "state": self.state,
                "created": self.created,
                "finished": self.finished,
                "progress": dict(self.progress),
                "confirm": dict(self.confirm_request) if self.confirm_request else None,
            }

    def to_dict(self) -> dict:
        d = self.summary()
        with self._lock:
            d["log"] = list(self.log_lines)
            d["result"] = _jsonable(self.result)
        return d

    def _changed(self) -> None:
        self._mgr._changed(self)


def _jsonable(obj):
    if is_dataclass(obj):
        return asdict(obj)
    if isinstance(obj, dict):
        return {str(k): _jsonable(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple, set)):
        return [_jsonable(v) for v in obj]
    if obj is None or isinstance(obj, (str, int, float, bool)):
        return obj
    return str(obj)


class JobManager:
    def __init__(self, hub=None, confirm_timeout: float = 300.0, keep: int = 200):
        self.hub = hub
        self.confirm_timeout = confirm_timeout
        self.keep = keep
        self._jobs: dict[str, Job] = {}
        self._ids = itertools.count(1)
        self._lock = threading.Lock()

    def start(self, kind: str, title: str, fn) -> Job:
        """Run ``fn(job)`` on a daemon thread; its return value is the result.

        ``fn`` returning ``False`` marks the job failed (``run_launch`` style);
        raising marks it failed with the traceback logged.
        """
        job = Job(self, f"{kind}-{next(self._ids)}-{int(time.time())}", kind, title)
        with self._lock:
            self._jobs[job.id] = job
            self._trim()
        self._changed(job)

        def run():
            state = "succeeded"
            try:
                result = fn(job)
                job.result = result
                if result is False:
                    state = "failed"
            except JobCancelled:
                state = "cancelled"
            except Exception as e:  # noqa: BLE001
                job.log("error", f"{type(e).__name__}: {e}")
                job.log("dim", traceback.format_exc()[-2000:])
                state = "failed"
            with job._lock:
                if job.state != "cancelled":
                    job.state = state
                if job.finished is None:
                    job.finished = time.time()
            job.done.set()
            job._changed()

        threading.Thread(target=run, name=f"job-{job.id}", daemon=True).start()
        return job

    def get(self, job_id: str) -> Job | None:
        return self._jobs.get(job_id)

    def list(self) -> list[dict]:
        with self._lock:
            jobs = list(self._jobs.values())
        return [j.summary() for j in sorted(jobs, key=lambda j: j.created)]

    def wait(self, job_id: str, timeout: float = 10.0) -> Job:
        """Block until the job's worker thread has finished (or ``timeout``)."""
        job = self._jobs[job_id]
        job.done.wait(timeout)
        return job

    def _trim(self) -> None:
        done = [j for j in self._jobs.values() if j.finished is not None]
        for j in sorted(done, key=lambda j: j.created)[
            : max(0, len(self._jobs) - self.keep)
        ]:
            self._jobs.pop(j.id, None)

    def _changed(self, job: Job) -> None:
        if self.hub is None:
            return
        self.hub.publish(f"job/{job.id}", job.to_dict())
        self.hub.publish("jobs", self.list())
