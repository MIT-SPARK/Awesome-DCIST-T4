import threading
import time

from adt4_console.hub import Hub
from adt4_console.jobs import JobManager


def _wait_for(pred, timeout=5.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if pred():
            return True
        time.sleep(0.01)
    return False


def _confirm_job(mgr, **confirm_kw):
    return mgr.start("t", "test", lambda job: job.confirm("proceed?", **confirm_kw))


def test_confirm_yes_no():
    mgr = JobManager()
    for answer in (True, False):
        job = _confirm_job(mgr)
        assert _wait_for(lambda: job.state == "waiting")
        assert job.to_dict()["confirm"]["message"] == "proceed?"
        assert job.answer(answer)
        mgr.wait(job.id)
        assert job.result is answer
        assert job.confirm_request is None


def test_typed_confirm_requires_exact_text():
    mgr = JobManager()
    job = _confirm_job(mgr, required_text="wipe")
    assert _wait_for(lambda: job.state == "waiting")
    assert job.answer(True, "WIPE")  # wrong text counts as "no"
    mgr.wait(job.id)
    assert job.result is False

    job = _confirm_job(mgr, required_text="wipe")
    assert _wait_for(lambda: job.state == "waiting")
    assert job.answer(True, "wipe")
    mgr.wait(job.id)
    assert job.result is True


def test_confirm_timeout_returns_none():
    mgr = JobManager(confirm_timeout=0.1)
    job = _confirm_job(mgr)
    mgr.wait(job.id)
    assert job.result is None
    assert job.state == "succeeded"
    assert any("timed out" in line["text"] for line in job.log_lines)


def test_cancel_unblocks_confirm_and_marks_cancelled():
    mgr = JobManager()
    seen = []

    def fn(job):
        seen.append(job.confirm("go?"))
        job.check_cancelled()  # raises JobCancelled
        seen.append("not reached")

    job = mgr.start("t", "test", fn)
    assert _wait_for(lambda: job.state == "waiting")
    job.cancel()
    mgr.wait(job.id)
    assert seen == [None]
    assert job.state == "cancelled"
    assert not job.answer(True)  # nothing pending any more


def test_failure_and_false_result_mark_failed():
    mgr = JobManager()
    j1 = mgr.start("t", "boom", lambda job: 1 / 0)
    j2 = mgr.start("t", "false", lambda job: False)
    assert mgr.wait(j1.id).state == "failed"
    assert any("ZeroDivisionError" in line["text"] for line in j1.log_lines)
    assert mgr.wait(j2.id).state == "failed"


def test_job_changes_are_published():
    hub = Hub()
    client = hub.add_client()
    hub.subscribe(client, ["job/", "jobs"])
    mgr = JobManager(hub)
    gate = threading.Event()

    def fn(job):
        job.log("info", "hello")
        gate.wait(2)
        return {"ok": 1}

    job = mgr.start("t", "pub", fn)
    gate.set()
    mgr.wait(job.id)
    topics = {t for t, _ in client.queue}
    assert {f"job/{job.id}", "jobs"} <= topics
    final = hub.latest(f"job/{job.id}")
    assert final["state"] == "succeeded" and final["result"] == {"ok": 1}
    assert [line["text"] for line in final["log"]] == ["hello"]


def test_confirm_route(client):
    svc = client.app.state.services
    job = _confirm_job(svc.jobs, required_text="wipe")
    assert _wait_for(lambda: job.state == "waiting")
    r = client.get(f"/api/jobs/{job.id}")
    assert r.json()["confirm"]["required_text"] == "wipe"
    assert (
        client.post(
            f"/api/jobs/{job.id}/confirm", json={"answer": True, "text": "wipe"}
        ).status_code
        == 200
    )
    svc.jobs.wait(job.id)
    assert job.result is True
    # nothing pending now
    r = client.post(f"/api/jobs/{job.id}/confirm", json={"answer": True})
    assert r.status_code == 409
    assert client.get("/api/jobs/nope").status_code == 404
    assert any(
        j["id"] == job.id and "log" not in j for j in client.get("/api/jobs").json()
    )
