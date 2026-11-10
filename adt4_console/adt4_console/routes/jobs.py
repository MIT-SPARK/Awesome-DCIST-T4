"""Job inspection, confirmation and cancellation (API.md §4)."""

from __future__ import annotations

from fastapi import APIRouter, HTTPException, Request
from pydantic import BaseModel

from . import services

router = APIRouter(prefix="/api/jobs")


def _job(request: Request, job_id: str):
    job = services(request).jobs.get(job_id)
    if job is None:
        raise HTTPException(404, f"unknown job {job_id}")
    return job


@router.get("")
def list_jobs(request: Request):
    return services(request).jobs.list()


@router.get("/{job_id}")
def get_job(job_id: str, request: Request):
    return _job(request, job_id).to_dict()


class ConfirmBody(BaseModel):
    answer: bool
    text: str | None = None


@router.post("/{job_id}/confirm")
def confirm(job_id: str, body: ConfirmBody, request: Request):
    job = _job(request, job_id)
    if not job.answer(body.answer, body.text):
        raise HTTPException(409, "job has no pending confirmation")
    return {"ok": True}


@router.post("/{job_id}/cancel")
def cancel(job_id: str, request: Request):
    job = _job(request, job_id)
    job.cancel()
    return {"ok": True, "state": job.state}
