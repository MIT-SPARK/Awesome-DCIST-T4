import {useEffect, useMemo, useRef, useState} from "react";
import {fetchJob, isActive, useJobs} from "../stores/jobs";
import {errorText, postJSON} from "../lib/api";
import {canAnswerYes, confirmBody} from "../lib/confirm";
import {fmtTime} from "../lib/fuzzy";
import type {Job, JobConfirm, JobState} from "../types";
import {Badge, Empty, Modal, ProgressBar, Spinner, type Tone} from "./ui";

export const stateTone = (s: JobState): Tone =>
  s === "succeeded" ? "ok" : s === "failed" ? "err" : s === "waiting" ? "warn" : s === "running" ? "info" : "off";

export function JobStateBadge({state}: {state: JobState}) {
  return <Badge tone={stateTone(state)}>{state}</Badge>;
}

function JobLog({job}: {job: Job}) {
  const ref = useRef<HTMLDivElement>(null);
  const stick = useRef(true);
  const n = job.log?.length ?? 0;
  useEffect(() => {
    const el = ref.current;
    if (el && stick.current) el.scrollTop = el.scrollHeight;
  }, [n]);
  return (
    <div
      className="joblog mono"
      ref={ref}
      onScroll={e => {
        const el = e.currentTarget;
        stick.current = el.scrollHeight - el.scrollTop - el.clientHeight < 24;
      }}
    >
      {n === 0 && <div className="log-dim">No output yet.</div>}
      {job.log?.map((l, i) => (
        <div key={i} className={`log-line log-${l.level}`}>
          <span className="log-t">{fmtTime(l.t)}</span>
          <span className="log-text">{l.text}</span>
        </div>
      ))}
    </div>
  );
}

export function JobDetail({job}: {job: Job}) {
  const [busy, setBusy] = useState(false);
  const [err, setErr] = useState<string | null>(null);
  useEffect(() => {
    if (!job.log) void fetchJob(job.id);
  }, [job.id, job.log]);
  const progress = Object.entries(job.progress ?? {});
  return (
    <div className="jobdetail">
      <div className="jobdetail-head">
        <div>
          <div className="job-title">{job.title || job.kind}</div>
          <div className="muted small mono">
            {job.kind} · {job.id} · started {fmtTime(job.created)}
            {job.finished ? ` · finished ${fmtTime(job.finished)}` : ""}
          </div>
        </div>
        <div className="row gap">
          <JobStateBadge state={job.state} />
          {isActive(job) && (
            <button
              className="btn btn-sm btn-danger-outline"
              disabled={busy}
              onClick={async () => {
                setBusy(true);
                try {
                  await postJSON(`api/jobs/${job.id}/cancel`);
                } catch (e) {
                  setErr(errorText(e));
                } finally {
                  setBusy(false);
                }
              }}
            >
              Cancel
            </button>
          )}
        </div>
      </div>
      {err && <div className="inline-err">{err}</div>}
      {progress.length > 0 && (
        <div className="progress-list">
          {progress.map(([k, v]) => (
            <ProgressBar key={k} label={k} pct={v} tone={v >= 100 ? "ok" : "info"} />
          ))}
        </div>
      )}
      <JobLog job={job} />
      {job.result !== undefined && job.result !== null && !isActive(job) && <JobResult result={job.result} />}
    </div>
  );
}

function JobResult({result}: {result: unknown}) {
  return (
    <details className="job-result">
      <summary>Result</summary>
      <pre className="mono">{JSON.stringify(result, null, 2)}</pre>
    </details>
  );
}

export function JobsDrawer() {
  const {drawerOpen, close, order, jobs, focusId} = useJobs();
  const focus = focusId ? jobs[focusId] : null;
  if (!drawerOpen) return null;
  return (
    <aside className="drawer" aria-label="Jobs">
      <header className="drawer-head">
        <h2>Jobs</h2>
        <button className="icon-btn" onClick={close} aria-label="Close jobs">
          ×
        </button>
      </header>
      <div className="drawer-body">
        <ul className="joblist">
          {order.length === 0 && <li className="muted pad">No jobs yet. Launches, transfers and verifies show up here.</li>}
          {order.map(id => {
            const j = jobs[id];
            return (
              <li key={id} className={id === focusId ? "on" : ""} onClick={() => useJobs.setState({focusId: id})}>
                <div className="row gap between">
                  <span className="job-title-sm">{j.title || j.kind}</span>
                  {isActive(j) ? <Spinner /> : null}
                </div>
                <div className="row gap between">
                  <span className="muted small mono">{fmtTime(j.created)}</span>
                  <JobStateBadge state={j.state} />
                </div>
              </li>
            );
          })}
        </ul>
        <div className="drawer-detail">{focus ? <JobDetail job={focus} /> : <Empty title="Select a job" />}</div>
      </div>
    </aside>
  );
}

/** Global modal for any job waiting on a yes/no or typed confirmation. */
export function JobConfirmHost() {
  const jobs = useJobs(s => s.jobs);
  const [answered, setAnswered] = useState<Record<string, string>>({});
  const pending = useMemo(
    () => Object.values(jobs).filter(j => j.confirm && j.state === "waiting" && answered[j.id] !== j.confirm.message),
    [jobs, answered],
  );
  // `waiting` jobs from the summary list may lack the confirm payload: fetch them
  useEffect(() => {
    for (const j of Object.values(jobs)) if (j.state === "waiting" && !j.confirm) void fetchJob(j.id);
  }, [jobs]);
  const job = pending[0];
  if (!job || !job.confirm) return null;
  return (
    <ConfirmDialog
      key={job.id + job.confirm.message}
      job={job}
      confirm={job.confirm}
      onDone={() => setAnswered(a => ({...a, [job.id]: job.confirm!.message}))}
    />
  );
}

export function ConfirmDialog({job, confirm, onDone}: {job: Job; confirm: JobConfirm; onDone: () => void}) {
  const [typed, setTyped] = useState("");
  const [busy, setBusy] = useState(false);
  const [err, setErr] = useState<string | null>(null);
  const yesOk = canAnswerYes(confirm, typed);
  const answer = async (yes: boolean) => {
    setBusy(true);
    setErr(null);
    try {
      await postJSON(`api/jobs/${job.id}/confirm`, confirmBody(confirm, yes, typed));
      onDone();
    } catch (e) {
      setErr(errorText(e));
    } finally {
      setBusy(false);
    }
  };
  const danger = Boolean(confirm.required_text);
  return (
    <Modal
      title={<>Confirm: {job.title || job.kind}</>}
      tone={danger ? "err" : "warn"}
      footer={
        <>
          <button className="btn" disabled={busy} onClick={() => answer(false)}>
            {confirm.no_label || "No"}
          </button>
          <button className={`btn ${danger ? "btn-danger" : "btn-primary"}`} disabled={busy || !yesOk} onClick={() => answer(true)} data-testid="confirm-yes">
            {confirm.yes_label || "Yes"}
          </button>
        </>
      }
    >
      <p className="confirm-msg">{confirm.message}</p>
      {confirm.detail !== undefined && confirm.detail !== null && !(typeof confirm.detail === "object" && Object.keys(confirm.detail as object).length === 0) && (
        <pre className="confirm-detail mono">{typeof confirm.detail === "string" ? confirm.detail : JSON.stringify(confirm.detail, null, 2)}</pre>
      )}
      {confirm.required_text && (
        <label className="field">
          <span className="field-label">
            Type <code className="mono req-text">{confirm.required_text}</code> to confirm
          </span>
          <input
            className="input mono"
            autoFocus
            value={typed}
            onChange={e => setTyped(e.target.value)}
            onKeyDown={e => e.key === "Enter" && yesOk && !busy && answer(true)}
            aria-label="Confirmation text"
          />
        </label>
      )}
      {err && <div className="inline-err">{err}</div>}
    </Modal>
  );
}
