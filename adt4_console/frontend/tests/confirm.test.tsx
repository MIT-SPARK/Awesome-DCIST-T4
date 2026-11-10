import {afterEach, describe, expect, it, vi} from "vitest";
import {act} from "react";
import {createRoot, type Root} from "react-dom/client";
import {canAnswerYes, confirmBody} from "../src/lib/confirm";
import {ConfirmDialog} from "../src/components/Jobs";
import type {Job, JobConfirm} from "../src/types";

(globalThis as unknown as {IS_REACT_ACT_ENVIRONMENT: boolean}).IS_REACT_ACT_ENVIRONMENT = true;

const typed: JobConfirm = {kind: "wipe", message: "Wipe Neo4j?", required_text: "wipe", yes_label: "Wipe", no_label: "Cancel"};
const plain: JobConfirm = {kind: "kill_sessions", message: "Kill sessions?", required_text: null};

describe("confirm logic", () => {
  it("plain yes/no is always answerable", () => {
    expect(canAnswerYes(plain, "")).toBe(true);
    expect(confirmBody(plain, true, "")).toEqual({answer: true});
  });
  it("typed confirm only enables yes on an exact match", () => {
    expect(canAnswerYes(typed, "")).toBe(false);
    expect(canAnswerYes(typed, "wip")).toBe(false);
    expect(canAnswerYes(typed, "WIPE")).toBe(false);
    expect(canAnswerYes(typed, "wipe ")).toBe(false);
    expect(canAnswerYes(typed, "wipe")).toBe(true);
    expect(confirmBody(typed, true, "wipe")).toEqual({answer: true, text: "wipe"});
    expect(confirmBody(typed, false, "wipe")).toEqual({answer: false});
    expect(canAnswerYes(null, "x")).toBe(false);
  });
});

describe("ConfirmDialog", () => {
  let root: Root | null = null;
  let host: HTMLDivElement | null = null;
  afterEach(() => {
    act(() => root?.unmount());
    host?.remove();
    vi.restoreAllMocks();
  });

  it("keeps Yes disabled until the required text is typed, then posts it", async () => {
    const fetchMock = vi.fn(async () => new Response("{}", {status: 200}));
    vi.stubGlobal("fetch", fetchMock);
    const job: Job = {id: "j1", kind: "ingest", title: "Load map", state: "waiting", created: 0, confirm: typed};
    const onDone = vi.fn();
    host = document.createElement("div");
    document.body.appendChild(host);
    root = createRoot(host);
    act(() => root!.render(<ConfirmDialog job={job} confirm={typed} onDone={onDone} />));
    const yes = document.querySelector('[data-testid="confirm-yes"]') as HTMLButtonElement;
    const input = document.querySelector('input[aria-label="Confirmation text"]') as HTMLInputElement;
    expect(yes.textContent).toBe("Wipe");
    expect(yes.disabled).toBe(true);
    const setValue = (v: string) => {
      const setter = Object.getOwnPropertyDescriptor(HTMLInputElement.prototype, "value")!.set!;
      setter.call(input, v);
      input.dispatchEvent(new Event("input", {bubbles: true}));
    };
    act(() => setValue("wip"));
    expect(yes.disabled).toBe(true);
    act(() => setValue("wipe"));
    expect(yes.disabled).toBe(false);
    await act(async () => yes.click());
    expect(fetchMock).toHaveBeenCalledTimes(1);
    const [url, init] = fetchMock.mock.calls[0] as unknown as [string, RequestInit];
    expect(url).toBe("api/jobs/j1/confirm");
    expect(JSON.parse(init.body as string)).toEqual({answer: true, text: "wipe"});
    expect(onDone).toHaveBeenCalled();
  });
});
