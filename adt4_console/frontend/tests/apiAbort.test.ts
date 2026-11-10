import {afterEach, describe, expect, it, vi} from "vitest";
import {getJSON, onBackendState} from "../src/lib/api";

describe("backend state", () => {
  afterEach(() => vi.unstubAllGlobals());

  it("a cancelled request does not mark the server unreachable", async () => {
    vi.stubGlobal("fetch", vi.fn(async () => {
      throw new DOMException("The operation was aborted.", "AbortError");
    }));
    const seen: boolean[] = [];
    const off = onBackendState(ok => seen.push(ok));
    const ctl = new AbortController();
    ctl.abort();
    await expect(getJSON("api/x", ctl.signal)).rejects.toThrow();
    off();
    expect(seen).toEqual([]);
  });

  it("a network failure does", async () => {
    vi.stubGlobal("fetch", vi.fn(async () => {
      throw new TypeError("Failed to fetch");
    }));
    const seen: boolean[] = [];
    const off = onBackendState(ok => seen.push(ok));
    await expect(getJSON("api/x")).rejects.toThrow("Backend unreachable");
    off();
    expect(seen).toEqual([false]);
  });
});
