import {afterEach, describe, expect, it, vi} from "vitest";
import {RateMeter, fmtBytesPair, fmtEta, summarize, summaryText, wireProgress, type LoadTask} from "../src/lib/loadProgress";
import {beginTask, dropTask, endTask, progressTask, resetLoading, transfer, useLoading} from "../src/stores/loading";

const task = (p: Partial<LoadTask>): LoadTask => ({label: "Mesh", unit: "bytes", done: 0, total: null, state: "active", ...p});

describe("RateMeter", () => {
  it("measures a steady link and predicts the time left", () => {
    const m = new RateMeter();
    m.busy(0);
    for (let t = 0.5; t <= 10; t += 0.5) m.add(125_000, t); // 250 kB/s = 2 Mbit/s
    expect(m.rate!).toBeCloseTo(250_000, -2);
    expect(m.eta(5_000_000)!).toBeCloseTo(20, 0);
  });

  it("follows a change of speed with an EWMA (tau 3 s)", () => {
    const m = new RateMeter(3);
    m.busy(0);
    for (let t = 0.5; t <= 10; t += 0.5) m.add(50_000, t); // 100 kB/s
    for (let t = 10.5; t <= 13; t += 0.5) m.add(500_000, t); // then 1 MB/s for 3 s
    // one time constant in: ~63% of the way there
    expect(m.rate!).toBeGreaterThan(100_000 + 0.55 * 900_000);
    expect(m.rate!).toBeLessThan(100_000 + 0.72 * 900_000);
  });

  it("does not count idle gaps (server busy, nothing in flight)", () => {
    const m = new RateMeter();
    m.busy(0);
    for (let t = 0.5; t <= 4; t += 0.5) m.add(100_000, t); // 200 kB/s
    m.idle(4);
    m.busy(60); // a minute later
    m.add(100_000, 60.5);
    expect(m.rate!).toBeCloseTo(200_000, -3);
  });

  it("has no ETA until two seconds of transfer have been sampled", () => {
    const m = new RateMeter();
    expect(m.eta(1000)).toBeNull();
    m.busy(0);
    m.add(1000, 0.5);
    m.add(1000, 1.0);
    m.add(1000, 1.5);
    expect(m.eta(1000)).toBeNull();
    m.add(1000, 2.0);
    expect(m.eta(2000)).toBeCloseTo(1, 1);
    expect(m.eta(0)).toBe(0);
  });

  it("ignores samples shorter than its minimum interval", () => {
    const m = new RateMeter(3, 0.25);
    m.busy(0);
    m.add(1e6, 0.01); // a cached response landing at once
    expect(m.rate).toBeNull();
    m.add(0, 0.5);
    expect(m.rate!).toBeCloseTo(2e6, -3);
  });
});

describe("formatting", () => {
  it("formats ETAs coarsely as they grow", () => {
    expect(fmtEta(null)).toBe("");
    expect(fmtEta(0.3)).toBe("<1 s left");
    expect(fmtEta(4.2)).toBe("~5 s left");
    expect(fmtEta(26)).toBe("~25 s left");
    expect(fmtEta(150)).toBe("~3 min left");
    expect(fmtEta(3600 + 20 * 60)).toBe("~1 h 20 min left");
  });

  it("formats byte pairs in the total's unit", () => {
    expect(fmtBytesPair(12_400_000, 38_000_000)).toBe("12.4/38.0 MB");
    expect(fmtBytesPair(1_234_000, 5_750_000)).toBe("1.23/5.75 MB");
    expect(fmtBytesPair(74_000, 120_000)).toBe("74/120 kB");
    expect(fmtBytesPair(200_000, 100_000)).toBe("100/100 kB"); // clamps done to total
  });

  it("scales decompressed reads to wire bytes", () => {
    expect(wireProgress(500, 100, 1000, true)).toBe(50);
    expect(wireProgress(2000, 100, 1000, true)).toBe(100);
    expect(wireProgress(500, null, null, true)).toBe(500);
    expect(wireProgress(500, 1000, null, false)).toBe(500);
  });
});

describe("summarize", () => {
  it("says what is loading, bytes so far / total and the time left", () => {
    const m = new RateMeter();
    m.busy(0);
    for (let t = 0.5; t <= 4; t += 0.5) m.add(500_000, t); // 1 MB/s
    const s = summarize(
      [
        task({label: "Graph", done: 74_000, total: 74_000, state: "done"}),
        task({label: "", done: 96_000, total: 96_000, state: "done"}), // trajectory: bytes only
        task({done: 12_400_000, total: 38_000_000}),
      ],
      m,
    );
    expect(s.parts.map(p => `${p.label} ${p.text}`)).toEqual(["Graph ✓", "Mesh 12.4/38.0 MB"]);
    expect(s.bytesTotal).toBe(38_170_000);
    expect(s.fraction!).toBeCloseTo(12_570_000 / 38_170_000, 5);
    expect(s.eta!).toBeCloseTo(25.6, 0);
    expect(summaryText(s)).toBe("Graph ✓ · Mesh 12.4/38.0 MB · ~25 s left");
  });

  it("sums tasks with the same label (several sources)", () => {
    const s = summarize([task({done: 1e6, total: 2e6}), task({done: 0, total: 3e6})], null);
    expect(s.parts).toEqual([{label: "Mesh", text: "1.00/5.00 MB", done: false}]);
  });

  it("has no fraction or ETA while a total is unknown, and shows the note", () => {
    const m = new RateMeter();
    m.busy(0);
    m.add(1e6, 2);
    const s = summarize([task({label: "Graph", done: 1000, total: 1000, state: "done"}), task({note: "preparing"})], m);
    expect(s.parts[1].text).toBe("preparing");
    expect(s.fraction).toBeNull();
    expect(s.eta).toBeNull();
    expect(s.active).toBe(true);
  });

  it("counts imagery tiles, flags failures and is idle when all is done", () => {
    const s = summarize([task({label: "Imagery", unit: "count", done: 30, total: 42})], null);
    expect(s.parts[0].text).toBe("30/42");
    expect(s.fraction!).toBeCloseTo(30 / 42);
    const f = summarize([task({label: "Graph", state: "error"})], null);
    expect(f.parts[0].text).toBe("failed");
    expect(f.error && !f.active).toBe(true);
    expect(summarize([], null).active).toBe(false);
  });
});

describe("loading store", () => {
  afterEach(() => {
    resetLoading();
    vi.useRealTimers();
  });

  it("publishes tasks and keeps a finished batch until a new load starts", () => {
    resetLoading();
    beginTask("graph:a", "Graph");
    progressTask("graph:a", 10, 100);
    endTask("graph:a");
    expect(useLoading.getState().summary.parts).toEqual([{label: "Graph", text: "✓", done: true}]);
    expect(useLoading.getState().finishedAt).not.toBeNull();
    // joins the batch while it lingers
    beginTask("mesh:a", "Mesh", "bytes", null, "preparing");
    expect(useLoading.getState().summary.parts.map(p => p.label)).toEqual(["Graph", "Mesh"]);
    dropTask("mesh:a");
    expect(useLoading.getState().summary.active).toBe(false);
  });

  it("starts a new batch after the linger time", () => {
    resetLoading();
    const t0 = performance.now();
    const spy = vi.spyOn(performance, "now").mockReturnValue(t0);
    beginTask("graph:a", "Graph");
    endTask("graph:a");
    spy.mockReturnValue(t0 + 10_000);
    beginTask("graph:b", "Graph");
    expect(useLoading.getState().summary.parts).toEqual([{label: "Graph", text: "…", done: false}]);
    spy.mockRestore();
  });

  it("feeds transfers into one throughput meter", () => {
    resetLoading();
    const t0 = performance.now();
    const spy = vi.spyOn(performance, "now").mockReturnValue(t0);
    beginTask("mesh:a", "Mesh", "bytes", 4_000_000);
    transfer.start();
    for (let i = 1; i <= 12; i++) {
      spy.mockReturnValue(t0 + i * 250);
      transfer.bytes(250_000); // 1 MB/s
      progressTask("mesh:a", i * 250_000);
    }
    transfer.end();
    endTask("mesh:a"); // publishes now
    beginTask("mesh:b", "Mesh", "bytes", 3_000_000);
    expect(useLoading.getState().summary.eta!).toBeCloseTo(3, 0);
    spy.mockRestore();
  });
});

describe("summarize: order", () => {
  it("names Graph, Mesh, Imagery in that order whatever started first", () => {
    const s = summarize([task({label: "Imagery", unit: "count", done: 1, total: 2}), task({note: "preparing"}), task({label: "Graph", done: 1, total: 1, state: "done"})], null);
    expect(s.parts.map(p => p.label)).toEqual(["Graph", "Mesh", "Imagery"]);
  });
});
