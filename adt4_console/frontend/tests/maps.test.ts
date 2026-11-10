import {afterEach, describe, expect, it, vi} from "vitest";
import {fuzzyMatch, matchRanges} from "../src/lib/fuzzy";
import {DEFAULT_PREFS, isIncomplete, loadPrefs, mapTime, savePrefs, searchMaps, sortMaps, type MapRow} from "../src/lib/mapList";
import type {MapEntry} from "../src/types";

const picked = (s: string, idx: number[]) => idx.map(i => s[i]).join("");

describe("fuzzyMatch", () => {
  it("matches subsequences case-insensitively and returns positions", () => {
    const s = "imgstore_full_20261018";
    const m = fuzzyMatch("ISF", s)!;
    expect(m).not.toBeNull();
    expect(picked(s, m.indices).toLowerCase()).toBe("isf");
    expect(fuzzyMatch("xyz", s)).toBeNull();
    expect(fuzzyMatch("", s)).toEqual({score: 0, indices: []});
  });

  it("prefers word boundaries and consecutive runs", () => {
    // "full" should land on the word, not scattered letters
    const s = "fleet_ugly_long_full_run";
    const m = fuzzyMatch("full", s)!;
    expect(m.indices).toEqual([16, 17, 18, 19]);
    // boundary-initial match beats a mid-word one
    const a = fuzzyMatch("cf", "camp_fleet")!.score;
    const b = fuzzyMatch("cf", "xxcxxfxx")!.score;
    expect(a).toBeGreaterThan(b);
    // exact substring at the start beats a later one
    expect(fuzzyMatch("bldg", "bldg45_spot")!.score).toBeGreaterThan(fuzzyMatch("bldg", "spot_bldg45")!.score);
  });

  it("tolerates one typo (dropped/extra or transposed char) for longer queries", () => {
    expect(fuzzyMatch("imgsotre", "imgstore_full")).not.toBeNull(); // transposition
    expect(fuzzyMatch("imgsxtore", "imgstore_full")).not.toBeNull(); // extra char
    expect(fuzzyMatch("bldq45", "bldg45_spot")).not.toBeNull(); // substitution = drop one
    const exact = fuzzyMatch("imgstore", "imgstore_full")!.score;
    const typo = fuzzyMatch("imgsotre", "imgstore_full")!.score;
    expect(exact).toBeGreaterThan(typo);
    // short queries stay strict
    expect(fuzzyMatch("xb", "bldg")).toBeNull();
    // two typos is too many
    expect(fuzzyMatch("imxsoxre", "imgstore")).toBeNull();
  });

  it("merges highlight ranges", () => {
    expect(matchRanges([5, 0, 1, 2, 7, 6])).toEqual([
      [0, 3],
      [5, 8],
    ]);
    expect(matchRanges([])).toEqual([]);
  });
});

const entry = (name: string, extra: Partial<MapEntry> = {}): MapEntry => ({name, size: "?", path: `~/adt4_output/${name}`, ...extra});

const rows: MapRow[] = [
  {machine: "hamilton", map: entry("nstf_test_fl_20261019", {mtime: 300, size_bytes: 68e6, image_store: {frames: {keyframe: 58}, frames_total: 58, observations: 0, sensors: 1, check: {ok: true, problems: []}}, has_image_store: true})},
  {machine: "willow", map: entry("building45_floor6", {mtime: 100, size_bytes: 653e6})},
  {machine: "fulu", map: entry("imgstore_full_20261018", {mtime: 200, size_bytes: 1e9, image_store: {frames: {keyframe: 1098}, frames_total: 1098, observations: 0, sensors: 1, check: {ok: true, problems: []}}, has_image_store: true})},
  {machine: "fulu", map: entry("03192026_afternoon", {mtime: null, name_date: 150, size_bytes: null, size_pending: true})},
  {machine: "willow", map: entry("no_date_run", {mtime: null, name_date: null})},
];

describe("searchMaps", () => {
  it("keeps every row for an empty query", () => {
    expect(searchMaps(rows, "  ")).toHaveLength(rows.length);
  });

  it("matches name, machine or path; all tokens must match", () => {
    const names = (q: string) => searchMaps(rows, q).map(r => r.map.name);
    expect(names("willow")).toEqual(["building45_floor6", "no_date_run"]);
    expect(names("fulu imgst")).toEqual(["imgstore_full_20261018"]);
    expect(names("b45 f6")).toEqual(["building45_floor6"]);
    expect(names("zzzz")).toEqual([]);
  });

  it("returns highlight positions per field", () => {
    const [hit] = searchMaps(rows, "willow b45f6");
    expect(hit.hits.machine).toBeDefined();
    expect(picked(hit.map.name, hit.hits.name!)).toBe("b45f6");
  });
});

describe("sortMaps", () => {
  const all = searchMaps(rows, "");
  const order = (k: Parameters<typeof sortMaps>[1], q = "") => sortMaps(q ? searchMaps(rows, q) : all, k, q).map(r => r.map.name);

  it("dates: mtime, falling back to a date in the name; unknown last", () => {
    expect(mapTime(rows[3].map)).toBe(150);
    expect(order("newest")).toEqual(["nstf_test_fl_20261019", "imgstore_full_20261018", "03192026_afternoon", "building45_floor6", "no_date_run"]);
    expect(order("oldest")).toEqual(["building45_floor6", "03192026_afternoon", "imgstore_full_20261018", "nstf_test_fl_20261019", "no_date_run"]);
  });

  it("size, name, machine, keyframes", () => {
    expect(order("size").slice(0, 3)).toEqual(["imgstore_full_20261018", "building45_floor6", "nstf_test_fl_20261019"]);
    expect(order("name")[0]).toBe("03192026_afternoon");
    expect(sortMaps(all, "machine").map(r => r.machine)).toEqual(["fulu", "fulu", "hamilton", "willow", "willow"]);
    expect(order("keyframes").slice(0, 2)).toEqual(["imgstore_full_20261018", "nstf_test_fl_20261019"]);
  });

  it("relevance orders by score while searching, else newest", () => {
    expect(order("relevance")).toEqual(order("newest"));
    expect(order("relevance", "full")[0]).toBe("imgstore_full_20261018");
  });
});

describe("prefs", () => {
  afterEach(() => {
    vi.restoreAllMocks();
    localStorage.clear();
  });

  it("round-trips and validates", () => {
    savePrefs({sort: "size", grouped: false, hideIncomplete: true});
    expect(loadPrefs()).toEqual({sort: "size", grouped: false, hideIncomplete: true});
    // prefs saved before the toggle existed keep showing everything
    localStorage.setItem("adt4.maps.prefs", JSON.stringify({sort: "size", grouped: false}));
    expect(loadPrefs()).toEqual({sort: "size", grouped: false, hideIncomplete: false});
    localStorage.setItem("adt4.maps.prefs", JSON.stringify({sort: "bogus", grouped: "x"}));
    expect(loadPrefs()).toEqual(DEFAULT_PREFS);
    localStorage.setItem("adt4.maps.prefs", "{not json");
    expect(loadPrefs()).toEqual(DEFAULT_PREFS);
  });

  it("survives blocked storage", () => {
    vi.spyOn(Storage.prototype, "getItem").mockImplementation(() => {
      throw new Error("blocked");
    });
    vi.spyOn(Storage.prototype, "setItem").mockImplementation(() => {
      throw new Error("blocked");
    });
    expect(loadPrefs()).toEqual(DEFAULT_PREFS);
    expect(() => savePrefs({sort: "name", grouped: true, hideIncomplete: false})).not.toThrow();
  });
});

describe("isIncomplete", () => {
  const m = (files: Partial<MapEntry>): MapEntry => ({name: "r", size: 1, ...files});
  it("needs all three hydra backend outputs", () => {
    expect(isIncomplete(m({has_dsg: true, has_dsg_mesh: true, has_mesh: true}))).toBe(false);
    expect(isIncomplete(m({has_dsg: true, has_dsg_mesh: false, has_mesh: true}))).toBe(true);
    expect(isIncomplete(m({has_dsg: false, has_dsg_mesh: false, has_mesh: false}))).toBe(true);
  });
  it("an older backend that does not report files is not hidden", () => {
    expect(isIncomplete(m({}))).toBe(false);
  });
});
