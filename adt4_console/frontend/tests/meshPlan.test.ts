import {describe, expect, it} from "vitest";
import {autoDetail, levelBytes, loadDetail, saveDetail, manifestLevels, nextStep, planTile, resolveLevel, tileRow, wanted, type TileStep} from "../src/lib/meshPlan";
import type {TileManifest} from "../src/types";

const LODS = [
  {level: 0, cell: 0, name: "full"},
  {level: 1, cell: 0.2, name: "medium"},
  {level: 2, cell: 0.4, name: "coarse"},
];
const tile = (lod: ([number, string] | null)[]) => ({hash: "h", nv: 3, nf: 1, origin: [0, 0, 0] as [number, number, number], lod});
const M: TileManifest = {
  tile_size: 8,
  enc: 2,
  lods: LODS,
  tiles: {
    a: tile([[1000, "a0"], [300, "a1"], [100, "a2"]]),
    b: tile([[2000, "b0"], [500, "b1"], null]), // empty when coarse
  },
};
const LEGACY: TileManifest = {tile_size: 8, tiles: {a: {hash: "ha", nv: 3, nf: 1, origin: [0, 0, 0]}}};

describe("mesh levels of detail", () => {
  it("picks what auto means per page", () => {
    expect(autoDetail(true)).toBe("medium");
    expect(autoDetail(false)).toBe("full");
  });

  it("resolves a wanted level against what the manifest offers", () => {
    expect(manifestLevels(M)).toEqual([0, 1, 2]);
    expect(manifestLevels(LEGACY)).toEqual([0]);
    expect(resolveLevel([0, 1, 2], 1)).toBe(1);
    expect(resolveLevel([0], 2)).toBe(0); // a live source has the full mesh only
    expect(resolveLevel([0, 2], 1)).toBe(0); // never coarser than wanted when finer exists
    expect(resolveLevel([2], 0)).toBe(2);
  });

  it("reads tile rows (legacy manifests: the full tile, hash as etag)", () => {
    expect(tileRow(M, "a", 1)).toEqual([300, "a1"]);
    expect(tileRow(M, "b", 2)).toBeNull();
    expect(tileRow(M, "zz", 0)).toBeNull();
    expect(tileRow(LEGACY, "a", 0)).toEqual([0, "ha"]);
    expect(tileRow(LEGACY, "a", 2)).toBeNull();
  });

  it("plans a coarse preview, then the target, for a tile not shown yet", () => {
    expect(planTile(M, "a", 0, undefined).steps).toEqual([
      {key: "a", level: 2, bytes: 100, etag: "a2", preview: true},
      {key: "a", level: 0, bytes: 1000, etag: "a0", preview: false},
    ]);
    // no coarse level for b: straight to the target
    expect(planTile(M, "b", 0, undefined).steps.map(s => s.level)).toEqual([0]);
    // coarse wanted: one step, no preview
    expect(planTile(M, "a", 2, undefined).steps).toEqual([{key: "a", level: 2, bytes: 100, etag: "a2", preview: false}]);
  });

  it("plans nothing for a tile already shown, refetches a changed one, clears an emptied one", () => {
    expect(planTile(M, "a", 0, {level: 0, etag: "a0"})).toEqual({steps: [], clear: false});
    expect(planTile(M, "a", 0, {level: 0, etag: "old"}).steps).toEqual([{key: "a", level: 0, bytes: 1000, etag: "a0", preview: false}]);
    expect(planTile(M, "a", 0, {level: 2, etag: "a2"}).steps.map(s => s.level)).toEqual([0]); // refine, no preview
    expect(planTile(M, "b", 2, {level: 0, etag: "b0"})).toEqual({steps: [], clear: true});
    expect(planTile(M, "b", 2, undefined)).toEqual({steps: [], clear: false});
  });

  it("shows an arrival only while it is still wanted", () => {
    const prev: TileStep = {key: "a", level: 2, bytes: 100, etag: "a2", preview: true};
    const full: TileStep = {key: "a", level: 0, bytes: 1000, etag: "a0", preview: false};
    expect(wanted(M, prev, 0, undefined)).toBe(true);
    expect(wanted(M, prev, 0, {level: 0, etag: "a0"})).toBe(false); // late preview never covers the full tile
    expect(wanted(M, full, 0, {level: 2, etag: "a2"})).toBe(true);
    expect(wanted(M, full, 2, undefined)).toBe(false); // detail changed to coarse meanwhile
    expect(wanted(M, {...full, etag: "stale"}, 0, undefined)).toBe(false); // tile changed meanwhile
  });

  it("orders previews first, then nearest the camera target", () => {
    const d: Record<string, number> = {a: 50, b: 1, c: 9};
    const heads: TileStep[] = [
      {key: "a", level: 2, bytes: 1, etag: "", preview: true},
      {key: "b", level: 0, bytes: 1, etag: "", preview: false},
      {key: "c", level: 2, bytes: 1, etag: "", preview: true},
    ];
    expect(nextStep(heads, k => d[k])).toBe(2);
    expect(nextStep(heads.slice(1, 2), k => d[k])).toBe(0);
    expect(nextStep([], () => 0)).toBe(-1);
  });

  it("sums a level's download size for the detail picker", () => {
    expect(levelBytes(M, 0)).toBe(3000);
    expect(levelBytes(M, 1)).toBe(800);
    expect(levelBytes(M, 2)).toBe(100);
    expect(levelBytes(LEGACY, 0)).toBe(0);
    expect(levelBytes(null, 0)).toBe(0);
  });
});

describe("mesh detail preference", () => {
  it("is kept per device, auto when unset", () => {
    saveDetail("auto");
    expect(loadDetail()).toBe("auto");
    saveDetail("medium");
    expect(loadDetail()).toBe("medium");
    localStorage.setItem("adt4.meshDetail", "bogus");
    expect(loadDetail()).toBe("auto");
    saveDetail("auto");
    expect(localStorage.getItem("adt4.meshDetail")).toBeNull();
  });
});
