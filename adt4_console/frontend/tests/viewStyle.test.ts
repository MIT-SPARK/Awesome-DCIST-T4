import {beforeEach, describe, expect, it} from "vitest";
import type {NodeRecord} from "../src/codec";
import {GraphModel} from "../src/lib/graphModel";
import {bboxZOffset, displayPos, layoutGraph, regionParents} from "../src/viewer/layout";
import {
  contrastFor,
  contrastRatio,
  effectiveShot,
  heightScale,
  luminance,
  niceLength,
  overrideColor,
  placeFootprint,
  regionFootprint,
  shotFilename,
  DEFAULT_SHOT,
  STYLE_BACKGROUND,
} from "../src/viewer/style";
import {DEFAULT_VIEW_STYLE, parseStoredStyle, saveStyle, STYLE_STORAGE_KEY} from "../src/stores/viewStyle";
import {pathColors, splitPath, trajectoryPaths} from "../src/stores/viewer";

const rec = (sym: string, layer: number, part: number, pos: [number, number, number], label = -1, name = "", bbox: NodeRecord["bbox"] = null): NodeRecord => ({
  sym, layer, part, pos, label, bbox, color: [0, 0, 0], name, active: false,
});

const base = {hiddenLayers: {}, edgeKinds: {}, flat: false, stackHeight: 2, style: "hydra" as const};

describe("background contrast", () => {
  it("picks dark ink on white and light ink on dark", () => {
    const white = contrastFor("#ffffff");
    expect(white.light).toBe(true);
    expect(white.text).toBe("#111111");
    expect(contrastRatio(white.text, "#ffffff")).toBeGreaterThan(7);
    expect(contrastRatio(white.grid[0], "#ffffff")).toBeGreaterThan(1.8);
    const black = contrastFor("#000000");
    expect(black.light).toBe(false);
    expect(black.text).toBe("#ffffff");
    expect(contrastRatio(black.grid[0], "#000000")).toBeGreaterThan(1.8);
  });
  it("keeps the historical style grids and black hydra edges on RViz gray", () => {
    expect(contrastFor(STYLE_BACKGROUND.hydra).grid).toEqual(["#7a7a80", "#55555a"]);
    expect(contrastFor(STYLE_BACKGROUND.hydra).edge).toBe("#000000");
    expect(contrastFor("#ffffff").edge).toBe("#000000");
    // darker than RViz gray: black edges would vanish
    expect(contrastFor("#000000").edge).not.toBe("#000000");
  });
  it("luminance spans 0..1", () => {
    expect(luminance("#000")).toBe(0);
    expect(luminance("#ffffff")).toBeCloseTo(1);
  });
});

describe("colour overrides", () => {
  it("uniform, by label (custom + default) and default modes", () => {
    const opts = {rules: {objects: {mode: "label" as const, color: "#000000"}, places: {mode: "uniform" as const, color: "#ff0000"}}, labelColors: {chair: "#00ff00"}, placesByRegion: false};
    expect(overrideColor("places", opts, null)).toEqual([255, 0, 0]);
    expect(overrideColor("objects", opts, {id: 4, name: "chair"})).toEqual([0, 255, 0]);
    const d = overrideColor("objects", opts, {id: 5, name: "sign"});
    expect(d).toHaveLength(3);
    expect(overrideColor("rooms", opts, null)).toBeNull();
  });
  it("layout applies object, bbox and region-parent colours", () => {
    const m = GraphModel.fromSnapshot({
      nodes: [rec("O(1)", 2, 0, [0, 0, 0], 4, "", [-0.5, -0.5, -0.5, 0.5, 0.5, 0.5]), rec("P(1)", 3, 1, [1, 0, 0]), rec("P(2)", 3, 1, [9, 0, 0]), rec("r(1)", 4, 0, [1, 0, 0], 0, "road")],
      edges: [["r(1)", "P(1)"]],
      labelspaces: {"2/0": {"4": "chair"}},
    });
    const colors = {rules: {objects: {mode: "uniform" as const, color: "#0000ff"}, bboxes: {mode: "uniform" as const, color: "#ff0000"}}, labelColors: {}, placesByRegion: true};
    const l = layoutGraph(m, {...base, colors});
    const obj = l.layers.find(x => x.info.kind === "objects")!;
    expect(Array.from(obj.colors)).toEqual([0, 0, 1]);
    expect(l.bboxes.colors[0]).toBeCloseTo(1); // linear red
    expect(l.bboxes.colors[1]).toBe(0);
    const places = l.layers.find(x => x.info.kind === "places")!;
    const i1 = places.syms.indexOf("P(1)");
    const i2 = places.syms.indexOf("P(2)");
    // P(1) takes region r(1)'s colorbrewer colour (index 1: #1f78b4), P(2) keeps hydra black
    expect(Array.from(places.colors.slice(i1 * 3, i1 * 3 + 3)).map(v => Math.round(v * 255))).toEqual([31, 120, 180]);
    expect(Array.from(places.colors.slice(i2 * 3, i2 * 3 + 3))).toEqual([0, 0, 0]);
  });
});

describe("layer heights", () => {
  it("defaults follow hydra_visualizer.yaml and overrides win", () => {
    expect(heightScale("places", "hydra", {})).toBe(2);
    expect(heightScale("objects", "hydra", {})).toBe(1);
    expect(heightScale("rooms", "hydra", {})).toBe(3);
    expect(heightScale("agents", "hydra", {})).toBe(0);
    expect(heightScale("places", "classic", {})).toBe(1);
    expect(heightScale("places", "hydra", {places: 0.5})).toBe(0.5);
    // boxes follow object nodes until set on their own
    expect(heightScale("bboxes", "hydra", {objects: 4})).toBe(4);
    expect(heightScale("bboxes", "hydra", {objects: 4, bboxes: 0})).toBe(0);
  });
  it("displayPos and layout use the per-layer heights", () => {
    const n = rec("P(1)", 3, 1, [0, 0, 1]);
    expect(displayPos(n, false, 2, "hydra")[2]).toBe(5);
    expect(displayPos(n, false, 2, "hydra", {places: 0.5})[2]).toBe(2);
    expect(displayPos(n, true, 2, "hydra", {places: 3})[2]).toBe(1);
    const m = GraphModel.fromSnapshot({nodes: [rec("O(1)", 2, 0, [0, 0, 1], 1, "", [-1, -1, 0, 1, 1, 2])], edges: [], labelspaces: {}});
    const l = layoutGraph(m, {...base, heights: {bboxes: 0, objects: 3}});
    const obj = l.layers[0];
    expect(obj.positions[2]).toBe(1 + 6); // node floats 3 x 2 m
    expect(Math.min(...Array.from(l.bboxes.positions).filter((_, i) => i % 3 === 2))).toBe(0); // box at true height
    expect(obj.boxes![2]).toBe(0);
    expect(bboxZOffset(false, 2, "hydra", {})).toBe(2);
  });
});

describe("regions", () => {
  const sq = (x0: number, y0: number, x1: number, y1: number): [number, number][] => [[x0, y0], [x1, y0], [x1, y1], [x0, y1]];
  const area = (tris: number[]) => {
    let a = 0;
    for (let i = 0; i < tris.length; i += 6) a += Math.abs((tris[i + 2] - tris[i]) * (tris[i + 5] - tris[i + 1]) - (tris[i + 4] - tris[i]) * (tris[i + 3] - tris[i + 1])) / 2;
    return a;
  };
  const covers = (tris: number[], x: number, y: number) => {
    for (let i = 0; i < tris.length; i += 6) {
      const [ax, ay, bx, by, cx, cy] = tris.slice(i, i + 6);
      const d1 = (x - bx) * (ay - by) - (ax - bx) * (y - by);
      const d2 = (x - cx) * (by - cy) - (bx - cx) * (y - cy);
      const d3 = (x - ax) * (cy - ay) - (cx - ax) * (y - ay);
      if (!((d1 < 0 || d2 < 0 || d3 < 0) && (d1 > 0 || d2 > 0 || d3 > 0))) return true;
    }
    return false;
  };
  it("footprint area keeps a loop's hole (no hull over the building)", () => {
    // a 2 m wide road around a 10 x 10 m block
    const ring = [sq(0, 0, 14, 2), sq(0, 12, 14, 14), sq(0, 0, 2, 14), sq(12, 0, 14, 14)];
    const {tris, segs} = regionFootprint(ring);
    expect(covers(tris, 1, 7)).toBe(true);
    expect(covers(tris, 7, 7)).toBe(false); // the block
    expect(area(tris)).toBeGreaterThan(14 * 14 - 10 * 10 - 4);
    expect(area(tris)).toBeLessThan(14 * 14 - 10 * 10 + 8);
    expect(segs.length).toBeGreaterThan(0);
  });
  it("footprint area fuses touching places, keeps distant ones apart", () => {
    const {tris} = regionFootprint([sq(0, 0, 2, 2), sq(2.2, 0, 4, 2), sq(20, 0, 22, 2)]);
    expect(covers(tris, 2.1, 1)).toBe(true); // a 0.2 m gap is closed
    expect(covers(tris, 10, 1)).toBe(false);
    expect(covers(tris, 21, 1)).toBe(true);
    expect(regionFootprint([]).tris).toHaveLength(0);
  });
  it("footprints: rays, bbox, then padded square", () => {
    expect(placeFootprint({pos: [0, 0, 0], radii: [1, 1, 1, 1]})[0]).toEqual([1, 0]);
    expect(placeFootprint({pos: [0, 0, 0], bbox: [-1, -2, 0, 1, 2, 1]})).toContainEqual([1, 2]);
    expect(placeFootprint({pos: [0, 0, 0]}, 0.5)).toContainEqual([-0.5, 0.5]);
  });
  it("parents + hull geometry per region at the places height", () => {
    const m = GraphModel.fromSnapshot({
      nodes: [rec("P(1)", 3, 1, [0, 0, 0]), rec("P(2)", 3, 1, [4, 0, 0]), rec("P(3)", 3, 1, [2, 4, 0]), rec("r(0)", 4, 0, [2, 1, 0], 0, "parking lot")],
      edges: [["r(0)", "P(1)"], ["P(2)", "r(0)"], ["r(0)", "P(3)"]],
      labelspaces: {},
    });
    expect([...regionParents(m).keys()].sort()).toEqual(["P(1)", "P(2)", "P(3)"]);
    const l = layoutGraph(m, {...base, regionHulls: true, text: true, colors: {rules: {}, labelColors: {}, placesByRegion: true}});
    expect(l.regionFill).not.toBeNull();
    const zs = new Set(Array.from(l.regionFill!.positions).filter((_, i) => i % 3 === 2).map(z => z.toFixed(2)));
    expect([...zs]).toEqual([(4 - 0.04).toFixed(2)]);
    expect(l.regionOutline!.positions.length).toBeGreaterThan(0);
    expect(l.texts.find(t => t.kind === "rooms")?.text).toBe("parking lot"); // a named area is labelled by its name (R(n) is in Inspect)
    expect(layoutGraph(m, {...base, regionHulls: false}).regionFill).toBeNull();
    expect(layoutGraph(m, {...base, regionHulls: true, hiddenLayers: {"4/0": true}}).regionFill).toBeNull();
  });
  it("unnamed rooms fall back to their labelspace class", () => {
    const m = GraphModel.fromSnapshot({nodes: [rec("R(2)", 4, 0, [0, 0, 0], 3)], edges: [], labelspaces: {"4/0": {"3": "kitchen"}}});
    expect(layoutGraph(m, {...base, text: true}).texts[0].text).toBe("kitchen R(2)");
  });
});

describe("screenshot options", () => {
  it("JPEG ignores transparency and uses .jpg", () => {
    const e = effectiveShot({...DEFAULT_SHOT, format: "jpeg", transparent: true, quality: 0.8});
    expect(e.transparent).toBe(false);
    expect(e.mime).toBe("image/jpeg");
    expect(e.ext).toBe("jpg");
    expect(e.quality).toBe(0.8);
    const p = effectiveShot({...DEFAULT_SHOT, transparent: true});
    expect(p.transparent).toBe(true);
    expect(p.ext).toBe("png");
  });
  it("filenames", () => {
    const d = new Date(2026, 9, 20, 9, 5, 7);
    expect(shotFilename(["file:test"], d, "png")).toBe("adt4_test_20261020-090507.png");
    expect(shotFilename(["file:a", "live:spot 1"], d, "jpg")).toBe("adt4_a+spot-1_20261020-090507.jpg");
    expect(shotFilename([], d, "png")).toBe("adt4_view_20261020-090507.png");
  });
  it("nice scale-bar lengths", () => {
    expect(niceLength(3.3)).toBe(2);
    expect(niceLength(7)).toBe(5);
    expect(niceLength(0.13)).toBeCloseTo(0.1);
    expect(niceLength(12)).toBe(10);
  });
});

describe("settings persistence", () => {
  beforeEach(() => localStorage.clear());
  it("round-trips through localStorage", () => {
    const s = structuredClone(DEFAULT_VIEW_STYLE);
    s.background = "#ffffff";
    s.colors.objects = {mode: "label", color: "#123456"};
    s.labelColors.chair = "#00ff00";
    s.heights.places = 1.5;
    s.shot = {...s.shot, format: "jpeg", quality: 0.8};
    s.meshColor = {mode: "height", color: "#ffffff"};
    saveStyle(s);
    const back = parseStoredStyle(localStorage.getItem(STYLE_STORAGE_KEY));
    expect(back).toEqual(s);
  });
  it("ignores junk and keeps defaults", () => {
    expect(parseStoredStyle("{not json")).toEqual(DEFAULT_VIEW_STYLE);
    expect(parseStoredStyle(null)).toEqual(DEFAULT_VIEW_STYLE);
    const b = parseStoredStyle(JSON.stringify({background: 3, heights: {places: "x", rooms: 2}, shot: {format: "gif", quality: 7}}));
    expect(b.background).toBeNull();
    expect(b.heights).toEqual({rooms: 2});
    expect(b.shot.format).toBe("png");
    expect(b.shot.quality).toBe(DEFAULT_SHOT.quality);
  });
});

describe("robot paths", () => {
  it("one polyline per robot, broken where keyframes are far apart", () => {
    const m = GraphModel.fromSnapshot({
      nodes: [
        rec("a(0)", 2, 97, [0, 0, 0]), rec("a(1)", 2, 97, [1, 0, 0]), rec("a(2)", 2, 97, [41, 0, 0]), rec("a(3)", 2, 97, [42, 0, 0]),
        rec("b(0)", 2, 98, [0, 5, 0]), rec("b(1)", 2, 98, [1, 5, 0]),
      ],
      edges: [],
      labelspaces: {},
    });
    const paths = layoutGraph(m, {...base, flat: true}).agentPaths;
    expect(paths.map(p => p.points.map(q => q[0]))).toEqual([[0, 1], [41, 42], [0, 1]]);
    expect(paths.map(p => p.robot)).toEqual(["a", "a", "b"]);
  });
  it("splits server points at breaks, dropping lone points", () => {
    expect(splitPath([1, 2, 3, 4, 5, 6], [2, 3])).toEqual([[1, 2], [4, 5, 6]]);
    expect(splitPath([1, 2], [])).toEqual([[1, 2]]);
  });
});

describe("path sources", () => {
  it("tags each server path with its robot", () => {
    const pts = [[0, 0, 0], [1, 0, 0], [41, 0, 0], [42, 0, 0], [0, 5, 0], [1, 5, 0]] as [number, number, number][];
    const paths = trajectoryPaths(pts, [2, 4], [{key: "a", start: 0}, {key: "b", start: 4}]);
    expect(paths.map(p => [p.robot, p.points.length])).toEqual([["a", 2], ["a", 2], ["b", 2]]);
  });
  it("colours per bag, per robot (sessions of one robot match) or not at all", () => {
    const robots = [{key: "a", robot: "euclid", session: "courtyard"}, {key: "b", robot: "euclid", session: "slam"}];
    const byBag = pathColors(robots, "bag", n => n)!;
    expect(byBag.a).not.toBe(byBag.b);
    expect(pathColors(robots, "robot", n => `c-${n}`)).toEqual({a: "c-euclid", b: "c-euclid"});
    expect(pathColors(robots, "single", n => n)).toBeNull();
  });
});
