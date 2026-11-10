import {describe, expect, it} from "vitest";
import type {NodeRecord} from "../src/codec";
import {GraphModel, layerInfo, nearestPlace, searchNodes} from "../src/lib/graphModel";
import {appendPlaceBoundary, DEFAULT_EDGE_ON, layoutGraph, PLACE_BOUNDARY} from "../src/viewer/layout";

const rec = (sym: string, layer: number, part: number, pos: [number, number, number], label = -1, name = ""): NodeRecord => ({
  sym, layer, part, pos, label, bbox: null, color: [0, 0, 0], name, active: false,
});

function model() {
  return GraphModel.fromSnapshot({
    nodes: [
      rec("P(1)", 3, 0, [0, 0, 1]),
      rec("P(2)", 3, 0, [10, 0, 1]),
      rec("P(3)", 3, 0, [0, 10, 1]),
      rec("O(1)", 2, 0, [9, 1, 0.5], 5),
      rec("O(2)", 2, 0, [1, 9, 0.5], 4, "red sign"),
      rec("a(0)", 2, 97, [0, 0, 0]),
      rec("a(1)", 2, 97, [1, 0, 0]),
    ],
    edges: [["P(1)", "P(2)"], ["P(1)", "P(3)"], ["P(2)", "O(1)"], ["a(0)", "a(1)"]],
    labelspaces: {"2/0": {"5": "sign", "4": "chair"}},
  });
}

describe("graph store delta application", () => {
  it("upserts, deletes and drops edges of deleted nodes", () => {
    const m = model();
    const rev = m.rev;
    m.applyDelta({
      upserts: [rec("O(1)", 2, 0, [9.5, 1, 0.5], 5), rec("O(3)", 2, 0, [5, 5, 0], 4)],
      deletes: ["P(2)"],
      edgesAdd: [["P(3)", "O(3)"]],
      edgesDel: [["P(1)", "P(3)"]],
      labelspaces: null,
      seq: 3,
    });
    expect(m.rev).toBeGreaterThan(rev);
    expect(m.nodes.get("O(1)")!.pos[0]).toBe(9.5);
    expect(m.nodes.has("P(2)")).toBe(false);
    expect(m.nodes.has("O(3)")).toBe(true);
    const edges = [...m.edges.values()].map(e => e.join("-")).sort();
    // P(1)-P(2) and P(2)-O(1) go with P(2); P(1)-P(3) explicitly removed
    expect(edges).toEqual(["P(3)-O(3)", "a(0)-a(1)"]);
    expect(m.labelspaces["2/0"]["5"]).toBe("sign"); // null ls keeps the old one
    expect(m.lastSeq).toBe(3);
  });

  it("replaces labelspaces when the delta carries them", () => {
    const m = model();
    m.applyDelta({upserts: [], deletes: [], edgesAdd: [], edgesDel: [], labelspaces: {"2/0": {"5": "placard"}}});
    expect(m.labelName(m.nodes.get("O(1)")!)).toBe("placard");
  });

  it("classifies layers like hydra (agents = layer 2 partition 'a')", () => {
    expect(layerInfo(2, 97).kind).toBe("agents");
    expect(layerInfo(2, 0).kind).toBe("objects");
    expect(layerInfo(3, 2).kind).toBe("places");
    expect(layerInfo(4, 0).kind).toBe("rooms");
  });

  it("lays out layers with Z stacking and a flat mode", () => {
    const m = model();
    const stacked = layoutGraph(m, {hiddenLayers: {}, edgeKinds: {}, flat: false, stackHeight: 3});
    const places = stacked.layers.find(l => l.key === "3/0")!;
    expect(places.positions[2]).toBe(1 + 3);
    const objects = stacked.layers.find(l => l.key === "2/0")!;
    expect(objects.positions[2]).toBe(0.5);
    const flat = layoutGraph(m, {hiddenLayers: {"2/0": true}, edgeKinds: {}, flat: true, stackHeight: 3});
    expect(flat.layers.find(l => l.key === "3/0")!.positions[2]).toBe(1);
    expect(flat.layers.find(l => l.key === "2/0")).toBeUndefined();
    expect(stacked.edgeCounts["places-places"]).toBe(2);
  });
});

describe("hydra style text", () => {
  it("labels objects/places/rooms per node but agents once per layer", () => {
    const m = model();
    const l = layoutGraph(m, {hiddenLayers: {}, edgeKinds: {}, flat: false, stackHeight: 2, style: "hydra", text: true});
    const agentTexts = l.texts.filter(t => t.kind === "agents");
    const agentNodes = [...m.nodes.values()].filter(n => n.layer === 2 && n.part === 97).length;
    expect(agentTexts.length).toBe(agentNodes ? 1 : 0);
    expect(l.texts.filter(t => t.kind === "places").length).toBe([...m.nodes.values()].filter(n => n.layer === 3).length);
    // places stack at z_offset_scale 2 * layer_z_step
    expect(l.layers.find(x => x.key === "3/0")!.positions[2]).toBe(1 + 4);
  });
});

describe("nearest place resolution", () => {
  it("snaps a mesh click to the closest place node", () => {
    const m = model();
    expect(nearestPlace(m, [8, 1, 0])!.sym).toBe("P(2)");
    expect(nearestPlace(m, [1, 8, 0])!.sym).toBe("P(3)");
    expect(nearestPlace(m, [0.4, 0.3, 0])!.sym).toBe("P(1)");
  });
  it("ignores objects/agents and honours maxDist", () => {
    const m = model();
    // O(1) is closer than any place but is not a place
    expect(nearestPlace(m, [9, 1, 0.5])!.sym).toBe("P(2)");
    expect(nearestPlace(m, [100, 100, 0], {maxDist: 5})).toBeNull();
    expect(nearestPlace(null, [0, 0, 0])).toBeNull();
  });
});

describe("search", () => {
  it("matches label names and node names, objects first", () => {
    const m = model();
    expect(searchNodes(m, "sign").map(n => n.sym)).toEqual(["O(1)", "O(2)"].sort((a, b) => (a === "O(1)" ? -1 : 1)));
    expect(searchNodes(m, "P(3)").map(n => n.sym)).toEqual(["P(3)"]);
    expect(searchNodes(m, "  ")).toEqual([]);
  });
});

describe("layerInfo partitions", () => {
  it("names Hydra partitions and keeps sub-keyframes out of objects", () => {
    expect(layerInfo(2, 0).kind).toBe("objects");
    expect(layerInfo(2, 97).name).toBe("agents");
    expect(layerInfo(2, 115)).toMatchObject({name: "subkeyframes", kind: "subkeyframes"});
    expect(layerInfo(3, 1).name).toBe("mesh places");
    expect(layerInfo(3, 2).name).toBe("traversability");
    expect(DEFAULT_EDGE_ON("agents-subkeyframes")).toBe(false);
    expect(DEFAULT_EDGE_ON("objects-places")).toBe(true);
  });
});

describe("place boundaries (RegionGrowingBoundaryPlugin)", () => {
  const lin = (c: number) => Math.pow(c / 255, 2.2);
  const place = (extra: Partial<NodeRecord>): NodeRecord => ({
    sym: "t(1)", layer: 3, part: 1, pos: [1, 2, 0.5], label: -1, bbox: null, color: [0, 0, 0], name: "", active: false, ...extra,
  });

  it("builds state-coloured segments i -> (i+1)%N and a fan fill at the z offset", () => {
    const lp: number[] = [], lc: number[] = [], fp: number[] = [], fc: number[] = [];
    const n = place({radii: new Float32Array([1, 2, 1, 2]), states: new Uint8Array([0, 1, 2, 3])});
    expect(appendPlaceBoundary(n, 4, [0, 0, 0], lp, lc, fp, fc)).toBe(true);
    expect(lp.length).toBe(4 * 2 * 3); // N segments, 2 endpoints
    expect(fp.length).toBe(4 * 3 * 3); // N triangles
    // bin 1 at 90deg, radius 2: (1, 4); every vertex at pos.z + offset
    expect(lp[3]).toBeCloseTo(1);
    expect(lp[4]).toBeCloseTo(4);
    for (let i = 2; i < lp.length; i += 3) expect(lp[i]).toBeCloseTo(4.5);
    // last segment wraps 3 -> 0 and its colours go TRAVERSED (green) -> UNKNOWN (gray)
    expect(lp.slice(21, 24).map(v => +v.toFixed(5))).toEqual([2, 2, 4.5]);
    expect(lc.slice(18, 24)).toEqual([0, 1, 0, lin(128), lin(128), lin(128)]);
    // segment 0 -> 1: UNKNOWN gray -> TRAVERSABLE blue; segment 1 -> 2: blue -> INTRAVERSABLE red
    expect(lc.slice(0, 6)).toEqual([lin(128), lin(128), lin(128), 0, 0, 1]);
    expect(lc.slice(6, 12)).toEqual([0, 0, 1, 1, 0, 0]);
    // fill: fan from the node centre in the node colour (black)
    expect(fp.slice(0, 3)).toEqual([1, 2, 4.5]);
    expect(fc.every(v => v === 0)).toBe(true);
  });

  it("falls back to the bbox footprint when there are no radii", () => {
    const lp: number[] = [], lc: number[] = [], fp: number[] = [], fc: number[] = [];
    expect(appendPlaceBoundary(place({}), 0, [0, 0, 0], lp, lc, fp, fc)).toBe(false);
    expect(appendPlaceBoundary(place({bbox: [0, 0, 0, 2, 1, 1]}), 0, [0, 0, 0], lp, lc, fp, fc)).toBe(true);
    expect(lp.length).toBe(4 * 6);
    expect(fp.length).toBe(4 * 9);
  });

  it("layoutGraph merges all places into one line buffer and one fill buffer (hydra only)", () => {
    const m = GraphModel.fromSnapshot({
      nodes: [
        place({sym: "t(1)", radii: new Float32Array(20).fill(1), states: new Uint8Array(20).fill(1)}),
        place({sym: "t(2)", pos: [5, 5, 0], radii: new Float32Array(30).fill(2), states: new Uint8Array(30)}),
      ],
      edges: [],
      labelspaces: {},
    });
    const opts = {hiddenLayers: {}, edgeKinds: {}, flat: false, stackHeight: 2};
    const h = layoutGraph(m, {...opts, style: "hydra"});
    expect(h.placeBoundary!.positions.length).toBe((20 + 30) * 6);
    expect(h.placeFill!.positions.length).toBe((20 + 30) * 9);
    // 3p1 z_offset_scale 2 x layer_z_step 2
    expect(h.placeBoundary!.positions[2]).toBeCloseTo(0.5 + 4);
    expect(PLACE_BOUNDARY.lineWidth).toBe(0.07);
    const c = layoutGraph(m, {...opts, style: "classic"});
    expect(c.placeBoundary).toBeNull();
    expect(c.placeFill).toBeNull();
  });
});
