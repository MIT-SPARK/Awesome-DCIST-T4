import {describe, expect, it} from "vitest";
import {readFileSync} from "node:fs";
import {join} from "node:path";
import {decodeDelta, decodeGraph, decodePackedTile, decodeTile, unpack, parseSym} from "../src/codec";
import {GraphModel} from "../src/lib/graphModel";

const FIX = join(__dirname, "fixtures");
const bin = (n: string) => new Uint8Array(readFileSync(join(FIX, n)));
const js = (n: string) => JSON.parse(readFileSync(join(FIX, n), "utf8"));

function stateOf(m: GraphModel) {
  const nodes: Record<string, unknown> = {};
  for (const sym of [...m.nodes.keys()].sort()) nodes[sym] = m.nodes.get(sym);
  return {nodes, edges: [...m.edges.values()].map(e => [...e]).sort(), labelspaces: m.labelspaces};
}

describe("codec", () => {
  it("decodes a graph snapshot exactly like fleetlink.codec.decode_records", () => {
    const g = decodeGraph(unpack(bin("graph_small.msgpack")));
    const exp = js("graph_small.json");
    expect(g.nodes.length).toBe(Object.keys(exp.nodes).length);
    expect(stateOf(GraphModel.fromSnapshot(g))).toEqual(exp);
    const o1 = g.nodes.find(n => n.sym === "O(1)")!;
    expect(o1.bbox).toEqual([0.5, 1.5, 0, 1.5, 2.5, 1]);
    expect(g.nodes.find(n => n.sym === "O(2)")!.bbox).toBeNull();
    expect(g.nodes.find(n => n.sym === "a(0)")!.part).toBe(97);
  });

  it("handles unaligned msgpack bin views", () => {
    // msgpack hands out subarray views at arbitrary offsets: decoding must copy
    const raw = unpack(bin("graph_small.msgpack")) as Record<string, Uint8Array>;
    const shifted = new Uint8Array(raw.pos.length + 1);
    shifted.set(raw.pos, 1);
    const g = decodeGraph({...raw, pos: shifted.subarray(1)});
    expect(g.nodes[0].pos).toEqual(decodeGraph(raw).nodes[0].pos);
  });

  it("decodes a graph delta and applies it like GraphState.apply", () => {
    const m = GraphModel.fromSnapshot(decodeGraph(unpack(bin("graph_small.msgpack"))));
    const d = decodeDelta(unpack(bin("delta_small.msgpack")));
    expect(d.seq).toBe(7);
    expect(d.deletes).toEqual(["O(2)"]);
    m.applyDelta(d);
    expect(stateOf(m)).toEqual(js("delta_small.json"));
    expect(m.lastSeq).toBe(7);
  });

  it("decodes a mesh tile (quantised int16 positions)", () => {
    const t = decodeTile(unpack(bin("tile_small.msgpack")));
    const exp = js("tile_small.json");
    expect(t.key).toBe(exp.key);
    expect(t.nv).toBe(exp.nv);
    expect(t.nf).toBe(exp.nf);
    expect(Array.from(t.colors)).toEqual(exp.col);
    expect(Array.from(t.indices)).toEqual(exp.idx);
    expect(t.positions.length).toBe(exp.pos.length);
    exp.pos.forEach((v: number, i: number) => expect(t.positions[i]).toBeCloseTo(v, 5));
  });

  it("decodes a packed tile (encoding 2) exactly like fleetlink.codec.unpack_tile", () => {
    const t = decodePackedTile(unpack(bin("tile_packed.msgpack")));
    const exp = js("tile_packed.json");
    expect(exp.ib).toBe(2);
    expect([t.key, t.nv, t.nf]).toEqual([exp.key, exp.nv, exp.nf]);
    expect(Array.from(t.colors)).toEqual(exp.col);
    expect(Array.from(t.indices)).toEqual(exp.idx);
    expect(t.positions.length).toBe(exp.pos.length);
    exp.pos.forEach((v: number, i: number) => expect(t.positions[i]).toBeCloseTo(v, 4));
  });

  it("decodes 4-byte packed indices and int16 extremes (reference encoder)", () => {
    // mirror of codec.pack_tile for > 65536 vertices
    const nv = 70000;
    const nf = Math.ceil(nv / 3);
    const q = new Int16Array(nv * 3);
    let seed = 7;
    const rnd = () => ((seed = (seed * 1103515245 + 12345) >>> 0) / 2 ** 32);
    for (let i = 0; i < q.length; i++) q[i] = Math.floor(rnd() * 65536) - 32768;
    q.set([-32768, 32767, 0, 32767, -32768, 5]);
    const idx = new Uint32Array(nf * 3);
    for (let i = 0; i < idx.length; i++) idx[i] = i < nv ? i : Math.floor(rnd() * nv);
    for (let i = idx.length - 1; i > 0; i--) {
      const j = Math.floor(rnd() * (i + 1));
      [idx[i], idx[j]] = [idx[j], idx[i]];
    }
    // first-use order is what the packer emits; build it the same way
    const order: number[] = [];
    const seen = new Int32Array(nv).fill(-1);
    for (const v of idx) if (seen[v] < 0) seen[v] = order.push(v) - 1;
    const zz16 = (d: number) => ((d << 1) ^ (d >> 15)) & 0xffff;
    const pos = new Uint8Array(6 * nv);
    for (let a = 0; a < 3; a++) {
      let prev = 0;
      order.forEach((v, i) => {
        const cur = q[v * 3 + a];
        const z = zz16(((cur - prev) << 16) >> 16);
        prev = cur;
        pos[2 * nv * a + i] = z & 255;
        pos[2 * nv * a + nv + i] = z >> 8;
      });
    }
    const n = 3 * nf;
    const ib = new Uint8Array(4 * n);
    let prev = 0;
    for (let i = 0; i < n; i++) {
      const cur = seen[idx[i]];
      const d = (cur - prev) | 0;
      prev = cur;
      const z = ((d << 1) ^ (d >> 31)) >>> 0;
      for (let b = 0; b < 4; b++) ib[b * n + i] = (z >>> (8 * b)) & 255;
    }
    const t = decodePackedTile({key: "k", origin: [8, -16, 0], scale: 0.01, nv, nf, hash: "h", enc: 2, ib: 4, pos, col: new Uint8Array(3 * nv), idx: ib});
    for (let i = 0; i < n; i += 997) {
      const v = idx[i];
      expect(t.indices[i]).toBe(seen[v]);
      expect(t.positions[seen[v] * 3]).toBeCloseTo(q[v * 3] * 0.01 + 8, 3);
      expect(t.positions[seen[v] * 3 + 1]).toBeCloseTo(q[v * 3 + 1] * 0.01 - 16, 3);
    }
    expect(t.positions[0]).toBeCloseTo(q[order[0] * 3] * 0.01 + 8, 3);
  });

  it("rejects malformed packed tiles", () => {
    const good = unpack(bin("tile_packed.msgpack")) as Record<string, unknown>;
    expect(() => decodePackedTile({...good, ib: 3})).toThrow();
    expect(() => decodePackedTile({...good, pos: (good.pos as Uint8Array).subarray(1)})).toThrow();
    expect(() => decodePackedTile({...good, nv: 2})).toThrow();
  });

  it("parses node symbols", () => {
    expect(parseSym("O(12)")).toEqual({prefix: "O", index: 12});
    expect(parseSym("a(3)")).toEqual({prefix: "a", index: 3});
    expect(parseSym("weird").index).toBe(-1);
  });
});

describe("codec: TravNode ray boundaries", () => {
  it("decodes radii/states exactly like fleetlink.codec.decode_records", () => {
    const g = decodeGraph(unpack(bin("graph_places.msgpack")));
    const exp = js("graph_places.json") as Record<string, unknown>[];
    expect(g.nodes.map(n => n.sym)).toEqual(exp.map(e => e.sym));
    g.nodes.forEach((n, i) => {
      const e = exp[i] as {radii?: number[]; states?: number[]};
      if (!e.radii) {
        expect(n.radii).toBeUndefined();
        return;
      }
      expect(n.radii!.length).toBe(e.radii.length);
      e.radii.forEach((r, j) => expect(n.radii![j]).toBeCloseTo(r, 5));
      expect([...n.states!]).toEqual(e.states);
    });
  });

  it("ignores boundary keys that are absent (old peers)", () => {
    const raw = unpack(bin("graph_places.msgpack")) as Record<string, unknown>;
    const {nrad: _n, radii: _r, states: _s, ...old} = raw;
    expect(decodeGraph(old).nodes.every(n => n.radii === undefined)).toBe(true);
  });
});
