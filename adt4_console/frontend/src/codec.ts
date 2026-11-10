// Decoders for the console wire formats (mirror of adt4_console/adt4_console/codec.py).
// msgpack `bin` fields arrive as Uint8Array views that may be unaligned, so every
// typed array is built from a fresh aligned copy.
import {decode} from "@msgpack/msgpack";

export interface NodeRecord {
  sym: string;
  layer: number;
  part: number;
  pos: [number, number, number];
  label: number;
  bbox: [number, number, number, number, number, number] | null;
  color: [number, number, number];
  name: string;
  active: boolean;
  /** spark_dsg TravNodeAttributes ray boundary (traversability places): ray i
   *  has length radii[i] metres at angle i/N*2pi from pos; states[i] is a
   *  TraversabilityState (0 unknown, 1 traversable, 2 intraversable, 3 traversed).
   *  Absent for every other node type and from peers that predate the fields. */
  radii?: Float32Array | null;
  states?: Uint8Array | null;
}

export type Edge = [string, string];
export type Labelspaces = Record<string, Record<string, string>>;

export interface EncodedRecords {
  ids: string[];
  layer: Uint8Array;
  part: Uint8Array;
  pos: Uint8Array;
  label: Uint8Array;
  bbox: Uint8Array;
  color: Uint8Array;
  names: string[];
  active: Uint8Array;
  /** optional boundary keys: uint16[N] rays per node, uint16[sum] cm, uint8[sum] */
  nrad?: Uint8Array;
  radii?: Uint8Array;
  states?: Uint8Array;
}

export interface DecodedGraph {
  nodes: NodeRecord[];
  edges: Edge[];
  labelspaces: Labelspaces;
  version?: number;
  src?: string;
  /** last delta seq already folded into this snapshot */
  seq?: number;
}

export interface DecodedDelta {
  upserts: NodeRecord[];
  deletes: string[];
  edgesAdd: Edge[];
  edgesDel: Edge[];
  labelspaces: Labelspaces | null;
  seq?: number;
}

export interface DecodedTile {
  key: string;
  origin: [number, number, number];
  scale: number;
  positions: Float32Array; // nv*3, metres
  colors: Uint8Array; // nv*3
  indices: Uint32Array; // nf*3
  nv: number;
  nf: number;
  hash: string;
}

/** Copy bytes into a new ArrayBuffer so typed-array views are aligned. */
function aligned(bytes: Uint8Array | ArrayBuffer | undefined | null): ArrayBuffer {
  if (!bytes) return new ArrayBuffer(0);
  const src = bytes instanceof Uint8Array ? bytes : new Uint8Array(bytes);
  const out = new ArrayBuffer(src.byteLength);
  new Uint8Array(out).set(src);
  return out;
}

export function asF32(b: Uint8Array | undefined | null): Float32Array {
  return new Float32Array(aligned(b));
}
export function asI16(b: Uint8Array | undefined | null): Int16Array {
  return new Int16Array(aligned(b));
}
export function asU32(b: Uint8Array | undefined | null): Uint32Array {
  return new Uint32Array(aligned(b));
}
export function asU16(b: Uint8Array | undefined | null): Uint16Array {
  return new Uint16Array(aligned(b));
}
export function asU8(b: Uint8Array | undefined | null): Uint8Array {
  return new Uint8Array(aligned(b));
}

export function decodeRecords(enc: Partial<EncodedRecords> | null | undefined): NodeRecord[] {
  if (!enc || !enc.ids) return [];
  const n = enc.ids.length;
  const layer = asU8(enc.layer);
  const part = asU8(enc.part);
  const pos = asF32(enc.pos);
  const label = asI16(enc.label);
  const bbox = asF32(enc.bbox);
  const color = asU8(enc.color);
  const active = asU8(enc.active);
  const names = enc.names ?? [];
  const nrad = enc.nrad ? asU16(enc.nrad) : null;
  const radCm = nrad ? asU16(enc.radii) : null;
  const radSt = nrad ? asU8(enc.states) : null;
  let off = 0;
  const out: NodeRecord[] = new Array(n);
  for (let i = 0; i < n; i++) {
    const b0 = bbox[i * 6];
    let radii: Float32Array | null = null;
    let states: Uint8Array | null = null;
    const k = nrad ? (nrad[i] ?? 0) : 0;
    if (k > 0 && radCm && radSt) {
      radii = new Float32Array(k);
      for (let j = 0; j < k; j++) radii[j] = (radCm[off + j] ?? 0) * 0.01;
      states = new Uint8Array(k);
      states.set(radSt.subarray(off, off + k));
      off += k;
    }
    const rec: NodeRecord = {
      sym: enc.ids[i],
      layer: layer[i] ?? 0,
      part: part[i] ?? 0,
      pos: [pos[i * 3] ?? 0, pos[i * 3 + 1] ?? 0, pos[i * 3 + 2] ?? 0],
      label: label[i] ?? -1,
      bbox:
        b0 === undefined || Number.isNaN(b0)
          ? null
          : [b0, bbox[i * 6 + 1], bbox[i * 6 + 2], bbox[i * 6 + 3], bbox[i * 6 + 4], bbox[i * 6 + 5]],
      color: [color[i * 3] ?? 0, color[i * 3 + 1] ?? 0, color[i * 3 + 2] ?? 0],
      name: names[i] ?? "",
      active: Boolean(active[i]),
    };
    if (radii) {
      rec.radii = radii;
      rec.states = states;
    }
    out[i] = rec;
  }
  return out;
}

type Raw = Record<string, unknown>;

export function decodeGraph(obj: unknown): DecodedGraph {
  const o = (obj ?? {}) as Raw;
  return {
    nodes: decodeRecords(o as Partial<EncodedRecords>),
    edges: ((o.edges as unknown[]) ?? []).map(e => [String((e as unknown[])[0]), String((e as unknown[])[1])] as Edge),
    labelspaces: (o.labelspaces as Labelspaces) ?? {},
    version: typeof o.version === "number" ? o.version : undefined,
    src: typeof o.src === "string" ? o.src : undefined,
    seq: typeof o.seq === "number" ? o.seq : undefined,
  };
}

export function decodeDelta(obj: unknown): DecodedDelta {
  const o = (obj ?? {}) as Raw;
  const pairs = (v: unknown): Edge[] =>
    ((v as unknown[]) ?? []).map(e => [String((e as unknown[])[0]), String((e as unknown[])[1])] as Edge);
  return {
    upserts: decodeRecords(o.up as Partial<EncodedRecords>),
    deletes: ((o.del as unknown[]) ?? []).map(String),
    edgesAdd: pairs(o.ea),
    edgesDel: pairs(o.ed),
    labelspaces: (o.ls as Labelspaces) ?? null,
    seq: typeof o.seq === "number" ? o.seq : undefined,
  };
}

export function decodeTile(obj: unknown): DecodedTile {
  const o = (obj ?? {}) as Raw;
  const origin = ((o.origin as number[]) ?? [0, 0, 0]).map(Number) as [number, number, number];
  const scale = Number(o.scale ?? 0.01);
  const q = asI16(o.pos as Uint8Array);
  const positions = new Float32Array(q.length);
  for (let i = 0; i < q.length; i += 3) {
    positions[i] = q[i] * scale + origin[0];
    positions[i + 1] = q[i + 1] * scale + origin[1];
    positions[i + 2] = q[i + 2] * scale + origin[2];
  }
  const colors = asU8(o.col as Uint8Array);
  const indices = asU32(o.idx as Uint8Array);
  return {
    key: String(o.key ?? ""),
    origin,
    scale,
    positions,
    colors,
    indices,
    nv: Number(o.nv ?? positions.length / 3),
    nf: Number(o.nf ?? indices.length / 3),
    hash: String(o.hash ?? ""),
  };
}

/** Packed tile (encoding 2, codec.pack_tile): per-axis zigzag deltas of
 *  the int16 positions in low/high byte planes, per-channel uint8 colour deltas,
 *  zigzag-delta indices in `ib` byte planes. Same mesh as decodeTile, with the
 *  vertices in first-use order. Reads the msgpack bin views in place (byte
 *  access needs no alignment). */
export function decodePackedTile(obj: unknown): DecodedTile {
  const o = (obj ?? {}) as Raw;
  const origin = ((o.origin as number[]) ?? [0, 0, 0]).map(Number) as [number, number, number];
  const scale = Number(o.scale ?? 0.01);
  const nv = Number(o.nv ?? 0);
  const nf = Number(o.nf ?? 0);
  const ib = Number(o.ib ?? 2);
  const pos = (o.pos as Uint8Array) ?? new Uint8Array(0);
  const col = (o.col as Uint8Array) ?? new Uint8Array(0);
  const idx = (o.idx as Uint8Array) ?? new Uint8Array(0);
  if ((ib !== 2 && ib !== 4) || pos.length < 6 * nv || col.length < 3 * nv || idx.length < 3 * nf * ib) {
    throw new Error("malformed packed tile");
  }
  const positions = new Float32Array(nv * 3);
  for (let a = 0; a < 3; a++) {
    const lo = 2 * nv * a;
    const hi = lo + nv;
    let q = 0;
    for (let i = 0; i < nv; i++) {
      const z = pos[lo + i] | (pos[hi + i] << 8);
      q = ((q + ((z >>> 1) ^ -(z & 1))) << 16) >> 16; // int16 wrap
      positions[i * 3 + a] = q * scale + origin[a];
    }
  }
  const colors = new Uint8Array(nv * 3);
  for (let c = 0; c < 3; c++) {
    let v = 0;
    for (let i = 0; i < nv; i++) {
      v = (v + col[c * nv + i]) & 255;
      colors[i * 3 + c] = v;
    }
  }
  const n = 3 * nf;
  const indices = new Uint32Array(n);
  let prev = 0;
  let max = 0;
  for (let i = 0; i < n; i++) {
    const z =
      ib === 2 ? idx[i] | (idx[n + i] << 8) : (idx[i] | (idx[n + i] << 8) | (idx[2 * n + i] << 16) | (idx[3 * n + i] << 24)) >>> 0;
    const d = (z >>> 1) ^ -(z & 1);
    prev = ib === 2 ? (prev + d) & 0xffff : (prev + d) >>> 0; // wrap like the encoder
    indices[i] = prev;
    if (prev > max) max = prev;
  }
  if (n && max >= nv) throw new Error("packed tile index out of range");
  return {key: String(o.key ?? ""), origin, scale, positions, colors, indices, nv, nf, hash: String(o.hash ?? "")};
}

export function unpack(buf: ArrayBuffer | Uint8Array): unknown {
  return decode(buf instanceof Uint8Array ? buf : new Uint8Array(buf));
}

/** "O(12)" → {prefix: "O", index: 12}. */
export function parseSym(sym: string): {prefix: string; index: number} {
  const m = /^(.+?)\((\d+)\)$/.exec(sym);
  if (!m) return {prefix: sym, index: -1};
  return {prefix: m[1], index: Number(m[2])};
}
