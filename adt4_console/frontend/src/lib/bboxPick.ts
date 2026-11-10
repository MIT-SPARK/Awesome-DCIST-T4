// Click picking of object bounding boxes (the 3D wireframes): a ray / AABB slab
// test over the displayed boxes. When boxes overlap the smallest enclosing one
// wins (a mug on a table selects the mug), ties go to the nearest entry point.

export interface BoxSet {
  syms: string[];
  /** 6 floats per sym: min xyz, max xyz (displayed coordinates); NaN = no box */
  boxes: Float32Array;
}

export interface BoxHit {
  sym: string;
  /** ray parameter at entry (0 when the ray starts inside the box) */
  t: number;
  volume: number;
}

/** Ray parameter interval [tmin, tmax] inside the box, or null when missed. */
export function rayBox(o: ArrayLike<number>, d: ArrayLike<number>, b: ArrayLike<number>, off = 0): [number, number] | null {
  let tmin = -Infinity;
  let tmax = Infinity;
  for (let k = 0; k < 3; k++) {
    const lo = b[off + k];
    const hi = b[off + 3 + k];
    if (!Number.isFinite(lo) || !Number.isFinite(hi)) return null;
    if (Math.abs(d[k]) < 1e-12) {
      if (o[k] < lo || o[k] > hi) return null;
      continue;
    }
    let t1 = (lo - o[k]) / d[k];
    let t2 = (hi - o[k]) / d[k];
    if (t1 > t2) [t1, t2] = [t2, t1];
    tmin = Math.max(tmin, t1);
    tmax = Math.min(tmax, t2);
    if (tmin > tmax) return null;
  }
  if (tmax < 0) return null;
  return [Math.max(0, tmin), tmax];
}

/** Best box along the ray among `sets`: smallest volume, then nearest. */
export function pickBox<K>(o: ArrayLike<number>, d: ArrayLike<number>, sets: Iterable<[K, BoxSet]>): (BoxHit & {key: K}) | null {
  let best: (BoxHit & {key: K}) | null = null;
  for (const [key, s] of sets) {
    const b = s.boxes;
    for (let i = 0; i < s.syms.length; i++) {
      const off = i * 6;
      const hit = rayBox(o, d, b, off);
      if (!hit) continue;
      const volume = Math.max(b[off + 3] - b[off], 1e-3) * Math.max(b[off + 4] - b[off + 1], 1e-3) * Math.max(b[off + 5] - b[off + 2], 1e-3);
      if (!best || volume < best.volume * 0.999 || (volume <= best.volume * 1.001 && hit[0] < best.t)) {
        best = {key, sym: s.syms[i], t: hit[0], volume};
      }
    }
  }
  return best;
}
