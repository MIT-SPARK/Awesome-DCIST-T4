/** Subsequence fuzzy score (lower is better), or null if `q` does not match `s`. */
export function fuzzyScore(q: string, s: string): number | null {
  q = q.toLowerCase();
  s = s.toLowerCase();
  if (!q) return 0;
  const sub = s.indexOf(q);
  if (sub >= 0) return sub;
  let score = 100;
  let pos = -1;
  for (const ch of q) {
    const nxt = s.indexOf(ch, pos + 1);
    if (nxt < 0) return null;
    score += nxt - pos - 1;
    pos = nxt;
  }
  return score;
}

export function fuzzyFilter<T>(items: T[], q: string, key: (t: T) => string): T[] {
  if (!q.trim()) return items;
  return items
    .map(it => [fuzzyScore(q.trim(), key(it)), it] as const)
    .filter((x): x is readonly [number, T] => x[0] !== null)
    .sort((a, b) => a[0] - b[0])
    .map(x => x[1]);
}

export function fmtBytes(n: number | null | undefined): string {
  if (n === null || n === undefined || !Number.isFinite(n)) return "-";
  const u = ["B", "kB", "MB", "GB", "TB"];
  let i = 0;
  while (Math.abs(n) >= 1000 && i < u.length - 1) {
    n /= 1000;
    i++;
  }
  return `${n.toFixed(n >= 100 || i === 0 ? 0 : 1)} ${u[i]}`;
}

export function fmtRate(bps: number | null | undefined): string {
  return bps === null || bps === undefined ? "-" : `${fmtBytes(bps)}/s`;
}

export function fmtAgo(epochS: number | null | undefined): string {
  if (!epochS) return "never";
  const d = Date.now() / 1000 - epochS;
  if (d < 2) return "just now";
  if (d < 60) return `${Math.round(d)} s ago`;
  if (d < 3600) return `${Math.round(d / 60)} min ago`;
  return `${Math.round(d / 3600)} h ago`;
}

export function fmtTime(epochS: number | null | undefined): string {
  if (!epochS) return "";
  return new Date(epochS * 1000).toLocaleTimeString([], {hour12: false});
}

// ---------------------------------------------------------------------------
// fzf-style matcher with highlight positions (higher score = better match)
// ---------------------------------------------------------------------------

export interface FuzzyMatch {
  score: number;
  /** indices into the candidate string of the matched characters */
  indices: number[];
}

const SCORE_MATCH = 16;
const BONUS_BOUNDARY = 10; // start of string / after _-./ space / camelCase / letter<->digit
const BONUS_CONSECUTIVE = 8;
const BONUS_FIRST_CHAR = 6;
const PENALTY_GAP = 1; // per skipped candidate char between matches
const PENALTY_LEAD = 0.25; // per candidate char before the first match (capped)
const PENALTY_TYPO = 24; // per dropped / transposed query char

function isSep(c: string): boolean {
  return c === "_" || c === "-" || c === "." || c === "/" || c === " " || c === ":";
}

function boundaryBonus(s: string, j: number): number {
  if (j === 0) return BONUS_BOUNDARY;
  const p = s[j - 1];
  const c = s[j];
  if (isSep(p)) return BONUS_BOUNDARY;
  const pl = p.toLowerCase() === p && p.toUpperCase() !== p;
  const cu = c.toUpperCase() === c && c.toLowerCase() !== c;
  if (pl && cu) return BONUS_BOUNDARY * 0.8;
  const pd = p >= "0" && p <= "9";
  const cd = c >= "0" && c <= "9";
  if (pd !== cd) return BONUS_BOUNDARY * 0.5;
  return 0;
}

function isSubsequence(q: string, s: string): boolean {
  let j = 0;
  for (let i = 0; i < q.length; i++) {
    j = s.indexOf(q[i], j);
    if (j < 0) return false;
    j++;
  }
  return true;
}

/** Best-scoring subsequence alignment of lowercase `q` in `s` (DP), or null. */
function alignSubsequence(q: string, s: string, sl: string): FuzzyMatch | null {
  const n = q.length;
  const m = s.length;
  if (!n) return {score: 0, indices: []};
  if (n > m || !isSubsequence(q, sl)) return null;
  const NEG = -Infinity;
  // M[i][j]: best score with q[i] matched at s[j]; from[i][j]: j of q[i-1] (or -1)
  const M: Float64Array[] = [];
  const from: Int32Array[] = [];
  const bonus = new Float64Array(m);
  for (let j = 0; j < m; j++) bonus[j] = boundaryBonus(s, j);
  for (let i = 0; i < n; i++) {
    const row = new Float64Array(m).fill(NEG);
    const fr = new Int32Array(m).fill(-1);
    // running best of M[i-1][k] - gap*(j-k-1) over k < j-1
    let best = NEG;
    let bestK = -1;
    for (let j = 0; j < m; j++) {
      if (i > 0 && j >= 2) {
        const cand = M[i - 1][j - 2];
        best -= PENALTY_GAP;
        if (cand > best) {
          best = cand;
          bestK = j - 2;
        }
      }
      if (sl[j] !== q[i]) continue;
      const base = SCORE_MATCH + bonus[j];
      if (i === 0) {
        row[j] = base + (j === 0 ? BONUS_FIRST_CHAR : 0) - Math.min(j * PENALTY_LEAD, 8);
        continue;
      }
      const consec = j >= 1 ? M[i - 1][j - 1] : NEG;
      if (consec > NEG) {
        // consecutive runs keep the boundary bonus of their first char (fzf)
        const v = consec + base + BONUS_CONSECUTIVE;
        if (v > row[j]) {
          row[j] = v;
          fr[j] = j - 1;
        }
      }
      if (best > NEG) {
        const v = best - PENALTY_GAP + base;
        if (v > row[j]) {
          row[j] = v;
          fr[j] = bestK;
        }
      }
    }
    M.push(row);
    from.push(fr);
  }
  let end = -1;
  let score = NEG;
  for (let j = 0; j < m; j++)
    if (M[n - 1][j] > score) {
      score = M[n - 1][j];
      end = j;
    }
  if (end < 0 || score === NEG) return null;
  const indices = new Array<number>(n);
  for (let i = n - 1, j = end; i >= 0; i--) {
    indices[i] = j;
    j = from[i][j];
  }
  return {score, indices};
}

/**
 * fzf-style fuzzy match of `query` against `s`: subsequence with bonuses for word
 * boundaries, consecutive runs and an early start. Typo tolerant: when the plain
 * subsequence fails and the query has >= 4 chars, one dropped or two transposed query
 * characters are allowed (penalised). Case-insensitive. Null when nothing matches.
 */
export function fuzzyMatch(query: string, s: string): FuzzyMatch | null {
  const q = query.toLowerCase();
  const sl = s.toLowerCase();
  const direct = alignSubsequence(q, s, sl);
  if (direct || q.length < 4) return direct;
  let best: FuzzyMatch | null = null;
  const consider = (variant: string, penalty: number) => {
    const r = alignSubsequence(variant, s, sl);
    if (r && (!best || r.score - penalty > best.score)) best = {score: r.score - penalty, indices: r.indices};
  };
  for (let i = 0; i < q.length; i++) consider(q.slice(0, i) + q.slice(i + 1), PENALTY_TYPO);
  for (let i = 0; i + 1 < q.length; i++) {
    if (q[i] === q[i + 1]) continue;
    consider(q.slice(0, i) + q[i + 1] + q[i] + q.slice(i + 2), PENALTY_TYPO);
  }
  return best;
}

/** Merge sorted match indices into [start, end) ranges for highlighting. */
export function matchRanges(indices: number[]): [number, number][] {
  const out: [number, number][] = [];
  for (const i of [...indices].sort((a, b) => a - b)) {
    const last = out[out.length - 1];
    if (last && last[1] === i) last[1] = i + 1;
    else if (!last || last[1] < i) out.push([i, i + 1]);
  }
  return out;
}
