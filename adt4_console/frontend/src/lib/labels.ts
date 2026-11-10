// Text-label decluttering for the 3D view: of all the node labels on screen,
// keep the most important ones that do not overlap (screen-space collision
// culling), at most `max`. Zoomed out, a dense map shows a readable handful;
// zooming in frees room and more appear. The selected / hovered node's label
// always wins (GraphLayers.tsx TextLabels).

export interface LabelBox {
  /** centre, px */
  x: number;
  y: number;
  w: number;
  h: number;
  prio: number;
}

/** Indices of the labels to show: by priority, each clear of those already kept. */
export function declutter(boxes: readonly LabelBox[], max: number, gap = 2): number[] {
  const order = boxes.map((_, i) => i).sort((a, b) => boxes[b].prio - boxes[a].prio);
  const kept: number[] = [];
  for (const i of order) {
    if (kept.length >= max) break;
    const a = boxes[i];
    const clear = kept.every(j => {
      const b = boxes[j];
      return Math.abs(a.x - b.x) * 2 >= a.w + b.w + gap * 2 || Math.abs(a.y - b.y) * 2 >= a.h + b.h + gap * 2;
    });
    if (clear) kept.push(i);
  }
  return kept;
}

/** Size estimate of a label (the CSS of .viz-text-<kind>), px. */
export function labelSize(text: string, kind: string): {w: number; h: number} {
  // areas: bold upper-case with letter spacing on a padded pill (styles.css .viz-text-rooms)
  const [cw, h, pad] = kind === "places" ? [5.2, 12, 6] : kind === "rooms" ? [10, 22, 20] : [6.2, 14, 6];
  return {w: text.length * cw + pad, h};
}

const KIND_WEIGHT: Record<string, number> = {rooms: 4, buildings: 4, agents: 3, objects: 2, places: 1};

/** Priority: a picked node first, then by kind (areas over objects over
 *  places), then nearest the pointer (or the view centre). */
export function labelPrio(kind: string, distPx: number, picked: boolean): number {
  return (picked ? 1e7 : 0) + (KIND_WEIGHT[kind] ?? 1) * 1e5 - Math.min(distPx, 99_999);
}

/** Labels kept on screen at once. */
export const MAX_LABELS = 120;
