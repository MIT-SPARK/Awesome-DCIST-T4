// Deterministic label -> colour palette (high contrast on a dark background).
import {useMemo} from "react";
import {create} from "zustand";

// Also the per-robot palette: keep in sync with dcist_launch_system/robot_colors.py.
export const PALETTE: [number, number, number][] = [
  [230, 25, 75], [60, 180, 75], [255, 225, 25], [67, 99, 216], [245, 130, 49],
  [145, 30, 180], [66, 212, 244], [240, 50, 230], [191, 239, 69], [250, 190, 212],
  [70, 153, 144], [220, 190, 255], [154, 99, 36], [255, 250, 200], [128, 0, 0],
  [170, 255, 195], [128, 128, 0], [255, 216, 177], [0, 117, 255], [169, 169, 169],
];

export function labelColor(label: number): [number, number, number] {
  if (label < 0) return [200, 200, 200];
  // multiplicative hash so neighbouring ids get distant palette entries
  const h = (Math.imul(label + 1, 2654435761) >>> 0) % PALETTE.length;
  return PALETTE[h];
}

export function nodeColor(label: number, color: [number, number, number]): [number, number, number] {
  if (color[0] || color[1] || color[2]) return color;
  return labelColor(label);
}

export const LAYER_COLORS: Record<string, [number, number, number]> = {
  agents: [255, 120, 40],
  subkeyframes: [255, 190, 130],
  places: [70, 200, 255],
  rooms: [240, 80, 200],
  buildings: [255, 255, 255],
  segments: [180, 180, 180],
  other: [180, 180, 180],
};

export const rgbCss = (c: [number, number, number]) => `rgb(${c[0]}, ${c[1]}, ${c[2]})`;

/** Per-source tint for overlaying several graph sources (robots use robotColor). */
export const SOURCE_TINTS = ["#4fc3f7", "#ffb74d", "#ba68c8", "#81c784", "#f06292"];

const hex2 = (n: number) => Math.max(0, Math.min(255, Math.round(n))).toString(16).padStart(2, "0");
export const rgbHex = (c: [number, number, number]) => `#${hex2(c[0])}${hex2(c[1])}${hex2(c[2])}`;
export const PALETTE_HEX = PALETTE.map(rgbHex);

function hexRgb(hex: string): [number, number, number] | null {
  const m = /^#?([0-9a-f]{6})$/i.exec(hex.trim());
  if (!m) return null;
  const v = parseInt(m[1], 16);
  return [(v >> 16) & 255, (v >> 8) & 255, v & 255];
}

/** k < 1 darkens toward black, k > 1 lightens toward white (k = 2 is white). */
export function shade(hex: string, k: number): string {
  const c = hexRgb(hex);
  if (!c) return hex;
  return rgbHex(k <= 1 ? (c.map(x => x * k) as typeof c) : (c.map(x => x + (255 - x) * (k - 1)) as typeof c));
}

/** Same rule as robot_colors.py: explicit colours win, everyone else takes the next
 *  unused palette entry in sorted-name order (no hashing, so robots never collide
 *  until the palette runs out). */
export function resolveRobotColors(names: Iterable<string>, explicit: Record<string, string>): Record<string, string> {
  const out: Record<string, string> = {};
  for (const [n, c] of Object.entries(explicit)) {
    const rgb = hexRgb(c);
    if (rgb) out[n] = rgbHex(rgb);
  }
  const used = new Set(Object.values(out));
  const free = PALETTE_HEX.filter(c => !used.has(c));
  const rest = [...new Set(names)].filter(n => !(n in out)).sort();
  rest.forEach((n, i) => (out[n] = free.length ? free[i % free.length] : PALETTE_HEX[i % PALETTE_HEX.length]));
  return out;
}

/** Robot colours: explicit ones (`colors`) plus every robot name seen (`names`), so
 *  each robot of a map's trajectories gets a stable colour. */
export const useRobotColors = create<{colors: Record<string, string>; names: string[]}>(() => ({colors: {}, names: []}));

export function robotColor(name: string): string {
  const {colors, names} = useRobotColors.getState();
  return resolveRobotColors([...names, name], colors)[name];
}

/** Reactive variant for components: re-renders when colours load or robots appear. */
export function useRobotColorOf(): (name: string) => string {
  const colors = useRobotColors(s => s.colors);
  const names = useRobotColors(s => s.names);
  return useMemo(() => {
    const all = resolveRobotColors(names, colors);
    return (name: string) => all[name] ?? resolveRobotColors([...names, name], colors)[name];
  }, [colors, names]);
}
