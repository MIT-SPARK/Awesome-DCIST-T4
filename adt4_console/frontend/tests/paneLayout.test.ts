import {describe, expect, it} from "vitest";
import {
  PANE_TABS,
  LAYOUT_KEY,
  LEFT,
  MIN_CANVAS,
  RIGHT,
  TITLE_H,
  WIN_MIN,
  clampRect,
  clampWidths,
  clearLayout,
  defaultLayout,
  dock,
  loadLayout,
  popOut,
  raise,
  resizeRect,
  sanitizeLayout,
  saveLayout,
  snapRect,
  updateWin,
} from "../src/viewer/paneLayout";

function memStore() {
  const m = new Map<string, string>();
  return {
    getItem: (k: string) => m.get(k) ?? null,
    setItem: (k: string, v: string) => void m.set(k, v),
    removeItem: (k: string) => void m.delete(k),
    m,
  };
}
const throwing = {
  getItem: () => {
    throw new Error("SecurityError");
  },
  setItem: () => {
    throw new Error("QuotaExceeded");
  },
  removeItem: () => {
    throw new Error("SecurityError");
  },
};

describe("pane layout persistence", () => {
  it("round-trips through storage", () => {
    const s = memStore();
    let l = popOut(defaultLayout(), "images", 1600, 900);
    l = {...l, leftW: 333, rightW: 444};
    expect(saveLayout(l, s)).toBe(true);
    expect(loadLayout(s)).toEqual(l);
  });

  it("falls back to defaults when storage is missing, throws or holds junk", () => {
    expect(loadLayout(null)).toEqual(defaultLayout());
    expect(loadLayout(throwing)).toEqual(defaultLayout());
    expect(saveLayout(defaultLayout(), throwing)).toBe(false);
    expect(saveLayout(defaultLayout(), null)).toBe(false);
    expect(() => clearLayout(throwing)).not.toThrow();
    const s = memStore();
    s.m.set(LAYOUT_KEY, "{not json");
    expect(loadLayout(s)).toEqual(defaultLayout());
    s.m.set(LAYOUT_KEY, "42");
    expect(loadLayout(s)).toEqual(defaultLayout());
  });

  it("clear removes the stored layout", () => {
    const s = memStore();
    saveLayout({...defaultLayout(), leftW: 300}, s);
    clearLayout(s);
    expect(loadLayout(s)).toEqual(defaultLayout());
  });

  it("sanitizes stored values", () => {
    const l = sanitizeLayout({
      leftW: 5,
      rightW: 99999,
      windows: {
        images: {x: 10.4, y: 20, w: 10, h: 10, min: "yes"},
        inspect: {x: "a", y: 0, w: 300, h: 300},
        bogus: {x: 0, y: 0, w: 300, h: 300},
      },
      order: ["bogus", "images", "images", "plan"],
    });
    expect(l.leftW).toBe(LEFT.min);
    expect(l.rightW).toBe(RIGHT.max);
    expect(Object.keys(l.windows)).toEqual(["images"]);
    expect(l.windows.images).toEqual({x: 10, y: 20, w: WIN_MIN.w, h: WIN_MIN.h, min: false});
    expect(l.order).toEqual(["images"]);
  });

  it("adds windows missing from the stored order", () => {
    const l = sanitizeLayout({windows: {map: {x: 0, y: 0, w: 300, h: 300}, images: {x: 0, y: 0, w: 300, h: 300}}, order: ["map"]});
    expect(l.order).toEqual(["map", "images"]);
  });
});

describe("splitter width clamping", () => {
  it("keeps each panel within its bounds", () => {
    expect(clampWidths(10, 10, 3000, true, true)).toEqual({leftW: LEFT.min, rightW: RIGHT.min});
    expect(clampWidths(9999, 9999, 3000, true, true)).toEqual({leftW: LEFT.max, rightW: RIGHT.max});
  });

  it("leaves the canvas MIN_CANVAS, the preferred panel keeping its width", () => {
    const total = 1200;
    const r = clampWidths(500, 600, total, true, true, "right");
    expect(r.rightW).toBe(600);
    expect(clampWidths(500, 700, total, true, true, "right").rightW).toBe(total - MIN_CANVAS - LEFT.min);
    expect(r.leftW + r.rightW).toBeLessThanOrEqual(total - MIN_CANVAS);
    const l = clampWidths(500, 700, total, true, true, "left");
    expect(l.leftW).toBe(500);
    expect(l.leftW + l.rightW).toBeLessThanOrEqual(total - MIN_CANVAS);
  });

  it("ignores a closed panel", () => {
    expect(clampWidths(280, 900, 1100, false, true).rightW).toBe(1100 - MIN_CANVAS);
    expect(clampWidths(560, 900, 1100, true, false).leftW).toBe(560);
  });

  it("never goes below the minimum even on a tiny viewer", () => {
    const r = clampWidths(300, 400, 500, true, true);
    expect(r.leftW).toBe(LEFT.min);
    expect(r.rightW).toBe(RIGHT.min);
  });
});

describe("floating window geometry", () => {
  it("clamps windows inside the area", () => {
    expect(clampRect({x: -50, y: -10, w: 300, h: 200}, 1000, 800)).toEqual({x: 0, y: 0, w: 300, h: 200});
    expect(clampRect({x: 900, y: 700, w: 300, h: 200}, 1000, 800)).toEqual({x: 700, y: 600, w: 300, h: 200});
  });

  it("shrinks windows larger than the area", () => {
    expect(clampRect({x: 100, y: 100, w: 2000, h: 1500}, 1000, 800)).toEqual({x: 0, y: 0, w: 1000, h: 800});
  });

  it("only needs the title bar inside when minimized", () => {
    expect(clampRect({x: 0, y: 790, w: 300, h: 400}, 1000, 800, true).y).toBe(800 - TITLE_H);
  });

  it("snaps near edges", () => {
    expect(snapRect({x: 9, y: 785, w: 300, h: 10}, 1000, 800)).toEqual({x: 0, y: 790, w: 300, h: 10});
    expect(snapRect({x: 690, y: 50, w: 300, h: 200}, 1000, 800)).toEqual({x: 700, y: 50, w: 300, h: 200});
    expect(snapRect({x: 100, y: 100, w: 300, h: 200}, 1000, 800)).toEqual({x: 100, y: 100, w: 300, h: 200});
  });

  it("resizes from edges and corners with a fixed opposite edge", () => {
    const s = {x: 100, y: 100, w: 400, h: 300};
    expect(resizeRect(s, "se", 50, 20, 1000, 800)).toEqual({x: 100, y: 100, w: 450, h: 320});
    expect(resizeRect(s, "nw", -30, -40, 1000, 800)).toEqual({x: 70, y: 60, w: 430, h: 340});
    // min size: the right edge stays at 500
    expect(resizeRect(s, "w", 1000, 0, 1000, 800)).toEqual({x: 500 - WIN_MIN.w, y: 100, w: WIN_MIN.w, h: 300});
    // area bounds
    expect(resizeRect(s, "e", 5000, 0, 1000, 800).w).toBe(900);
    expect(resizeRect(s, "n", 0, -5000, 1000, 800)).toEqual({x: 100, y: 0, w: 400, h: 400});
  });
});

describe("pop out / dock / raise", () => {
  it("pops out inside the area, cascading, and remembers the rectangle", () => {
    let l = popOut(defaultLayout(), "images", 1600, 900);
    const cam = l.windows.images!;
    expect(cam.x + cam.w).toBeLessThanOrEqual(1600);
    expect(cam.y + cam.h).toBeLessThanOrEqual(900);
    l = popOut(l, "inspect", 1600, 900);
    expect(l.order).toEqual(["images", "inspect"]);
    expect(l.windows.inspect!.x).not.toBe(cam.x);
    l = updateWin(l, "images", {x: 5, y: 6});
    const docked = dock(l, "images");
    expect(docked.windows.images).toBeUndefined();
    expect(docked.order).toEqual(["inspect"]);
    // popping out again centres on the pointer when given one
    const again = popOut(docked, "images", 1600, 900, {x: 800, y: 400});
    expect(again.windows.images!.x).toBe(800 - again.windows.images!.w / 2);
    expect(again.order).toEqual(["inspect", "images"]);
  });

  it("raise moves a window to the front and is a no-op for docked tabs", () => {
    let l = popOut(popOut(defaultLayout(), "images", 1600, 900), "inspect", 1600, 900);
    l = raise(l, "images");
    expect(l.order).toEqual(["inspect", "images"]);
    expect(raise(l, "map")).toBe(l);
    expect(raise(l, "images")).toBe(l);
    expect(updateWin(l, "map", {x: 1})).toBe(l);
    expect(dock(l, "map")).toBe(l);
  });
});

describe("tabs", () => {
  it("are the viewer's three side-panel tabs; stored windows of dropped tabs are ignored", () => {
    expect(PANE_TABS).toEqual(["inspect", "images", "map"]);
    const l = sanitizeLayout({leftW: 300, rightW: 400, windows: {camera: {x: 0, y: 0, w: 300, h: 300}}, order: ["camera"], drive: true});
    expect(l).toEqual({...defaultLayout(), leftW: 300, rightW: 400});
  });
});
