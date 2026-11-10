import {describe, expect, it} from "vitest";
import {pickRadius, TapTracker, TAP_MAX_MS} from "../src/lib/tap";

const ev = (pointerId: number, pointerType: string, x: number, y: number, t = 0, button = 0) => ({pointerId, pointerType, clientX: x, clientY: y, timeStamp: t, button});

describe("tap vs drag on the map", () => {
  it("a finger may wobble more than a mouse", () => {
    const t = new TapTracker();
    t.onDown(ev(1, "touch", 100, 100));
    expect(t.onUp(ev(1, "touch", 109, 104, 120))).toBe(true);
    t.onDown(ev(2, "mouse", 100, 100));
    expect(t.onUp(ev(2, "mouse", 109, 104, 120))).toBe(false); // a mouse drag
    t.onDown(ev(3, "touch", 100, 100));
    expect(t.onUp(ev(3, "touch", 140, 100, 120))).toBe(false); // a pan
  });
  it("a pinch is never a tap, whichever finger lifts first", () => {
    const t = new TapTracker();
    t.onDown(ev(1, "touch", 100, 100));
    t.onDown(ev(2, "touch", 200, 200, 10));
    expect(t.onUp(ev(2, "touch", 201, 200, 200))).toBe(false);
    expect(t.onUp(ev(1, "touch", 100, 101, 210))).toBe(false);
    // the next single tap counts again
    t.onDown(ev(3, "touch", 50, 50, 300));
    expect(t.onUp(ev(3, "touch", 50, 50, 380))).toBe(true);
  });
  it("a long still press is not a tap; a cancelled one neither", () => {
    const t = new TapTracker();
    t.onDown(ev(1, "touch", 10, 10, 0));
    expect(t.onUp(ev(1, "touch", 10, 10, TAP_MAX_MS + 1))).toBe(false);
    t.onDown(ev(2, "touch", 10, 10, 0));
    t.onCancel({pointerId: 2});
    expect(t.onUp(ev(2, "touch", 10, 10, 50))).toBe(false);
  });
  it("right / middle mouse buttons never tap", () => {
    const t = new TapTracker();
    t.onDown(ev(1, "mouse", 10, 10, 0, 2));
    expect(t.onUp(ev(1, "mouse", 10, 10, 10, 2))).toBe(false);
  });
  it("fingers pick nodes from further away", () => {
    expect(pickRadius("touch")).toBeGreaterThan(pickRadius("mouse"));
  });
});
