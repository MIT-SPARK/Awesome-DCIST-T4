import {describe, expect, it} from "vitest";
import {dayOf, nearestIndex, tracksByDay} from "../src/viewer/FrameBrowser";
import {keyframeTracks, type KeyframeTrack} from "../src/stores/viewer";

const track = (key: string, t: number[]): KeyframeTrack => ({robot: {key}, t, syms: t.map((_, i) => `${key}(${i})`), pos: t.map(() => [0, 0, 0])});

describe("frame browser", () => {
  it("groups sessions by the day they start, newest first", () => {
    const d1 = Date.UTC(2025, 9, 22, 15) / 1000;
    const d2 = Date.UTC(2025, 9, 23, 15) / 1000;
    const days = tracksByDay([{src: "s", track: track("a", [d1, d1 + 10])}, {src: "s", track: track("b", [d2])}, {src: "s", track: track("c", [d2 + 5])}]);
    expect(days.map(([d, xs]) => [d, xs.map(x => x.track.robot.key)])).toEqual([[dayOf(d2), ["b", "c"]], [dayOf(d1), ["a"]]]);
    expect(dayOf(12)).toBe("undated"); // index-timed maps
  });
  it("finds the keyframe nearest in time", () => {
    expect(nearestIndex([0, 10, 20], 14)).toBe(1);
    expect(nearestIndex([0, 10, 20], 16)).toBe(2);
    expect(nearestIndex([0, 10, 20], -5)).toBe(0);
    expect(nearestIndex([0, 10, 20], 99)).toBe(2);
  });
  it("splits server points into per-robot keyframe tracks, skipping points without a symbol", () => {
    const pts = [[1, 0, 0, 0], [2, 1, 0, 0], [5, 0, 5, 0], [6, 1, 5, 0]];
    const tracks = keyframeTracks(pts, ["a(0)", null, "b(0)", "b(1)"], [{key: "a", start: 0, session: "s1"}, {key: "b", start: 2}]);
    expect(tracks.map(t => [t.robot.key, t.robot.session, t.syms, t.t])).toEqual([["a", "s1", ["a(0)"], [1]], ["b", undefined, ["b(0)", "b(1)"], [5, 6]]]);
    expect(keyframeTracks(pts, [], [{key: "a", start: 0}])).toEqual([]);
  });
});

import {shortcutOf} from "../src/viewer/shortcuts";
import {enterMapFull, exitMapFull, toggleMapFull, useMapFull} from "../src/viewer/mapFull";

describe("full map", () => {
  it("M toggles it (not while typing); the body class follows", () => {
    const key = (k: string, target: EventTarget | null = null) => ({key: k, ctrlKey: false, metaKey: false, altKey: false, shiftKey: false, target});
    expect(shortcutOf(key("m"))).toBe("fullmap");
    expect(shortcutOf(key("m", document.createElement("input")))).toBeNull();
    enterMapFull();
    expect(useMapFull.getState().on).toBe(true);
    expect(document.body.classList.contains("map-full")).toBe(true);
    toggleMapFull();
    expect(useMapFull.getState().on).toBe(false);
    expect(document.body.classList.contains("map-full")).toBe(false);
    exitMapFull(); // idempotent
    expect(useMapFull.getState().on).toBe(false);
  });
});
