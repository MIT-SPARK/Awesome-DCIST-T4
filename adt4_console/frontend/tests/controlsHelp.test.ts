import {describe, expect, it} from "vitest";
import {COMMON_HELP, LAYOUT_HELP, MODE_HELP, TOP_HINT_KEY, dismissHint, helpSections, hintDismissed, type HelpMode} from "../src/viewer/controlsHelp";

describe("controls help", () => {
  it("every mode has camera entries", () => {
    for (const m of ["3d", "top"] as HelpMode[]) {
      expect(MODE_HELP[m].entries.length).toBeGreaterThan(0);
      for (const e of MODE_HELP[m].entries) {
        expect(e.keys.trim()).not.toBe("");
        expect(e.does.trim()).not.toBe("");
      }
    }
  });
  it("top view documents its rotate controls", () => {
    const keys = MODE_HELP.top.entries.map(e => e.keys).join(" ");
    expect(keys).toMatch(/Right-drag/);
    expect(keys).toMatch(/Q \/ E/);
    expect(keys).toMatch(/\bR\b/);
  });
  it("sections include common help, layout only when asked", () => {
    expect(helpSections("3d", false)).toEqual([MODE_HELP["3d"], ...COMMON_HELP]);
    expect(helpSections("top", true).at(-1)).toBe(LAYOUT_HELP);
  });
  it("key help lists the shortcuts that exist", () => {
    const keys = COMMON_HELP.flatMap(s => s.entries.map(e => e.keys)).join(" | ");
    for (const k of ["Enter", "Esc", "P", "M"]) expect(keys).toContain(k);
  });
  it("hint dismissal survives broken storage", () => {
    const m = new Map<string, string>();
    const s = {getItem: (k: string) => m.get(k) ?? null, setItem: (k: string, v: string) => void m.set(k, v)};
    expect(hintDismissed(s)).toBe(false);
    dismissHint(s);
    expect(m.get(TOP_HINT_KEY)).toBe("1");
    expect(hintDismissed(s)).toBe(true);
    const bad = {getItem: () => { throw new Error("x"); }, setItem: () => { throw new Error("x"); }};
    expect(hintDismissed(bad)).toBe(false);
    expect(() => dismissHint(bad)).not.toThrow();
    expect(hintDismissed(null)).toBe(false);
  });
});
