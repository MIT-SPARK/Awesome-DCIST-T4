import {afterEach, describe, expect, it} from "vitest";
import {PALETTE_HEX, resolveRobotColors, robotColor, shade, useRobotColors} from "../src/lib/palette";

// Mirrors network_diagnostics/config/topology.yaml robots (explicit colours).
const TOPOLOGY = {
  euclid: "#e6194b",
  hamilton: "#42d4f4",
  gauss: "#ffe119",
  lewis: "#3cb44b",
  apollo: "#f032e6",
  samoyed: "#f58231",
};

afterEach(() => useRobotColors.setState({colors: {}, names: []}));

describe("robot colours", () => {
  it("keeps topology colours and never collides", () => {
    const all = resolveRobotColors([...Object.keys(TOPOLOGY), "spot", "sim2"], TOPOLOGY);
    expect(all.hamilton).toBe("#42d4f4");
    expect(new Set(Object.values(all)).size).toBe(Object.keys(all).length);
  });
  it("falls back to the palette by sorted name, skipping used colours (like robot_colors.py)", () => {
    expect(resolveRobotColors(["zed", "alpha"], {})).toEqual({alpha: PALETTE_HEX[0], zed: PALETTE_HEX[1]});
    expect(resolveRobotColors(["alpha", "beta"], {beta: PALETTE_HEX[0].toUpperCase()})).toEqual({alpha: PALETTE_HEX[1], beta: PALETTE_HEX[0]});
  });
  it("robotColor reads the loaded colours store", () => {
    expect(robotColor("alpha")).toBe(PALETTE_HEX[0]);
    useRobotColors.setState({colors: {alpha: "#123456"}, names: ["alpha", "beta"]});
    expect(robotColor("alpha")).toBe("#123456");
    expect(robotColor("beta")).toBe(PALETTE_HEX[0]);
  });
  it("shades toward black and white", () => {
    expect(shade("#804020", 0.5)).toBe("#402010");
    expect(shade("#000000", 1.5)).toBe("#808080");
    expect(shade("#ffffff", 2)).toBe("#ffffff");
    expect(shade("bogus", 0.5)).toBe("bogus");
  });
  it("palette matches robot_colors.py", () => {
    expect(PALETTE_HEX.slice(0, 3)).toEqual(["#e6194b", "#3cb44b", "#ffe119"]);
    expect(PALETTE_HEX).toHaveLength(20);
  });
});
