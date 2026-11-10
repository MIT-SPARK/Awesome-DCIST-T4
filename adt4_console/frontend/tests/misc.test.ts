import {describe, expect, it} from "vitest";
import {ApiError, errorText} from "../src/lib/api";
import {fuzzyFilter} from "../src/lib/fuzzy";
import {labelColor} from "../src/lib/palette";

describe("api errors", () => {
  it("503s say which service is unavailable", () => {
    expect(errorText(new ApiError(503, "neo4j bolt://x: refused"))).toBe("Unavailable: neo4j bolt://x: refused");
    expect(errorText(new ApiError(503, ""))).toBe("Service unavailable");
    expect(errorText(new ApiError(502, "bad gateway"))).toBe("Backend unreachable");
    expect(new ApiError(404, "no such node").kind).toBe("notfound");
  });
});

describe("misc", () => {
  it("fuzzy filter ranks substring matches first", () => {
    expect(fuzzyFilter(["spot_relocalize", "spot_mapping", "phoenix_mapping"], "map", s => s)).toEqual(["spot_mapping", "phoenix_mapping"]);
    expect(fuzzyFilter(["spot_mapping", "base_station"], "sptmp", s => s)).toEqual(["spot_mapping"]);
  });
  it("label palette is deterministic", () => {
    expect(labelColor(5)).toEqual(labelColor(5));
    expect(labelColor(-1)).toEqual([200, 200, 200]);
  });
});
