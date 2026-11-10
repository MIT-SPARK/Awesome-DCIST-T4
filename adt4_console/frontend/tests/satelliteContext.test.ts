import {describe, expect, it} from "vitest";
import * as THREE from "three";
import {viewFootprint} from "../src/viewer/SatelliteGround";

describe("satellite context imagery", () => {
  it("covers the ground the camera sees, wider when zoomed out", () => {
    const cam = (h: number) => {
      const c = new THREE.PerspectiveCamera(60, 1, 0.1, 1e5);
      c.position.set(10, 20, h);
      c.up.set(0, 1, 0);
      c.lookAt(10, 20, 0);
      c.updateMatrixWorld();
      return c;
    };
    const near = viewFootprint(cam(100), 0);
    const far = viewFootprint(cam(1000), 0);
    const w = (b: {min: number[]; max: number[]}) => b.max[0] - b.min[0];
    expect(w(near)).toBeCloseTo(2 * 100 * Math.tan(Math.PI / 6), 0); // 60 deg fov: about 115 m
    expect(w(far)).toBeGreaterThan(9 * w(near));
    expect((near.min[0] + near.max[0]) / 2).toBeCloseTo(10, 3);
  });
  it("cuts rays toward the horizon at the max range", () => {
    const c = new THREE.PerspectiveCamera(60, 1, 0.1, 1e5);
    c.position.set(0, 0, 2);
    c.lookAt(100, 0, 2); // level: the upper corners never meet the ground
    c.updateMatrixWorld();
    const f = viewFootprint(c, 0, 500);
    expect(f.max[0]).toBeLessThanOrEqual(501);
    expect(f.max[0]).toBeGreaterThan(300);
  });
});
