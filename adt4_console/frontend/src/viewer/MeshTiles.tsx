// Progressive mesh-tile streaming: one BufferGeometry per tile. A coarse level
// of every tile comes first (the whole map for a few hundred kB), then the
// tiles nearest the camera target are refined to the wanted detail, with a
// concurrency limit; tiles whose etag changed are refetched (lib/meshPlan.ts).
// Byte progress goes to the load bar (stores/loading.ts).
//
// Ceilings: RViz (kimera_pgmo_rviz MeshDisplay, `Cull Backfaces: true`) hides them
// because ceiling triangles face down into the room, so from above only their
// back faces show. Same here: FrontSide material (three.js and kimera_pgmo both
// wind front faces counter-clockwise). An optional clipping plane cuts the mesh
// above a height over the floor for views from inside or at grazing angles.
import {useEffect, useRef, useState} from "react";
import * as THREE from "three";
import {useFrame, useThree} from "@react-three/fiber";
import {decodePackedTile, decodeTile, unpack, type DecodedTile} from "../codec";
import {fetchWithProgress} from "../lib/api";
import {manifestLevels, nextStep, planTile, resolveLevel, tileRow, wanted, type Shown, type TileStep} from "../lib/meshPlan";
import {beginTask, dropTask, endTask, progressTask} from "../stores/loading";
import type {TileManifest} from "../types";
import {srcPath} from "../stores/viewer";
import type {MeshColorMode} from "./style";

/** GLSL turbo-like colormap (polynomial fit, Google "Turbo"), t in 0..1 -> sRGB. */
const TURBO_GLSL = `
vec3 adt4Turbo(float t) {
  t = clamp(t, 0.0, 1.0);
  const vec4 kr = vec4(0.13572138, 4.61539260, -42.66032258, 132.13108234);
  const vec4 kg = vec4(0.09140261, 2.19418839, 4.84296658, -14.18503333);
  const vec4 kb = vec4(0.10667330, 12.64194608, -60.58204836, 110.36276771);
  const vec2 kr2 = vec2(-152.94239396, 59.28637943);
  const vec2 kg2 = vec2(4.27729857, 2.82956604);
  const vec2 kb2 = vec2(-89.90310912, 27.34824973);
  vec4 v4 = vec4(1.0, t, t * t, t * t * t);
  vec2 v2 = v4.zw * v4.z;
  return vec3(dot(v4, kr) + dot(v2, kr2), dot(v4, kg) + dot(v2, kg2), dot(v4, kb) + dot(v2, kb2));
}
`;
const MESH_MODE: Record<MeshColorMode, number> = {rgb: 0, uniform: 1, height: 2};

/** queue key of the coarse-level bundle step (not a tile key) */
const BUNDLE = "*bundle";

// requests in flight: each costs a round trip before its bytes flow, so on a
// high-latency link (ssh tunnel) fewer would leave the link idle between tiles
const CONCURRENCY = 6;

export interface TileStats {
  /** tiles shown at the wanted detail */
  loaded: number;
  /** tiles that have faces at the wanted detail */
  total: number;
  triangles: number;
}

class TileLoader {
  group = new THREE.Group();
  private have = new Map<string, Shown & {mesh: THREE.Mesh}>();
  /** pending fetches per tile, in order (a coarse preview, then the target) */
  private queue = new Map<string, TileStep[]>();
  private inflight = new Map<string, TileStep>();
  private centers = new Map<string, THREE.Vector3>();
  private retries = new Map<string, number>();
  private bundleFailed = new Set<string>();
  private manifest: TileManifest = {tile_size: 8, tiles: {}};
  private want = 0;
  private target = 0;
  private abort = new AbortController();
  private disposed = false;
  // progress of the current batch of fetches (load bar)
  private taskId: string;
  private batchOn = false;
  private hasManifest = false;
  private batchDone = 0;
  private partial = new Map<string, number>();
  material: THREE.MeshLambertMaterial;
  /** keeps z <= floor + height when clipping is on, in the tiles' own frame */
  localClip = new THREE.Plane(new THREE.Vector3(0, 0, -1), 0);
  /** `localClip` in world space (three.js clipping planes are world space) */
  clipPlane = new THREE.Plane(new THREE.Vector3(0, 0, -1), 0);
  clipHeight: number | null = null;
  private tileMinZ = new Map<string, number>();
  private tileMaxZ = new Map<string, number>();
  /** shader uniforms for the colour mode (rgb / uniform / height colormap) */
  uniforms = {
    uMeshMode: {value: 0},
    uMeshColor: {value: new THREE.Color("#b0b0b0")},
    uZMin: {value: 0},
    uZMax: {value: 3},
  };
  triangles = 0;

  constructor(
    private src: string,
    private camTarget: () => THREE.Vector3,
    private onChange: () => void,
  ) {
    this.taskId = `mesh:${src}`;
    this.material = new THREE.MeshLambertMaterial({vertexColors: true, side: THREE.FrontSide});
    // tile colours are sRGB bytes; three treats vertex colours as linear, so decode here
    // colour mode: 0 vertex RGB, 1 uniform colour, 2 height colormap (world z)
    this.material.onBeforeCompile = shader => {
      Object.assign(shader.uniforms, this.uniforms);
      shader.vertexShader =
        "uniform int uMeshMode;\nuniform vec3 uMeshColor;\nuniform float uZMin;\nuniform float uZMax;\n" +
        TURBO_GLSL +
        shader.vertexShader.replace(
          "#include <color_vertex>",
          [
            "#include <color_vertex>",
            "#ifdef USE_COLOR",
            "  if (uMeshMode == 1) vColor.rgb = uMeshColor;",
            "  else if (uMeshMode == 2) vColor.rgb = pow(adt4Turbo(((modelMatrix * vec4(position, 1.0)).z - uZMin) / max(uZMax - uZMin, 1e-3)), vec3(2.2));",
            "  else vColor.rgb = pow(vColor.rgb, vec3(2.2));",
            "#endif",
          ].join("\n"),
        );
    };
    this.group.name = `mesh:${src}`;
  }

  setManifest(m: TileManifest | null) {
    this.manifest = m ?? {tile_size: 8, tiles: {}};
    this.hasManifest = true;
    const size = this.manifest.tile_size ?? 8;
    this.centers.clear();
    for (const [k, t] of Object.entries(this.manifest.tiles)) {
      const o = t.origin ?? (k.split("_").map(v => Number(v) * size) as [number, number, number]);
      this.centers.set(k, new THREE.Vector3(o[0] + size / 2, o[1] + size / 2, o[2] + size / 2));
    }
    this.replan();
  }

  /** Wanted level of detail (lib/meshPlan DETAIL_LEVEL; the manifest's
   *  nearest offered level is used). */
  setWant(level: number) {
    if (level === this.want) return;
    this.want = level;
    this.replan();
  }

  private replan() {
    const m = this.manifest;
    this.target = resolveLevel(manifestLevels(m), this.want);
    for (const [k, t] of this.have) if (!m.tiles[k]) this.drop(k, t.mesh);
    this.queue.clear();
    // the coarse level comes as one bundle when the server offers it (one
    // round trip instead of hundreds); per tile if the bundle failed
    const bundle = m.bundles?.find(b => !this.bundleFailed.has(b.etag));
    let previews = 0;
    for (const k of Object.keys(m.tiles)) {
      const {steps, clear} = planTile(m, k, this.target, this.have.get(k));
      const cur = this.have.get(k);
      if (clear && cur) this.drop(k, cur.mesh);
      // the step in flight still counts: don't queue it twice
      const f = this.inflight.get(k);
      let rest = f ? steps.filter(s => !(s.level === f.level && s.etag === f.etag)) : steps;
      // a tile not shown yet gets its bundle-level step (preview or target) from the bundle
      if (bundle && !cur && rest[0]?.level === bundle.level) {
        rest = rest.slice(1);
        previews++;
      }
      if (rest.length) this.queue.set(k, rest);
    }
    const flying = this.inflight.get(BUNDLE);
    if (bundle && previews && !(flying && flying.etag === bundle.etag)) {
      this.queue.set(BUNDLE, [{key: BUNDLE, level: bundle.level, bytes: bundle.bytes, etag: bundle.etag, preview: bundle.level !== this.target}]);
    }
    this.progress();
    this.onChange();
    this.pump();
  }

  /** Byte progress of the batch: done + in flight + queued. */
  private progress() {
    const queued = [...this.queue.values()].reduce((n, st) => n + st.reduce((a, s) => a + s.bytes, 0), 0);
    const flying = [...this.inflight.values()].reduce((n, s) => n + s.bytes, 0);
    const busy = this.queue.size > 0 || this.inflight.size > 0;
    const sized = this.manifest.enc === 2;
    if (!this.hasManifest) return;
    if (busy) {
      const partial = [...this.partial.values()].reduce((n, v) => n + v, 0);
      if (!this.batchOn) {
        this.batchOn = true;
        this.batchDone = 0;
        beginTask(this.taskId, "Mesh", sized ? "bytes" : "count");
      }
      if (sized) progressTask(this.taskId, this.batchDone + partial, this.batchDone + flying + queued);
      else {
        const st = this.stats();
        progressTask(this.taskId, st.loaded, st.total);
      }
    } else {
      // also ends the viewer store's "preparing" task when there is nothing to fetch
      this.batchOn = false;
      endTask(this.taskId);
    }
  }

  setColorMode(mode: MeshColorMode, color: string) {
    this.uniforms.uMeshMode.value = MESH_MODE[mode] ?? 0;
    this.uniforms.uMeshColor.value.set(color);
    this.updateZRange();
  }

  /** Height colormap range: floor .. a high percentile of tile tops (clip height when clipping). */
  private updateZRange() {
    const lo = this.floorZ();
    const tops = [...this.tileMaxZ.values()].sort((a, b) => a - b);
    let hi = tops.length ? tops[Math.floor(tops.length * 0.9)] : lo + 3;
    if (this.clipHeight !== null) hi = Math.min(hi, lo + this.clipHeight);
    this.uniforms.uZMin.value = lo;
    this.uniforms.uZMax.value = Math.max(hi, lo + 0.5);
  }

  /** Floor height: low percentile of the tiles' lowest vertices (robust to pits). */
  floorZ(): number {
    const zs = [...this.tileMinZ.values()].sort((a, b) => a - b);
    return zs.length ? zs[Math.floor(zs.length * 0.1)] : 0;
  }

  /** Re-derive the world clip plane from the group's current world matrix. */
  syncClip() {
    this.group.updateWorldMatrix(true, false);
    this.clipPlane.copy(this.localClip).applyMatrix4(this.group.matrixWorld);
  }

  setCull(on: boolean) {
    const side = on ? THREE.FrontSide : THREE.DoubleSide;
    if (this.material.side !== side) {
      this.material.side = side;
      this.material.needsUpdate = true;
    }
  }

  setClip(height: number | null) {
    this.clipHeight = height;
    this.updateClip();
    this.updateZRange();
  }

  private updateClip() {
    if (this.clipHeight === null) {
      if (this.material.clippingPlanes?.length) {
        this.material.clippingPlanes = [];
        this.material.needsUpdate = true;
      }
      return;
    }
    this.localClip.constant = this.floorZ() + this.clipHeight;
    this.syncClip();
    if (!this.material.clippingPlanes?.length) {
      this.material.clippingPlanes = [this.clipPlane];
      this.material.needsUpdate = true;
    }
  }

  private drop(k: string, mesh: THREE.Mesh) {
    this.tileMinZ.delete(k);
    this.tileMaxZ.delete(k);
    this.group.remove(mesh);
    this.triangles -= mesh.geometry.index ? mesh.geometry.index.count / 3 : 0;
    mesh.geometry.dispose();
    this.have.delete(k);
  }

  private pump() {
    while (!this.disposed && this.inflight.size < CONCURRENCY && this.queue.size) {
      const target = this.camTarget();
      const heads: TileStep[] = [];
      for (const [k, st] of this.queue) if (!this.inflight.has(k)) heads.push(st[0]);
      if (!heads.length) break;
      const i = nextStep(heads, k => this.centers.get(k)?.distanceToSquared(target) ?? Infinity);
      const step = heads[i];
      const rest = this.queue.get(step.key)!.slice(1);
      if (rest.length) this.queue.set(step.key, rest);
      else this.queue.delete(step.key);
      this.inflight.set(step.key, step);
      void this.fetch(step);
    }
  }

  private tileUrl(step: TileStep): string {
    if (step.key === BUNDLE) return `${srcPath(this.src)}/mesh/bundle/${step.level}?h=${encodeURIComponent(step.etag)}`;
    const base = `${srcPath(this.src)}/mesh/tile/${encodeURIComponent(step.key)}`;
    if (this.manifest.enc !== 2) return base; // legacy manifest: full tiles, plain encoding
    return `${base}?enc=2&lod=${step.level}&h=${encodeURIComponent(step.etag)}`;
  }

  private async fetch(step: TileStep) {
    const key = step.key;
    let ok = false;
    try {
      const body = await fetchWithProgress(this.tileUrl(step), {
        signal: this.abort.signal,
        accept: "application/x-msgpack",
        onBytes: b => {
          this.partial.set(key, Math.min(b, step.bytes || b));
          this.progress();
        },
      });
      if (this.disposed) return;
      const raw = unpack(body);
      if (key === BUNDLE) {
        ok = true;
        // still this manifest's bundle: show each tile that still wants its preview
        if (!this.manifest.bundles?.some(b => b.etag === step.etag)) return;
        for (const o of (raw as unknown[]) ?? []) {
          const t = decodePackedTile(o);
          const row = tileRow(this.manifest, t.key, step.level);
          if (!row) continue;
          const one: TileStep = {key: t.key, level: step.level, bytes: 0, etag: row[1], preview: step.level !== this.target};
          if (!this.inflight.has(t.key) && wanted(this.manifest, one, this.target, this.have.get(t.key))) this.show(t.key, one, t);
        }
        return;
      }
      const t: DecodedTile = this.manifest.enc === 2 ? decodePackedTile(raw) : decodeTile(raw);
      ok = true;
      if (!wanted(this.manifest, step, this.target, this.have.get(key))) return;
      this.show(key, step, t);
    } catch {
      if (key === BUNDLE) {
        // fall back to per-tile previews
        this.bundleFailed.add(step.etag);
        this.inflight.delete(key);
        if (!this.disposed) this.replan();
        return;
      }
      // retried twice at the back of the queue, then left to a later manifest refresh
      const id = `${key}/${step.level}/${step.etag}`;
      const n = (this.retries.get(id) ?? 0) + 1;
      this.retries.set(id, n);
      if (n <= 2 && !this.disposed && !this.queue.has(key)) this.queue.set(key, [step]);
    } finally {
      this.inflight.delete(key);
      this.partial.delete(key);
      if (!this.disposed) {
        if (ok) this.batchDone += step.bytes;
        this.progress();
        this.onChange();
        this.pump();
      }
    }
  }

  private show(key: string, step: TileStep, t: DecodedTile) {
    const g = new THREE.BufferGeometry();
    g.setAttribute("position", new THREE.BufferAttribute(t.positions, 3));
    g.setAttribute("color", new THREE.BufferAttribute(t.colors, 3, true));
    g.setIndex(new THREE.BufferAttribute(t.indices, 1));
    g.computeVertexNormals();
    g.computeBoundingSphere();
    g.computeBoundingBox();
    const mesh = new THREE.Mesh(g, this.material);
    mesh.name = `tile:${key}`;
    mesh.userData.tileKey = key;
    mesh.userData.lod = step.level;
    mesh.matrixAutoUpdate = false;
    const old = this.have.get(key);
    if (old) this.drop(key, old.mesh);
    this.group.add(mesh);
    this.have.set(key, {level: step.level, etag: step.etag, mesh});
    if (g.boundingBox && Number.isFinite(g.boundingBox.min.z)) {
      this.tileMinZ.set(key, g.boundingBox.min.z);
      this.tileMaxZ.set(key, g.boundingBox.max.z);
    }
    this.updateZRange();
    if (this.clipHeight !== null) this.updateClip();
    this.triangles += t.indices.length / 3;
  }

  stats(): TileStats {
    let loaded = 0;
    let total = 0;
    for (const k of Object.keys(this.manifest.tiles)) {
      const row = tileRow(this.manifest, k, this.target);
      if (!row) continue; // no faces at this level
      total++;
      const h = this.have.get(k);
      if (h && h.level === this.target && h.etag === row[1]) loaded++;
    }
    return {loaded, total, triangles: this.triangles};
  }

  dispose() {
    this.disposed = true;
    this.abort.abort();
    for (const [k, t] of this.have) this.drop(k, t.mesh);
    this.material.dispose();
    if (this.batchOn) dropTask(this.taskId);
  }
}

export function MeshTiles({src, manifest, manifestRev, detail = 0, opacity, cullBackfaces = true, clipHeight = null, colorMode = "rgb", uniformColor = "#b0b0b0", onStats}: {
  src: string;
  manifest: TileManifest | null;
  manifestRev: number;
  /** wanted level of detail: 0 full, 1 medium, 2 coarse (lib/meshPlan DETAIL_LEVEL) */
  detail?: number;
  opacity: number;
  /** RViz MeshDisplay `Cull Backfaces` */
  cullBackfaces?: boolean;
  /** metres above the floor; null = no clipping */
  clipHeight?: number | null;
  colorMode?: MeshColorMode;
  uniformColor?: string;
  onStats?: (s: TileStats) => void;
}) {
  const {controls, invalidate, camera} = useThree() as unknown as {
    controls: {getTarget?: (v: THREE.Vector3) => THREE.Vector3} | null;
    invalidate: () => void;
    camera: THREE.Camera;
  };
  const ctlRef = useRef(controls);
  ctlRef.current = controls;
  const statsRef = useRef(onStats);
  statsRef.current = onStats;
  // Created in an effect (not useMemo) so StrictMode's mount/unmount/mount cycle
  // disposes the first loader and builds a fresh one instead of reusing a dead one.
  const [loader, setLoader] = useState<TileLoader | null>(null);
  useEffect(() => {
    const l: TileLoader = new TileLoader(
      src,
      () => {
        const c = ctlRef.current;
        const t = c?.getTarget ? c.getTarget(new THREE.Vector3()) : camera.position.clone();
        // tiles are in the source's own frame
        l.group.updateWorldMatrix(true, false);
        return l.group.worldToLocal(t);
      },
      () => {
        invalidate();
        statsRef.current?.(l.stats());
      },
    );
    l.group.userData.meshGroup = true;
    setLoader(l);
    return () => {
      l.dispose();
      invalidate();
    };
  }, [src, camera, invalidate]);
  useEffect(() => {
    loader?.setWant(detail);
  }, [loader, detail]);
  useEffect(() => {
    loader?.setManifest(manifest);
  }, [loader, manifest, manifestRev]);
  useEffect(() => {
    if (!loader) return;
    loader.material.transparent = opacity < 1;
    loader.material.opacity = opacity;
    loader.material.depthWrite = opacity >= 0.5;
    loader.material.needsUpdate = true;
    invalidate();
  }, [loader, opacity, invalidate]);
  useEffect(() => {
    if (!loader) return;
    loader.setCull(cullBackfaces);
    loader.setClip(clipHeight);
    invalidate();
  }, [loader, cullBackfaces, clipHeight, invalidate]);
  useEffect(() => {
    if (!loader) return;
    loader.setColorMode(colorMode, uniformColor);
    invalidate();
  }, [loader, colorMode, uniformColor, invalidate]);
  // follow the alignment group (it glides on loop closures) with the clip plane
  useFrame(() => {
    if (loader && loader.clipHeight !== null) loader.syncClip();
  });
  if (!loader) return null;
  // picking is done by viewer/Scene.tsx Picker (on click only), so no R3F handlers here:
  // per-pointermove raycasts against ~1.6M triangles would stall the frame.
  return <primitive object={loader.group} />;
}
