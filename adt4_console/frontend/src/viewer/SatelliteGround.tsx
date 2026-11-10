// Satellite imagery under the map (API.md §14): XYZ tiles from the console's
// cache placed in the map frame through the GPS anchor's affine fit. Drawn
// just under the lowest graph point so the mesh and graph stay on top in 3D.
// Off unless the viewer turns it on (stores/geo.ts).
import {useEffect, useMemo, useRef, useState} from "react";
import * as THREE from "three";
import {useFrame, useThree} from "@react-three/fiber";
import {withToken} from "../lib/auth";
import {fetchWithProgress} from "../lib/api";
import {lonLatToTile, pickZoom, tilesForBox, type TileLayer, type TileQuad} from "../lib/geo";
import {useGeo} from "../stores/geo";
import {beginTask, dropTask, endTask, progressTask} from "../stores/loading";

const MARGIN_M = 60;
// context imagery around the map: the ground in view, coarser, at most this many tiles
const CONTEXT_TILES = 48;
// how far out the view footprint reaches (rays toward the horizon are cut here), metres
const CONTEXT_MAX_M = 4000;
// close-up imagery: zoomed in past the map-wide detail zoom, the ground in view at up to the
// layer's zmax, at most this many tiles
const CLOSEUP_TILES = 64;

/** The map-frame box of the ground (plane ``z``) the camera sees: its four view
 *  corners cast onto the plane, rays that miss it (or reach past ``maxM``) cut at
 *  ``maxM`` from the camera. */
export function viewFootprint(camera: THREE.Camera, z: number, maxM = CONTEXT_MAX_M): {min: number[]; max: number[]} {
  const plane = new THREE.Plane(new THREE.Vector3(0, 0, 1), -z);
  const ray = new THREE.Raycaster();
  const hit = new THREE.Vector3();
  const xs: number[] = [];
  const ys: number[] = [];
  for (const [u, v] of [[-1, -1], [1, -1], [1, 1], [-1, 1], [0, 0]] as const) {
    ray.setFromCamera(new THREE.Vector2(u, v), camera);
    const o = ray.ray.origin;
    let p = ray.ray.intersectPlane(plane, hit);
    if (!p || p.distanceTo(o) > maxM) {
      const d = ray.ray.direction.clone().setZ(0);
      if (d.lengthSq() < 1e-9) d.set(1, 0, 0);
      p = hit.copy(o).addScaledVector(d.normalize(), maxM);
    }
    xs.push(p.x);
    ys.push(p.y);
  }
  return {min: [Math.min(...xs), Math.min(...ys)], max: [Math.max(...xs), Math.max(...ys)]};
}

/** Tiles of ``layer`` the cache can have (offline: inside its manifest's box). */
function servable(tiles: TileQuad[], layer: TileLayer, z: number): TileQuad[] {
  const bb = layer.bbox_lonlat;
  if (layer.live || !bb) return tiles;
  const [x0, y1] = lonLatToTile(bb[0], bb[1], z);
  const [x1, y0] = lonLatToTile(bb[2], bb[3], z);
  return tiles.filter(q => q.x >= Math.floor(x0) && q.x <= Math.floor(x1) && q.y >= Math.floor(y0) && q.y <= Math.floor(y1));
}

// imagery on the load bar (stores/loading): tiles in / tiles wanted
const IMAGERY_TASK = "imagery";
const imagery = {want: new Set<string>(), got: new Set<string>()};
function imageryWant(keys: string[]) {
  imagery.want = new Set(keys);
  imagery.got = new Set([...imagery.got].filter(k => imagery.want.has(k)));
  if (!keys.length || imagery.got.size >= keys.length) return dropTask(IMAGERY_TASK);
  beginTask(IMAGERY_TASK, "Imagery", "count", keys.length);
  progressTask(IMAGERY_TASK, imagery.got.size);
}
function imageryGot(key: string) {
  if (!imagery.want.has(key) || imagery.got.has(key)) return;
  imagery.got.add(key);
  progressTask(IMAGERY_TASK, imagery.got.size);
  if (imagery.got.size >= imagery.want.size) endTask(IMAGERY_TASK);
}

function Tile({q, layer, z, opacity, counted = true, order}: {q: TileQuad; layer: string; z: number; opacity: number; counted?: boolean; order?: number}) {
  const invalidate = useThree(s => s.invalidate);
  const [tex, setTex] = useState<THREE.Texture | null>(null);
  useEffect(() => {
    // fetched (not TextureLoader) so the bytes count on the load bar's throughput meter
    const ctl = new AbortController();
    let t: THREE.Texture | null = null;
    void (async () => {
      try {
        const body = await fetchWithProgress(withToken(`api/geo/tiles/${layer}/${q.z}/${q.x}/${q.y}`), {signal: ctl.signal});
        const bmp = await createImageBitmap(new Blob([body as BlobPart]), {imageOrientation: "flipY"});
        if (ctl.signal.aborted) return bmp.close();
        const tx = new THREE.Texture(bmp);
        tx.flipY = false; // flipped while decoding (ImageBitmaps ignore flipY)
        tx.colorSpace = THREE.SRGBColorSpace;
        tx.anisotropy = 4;
        tx.needsUpdate = true;
        t = tx;
        setTex(tx);
        invalidate();
      } catch {
        // not cached (and no live fetching): leave the hole
      } finally {
        if (counted) imageryGot(q.key);
      }
    })();
    return () => {
      ctl.abort();
      t?.dispose();
      (t?.image as ImageBitmap | undefined)?.close?.();
    };
  }, [layer, q.z, q.x, q.y, invalidate, counted]);
  const geom = useMemo(() => {
    const g = new THREE.BufferGeometry();
    const c = q.corners; // NW NE SE SW
    g.setAttribute("position", new THREE.Float32BufferAttribute([c[0][0], c[0][1], z, c[1][0], c[1][1], z, c[2][0], c[2][1], z, c[3][0], c[3][1], z], 3));
    g.setAttribute("uv", new THREE.Float32BufferAttribute([0, 1, 1, 1, 1, 0, 0, 0], 2));
    g.setIndex([0, 3, 1, 1, 3, 2]);
    return g;
  }, [q, z]);
  useEffect(() => () => geom.dispose(), [geom]);
  if (!tex) return null;
  return (
    <mesh geometry={geom} renderOrder={order ?? (counted ? -10 : -11)}>
      <meshBasicMaterial map={tex} transparent={opacity < 1} opacity={opacity} depthWrite={false} side={THREE.DoubleSide} toneMapped={false} />
    </mesh>
  );
}

export function SatelliteGround({floorZ}: {floorZ: number | null}) {
  const geo = useGeo(s => s.geo);
  const on = useGeo(s => s.satellite);
  const opacity = useGeo(s => s.satOpacity);
  const layers = useGeo(s => s.layers);
  const bounds = useGeo(s => s.bounds);
  const layer = layers.find(l => l.default) ?? layers.find(l => l.id === "esri_world_imagery") ?? layers[0];
  // detail: the map's area (+ margin) at the finest zoom that stays within the tile budget
  const {tiles, detailZ} = useMemo(() => {
    if (!on || !geo || !layer || !bounds) return {tiles: [], detailZ: 0};
    const b = bounds;
    const box = {min: [b[0] - MARGIN_M, b[1] - MARGIN_M], max: [b[2] + MARGIN_M, b[3] + MARGIN_M]};
    const zmax = layer.zmax ?? 19;
    const z = pickZoom(geo, box, zmax, Math.max(layer.zmin ?? 12, 10));
    // offline: only what the cache can have (its manifest's box), no 404 storm
    return {tiles: servable(tiles_(geo, box, z), layer, z), detailZ: z};
  }, [on, geo, layer, bounds]);
  // context: the ground in view (zoomed out: beyond the map), coarser than the detail
  const camera = useThree(s => s.camera);
  const [view, setView] = useState<{min: number[]; max: number[]} | null>(null);
  const lastView = useRef("");
  const frame = useRef(0);
  const zPlane = (floorZ ?? geo?.anchor.z0 ?? 0) - 0.3;
  useFrame(() => {
    if (!on || !geo || (frame.current++ & 15)) return; // every 16th frame is plenty
    const f = viewFootprint(camera, zPlane);
    // snap to a grid a tenth of the view size, so small moves re-render nothing
    const step = Math.max(25, Math.max(f.max[0] - f.min[0], f.max[1] - f.min[1]) / 10);
    const q = (v: number, up: boolean) => (up ? Math.ceil(v / step) : Math.floor(v / step)) * step;
    const snapped = {min: [q(f.min[0], false), q(f.min[1], false)], max: [q(f.max[0], true), q(f.max[1], true)]};
    const k = snapped.min.concat(snapped.max).join(",");
    if (k !== lastView.current) {
      lastView.current = k;
      setView(snapped);
    }
  });
  const context = useMemo(() => {
    if (!geo || !layer || !view || !detailZ) return [];
    const zmin = layer.live ? 10 : Math.max(layer.zmin ?? 12, 10);
    const z = pickZoom(geo, view, Math.max(zmin, detailZ - 1), zmin, CONTEXT_TILES);
    return servable(tiles_(geo, view, z, CONTEXT_TILES), layer, z);
  }, [geo, layer, view, detailZ]);
  const closeup = useMemo(() => {
    const zmax = layer?.zmax ?? 19;
    if (!geo || !layer || !view || !detailZ || zmax <= detailZ) return [];
    const z = pickZoom(geo, view, zmax, detailZ, CLOSEUP_TILES);
    return z > detailZ ? servable(tiles_(geo, view, z, CLOSEUP_TILES), layer, z) : [];
  }, [geo, layer, view, detailZ]);
  const shown = on && !!layer;
  useEffect(() => {
    imageryWant(shown ? tiles.map(q => q.key) : []);
  }, [shown, tiles]);
  useEffect(() => () => imageryWant([]), []);
  if (!on || !layer || (!tiles.length && !context.length)) return null;
  return (
    <group>
      {context.map(q => (
        <Tile key={`c${q.key}`} q={q} layer={layer.id} z={zPlane - 0.05} opacity={opacity} counted={false} />
      ))}
      {tiles.map(q => (
        <Tile key={q.key} q={q} layer={layer.id} z={zPlane} opacity={opacity} />
      ))}
      {closeup.map(q => (
        <Tile key={`f${q.key}`} q={q} layer={layer.id} z={zPlane + 0.02} opacity={opacity} counted={false} order={-9} />
      ))}
    </group>
  );
}

const tiles_ = (g: Parameters<typeof tilesForBox>[0], box: {min: number[]; max: number[]}, z: number, max?: number) => tilesForBox(g, box, z, max) ?? [];
