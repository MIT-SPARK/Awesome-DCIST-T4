import {useEffect, useMemo, useRef, type CSSProperties} from "react";
import * as THREE from "three";
import {Canvas, useThree} from "@react-three/fiber";
import {CameraControls, Html} from "@react-three/drei";
import CameraControlsImpl from "camera-controls";
import {useViewer, type SourceView} from "../stores/viewer";
import {layoutGraph, displayPos, type GraphLayout} from "./layout";
import {GraphLayers, pickRegistry} from "./GraphLayers";
import {pickBox, type BoxSet} from "../lib/bboxPick";
import {MeshTiles, type TileStats} from "./MeshTiles";
import {DETAIL_LEVEL} from "../lib/meshPlan";
import {useViewStyle} from "../stores/viewStyle";
import {contrastFor, defaultBackground, STYLE_BACKGROUND, type ColorOpts} from "./style";
import {useTheme} from "../stores/theme";
import {useBackground} from "./background";
import {captureBridge} from "./screenshot";
import {SatelliteGround} from "./SatelliteGround";
import {pickRadius, TapTracker} from "../lib/tap";

THREE.Object3D.DEFAULT_UP.set(0, 0, 1);

export interface SceneCallbacks {
  onNode: (src: string, sym: string) => void;
  onPoint: (p: [number, number, number]) => void;
  onMiss: () => void;
  onStats: (src: string, s: TileStats) => void;
  onLayout: (src: string, l: GraphLayout) => void;
}

export {useBackground};

export function useColorOpts(): ColorOpts {
  const rules = useViewStyle(s => s.colors);
  const labelColors = useViewStyle(s => s.labelColors);
  const placesByRegion = useViewStyle(s => s.regionColorPlaces);
  return useMemo(() => ({rules, labelColors, placesByRegion}), [rules, labelColors, placesByRegion]);
}

function SourceContent({src, cb}: {src: SourceView; cb: SceneCallbacks}) {
  const {hiddenLayers, edgeKinds, flat, stackHeight, showBBoxes, showTrajectory, meshOpacity, vizStyle, showText, meshCull, meshClipOn, meshClipHeight} = useViewer();
  const heights = useViewStyle(s => s.heights);
  const regionHulls = useViewStyle(s => s.regionHulls);
  const meshColor = useViewStyle(s => s.meshColor);
  const meshLevel = useViewer(s => DETAIL_LEVEL[s.meshDetail === "auto" ? s.meshAutoDetail : s.meshDetail]);
  const colors = useColorOpts();
  const bg = useBackground();
  const edgeInk = contrastFor(bg, vizStyle).edge;
  const layout = useMemo(
    () => (src.model ? layoutGraph(src.model, {hiddenLayers, edgeKinds, flat, stackHeight, style: vizStyle, text: showText, heights, colors, regionHulls}) : null),
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [src.model, src.rev, hiddenLayers, edgeKinds, flat, stackHeight, vizStyle, showText, heights, colors, regionHulls],
  );
  const onLayout = cb.onLayout;
  useEffect(() => {
    if (layout) onLayout(src.id, layout);
  }, [layout, src.id, onLayout]);
  return (
    <group>
      {src.showMesh && src.manifest && (
        <MeshTiles
          src={src.id}
          manifest={src.manifest}
          manifestRev={src.manifestRev}
          detail={meshLevel}
          opacity={meshOpacity}
          cullBackfaces={meshCull}
          clipHeight={meshClipOn ? meshClipHeight : null}
          colorMode={meshColor.mode}
          uniformColor={meshColor.color}
          onStats={s => cb.onStats(src.id, s)}
        />
      )}
      {src.showGraph && layout && (
        <GraphLayers
          src={src.id}
          layout={layout}
          showBBoxes={showBBoxes}
          showTrajectory={showTrajectory}
          trajectory={src.trajectory}
          trajectoryRobots={src.trajectoryRobots}
          edgeKinds={edgeKinds}
          style={vizStyle}
          showText={showText}
          colors={colors}
          edgeInk={edgeInk}
        />
      )}
    </group>
  );
}

/** Click picking: nearest projected node within a few pixels, else the mesh surface. */
function Picker({cb}: {cb: SceneCallbacks}) {
  const {gl, camera, scene} = useThree();
  const cbRef = useRef(cb);
  cbRef.current = cb;
  useEffect(() => {
    const el = gl.domElement;
    // tap vs drag / pinch (lib/tap.ts): only a tap picks or places a corner
    const taps = new TapTracker();
    const pd = (e: PointerEvent) => taps.onDown(e);
    const pc = (e: PointerEvent) => taps.onCancel(e);
    const pu = (e: PointerEvent) => {
      if (!taps.onUp(e)) return;
      const radius = pickRadius(e.pointerType);
      const rect = el.getBoundingClientRect();
      const x = e.clientX - rect.left;
      const y = e.clientY - rect.top;
      const enabled = useViewer.getState().sources;
      const v = new THREE.Vector3();
      let best: {src: string; sym: string; d: number; depth: number} | null = null;
      for (const [src, layers] of pickRegistry) {
        if (!enabled[src]?.enabled || !enabled[src]?.showGraph) continue;
        for (const l of layers.values()) {
          const p = l.positions;
          for (let i = 0; i < l.syms.length; i++) {
            v.set(p[i * 3], p[i * 3 + 1], p[i * 3 + 2]);
            v.project(camera);
            if (v.z > 1 || v.z < -1) continue;
            const sx = ((v.x + 1) / 2) * rect.width;
            const sy = ((1 - v.y) / 2) * rect.height;
            const d = Math.hypot(sx - x, sy - y);
            if (d < radius && (!best || d < best.d - 2 || (Math.abs(d - best.d) <= 2 && v.z < best.depth))) {
              best = {src, sym: l.syms[i], d, depth: v.z};
            }
          }
        }
      }
      if (best) {
        cbRef.current.onNode(best.src, best.sym);
        return;
      }
      const ray = new THREE.Raycaster();
      ray.setFromCamera(new THREE.Vector2((x / rect.width) * 2 - 1, -(y / rect.height) * 2 + 1), camera);
      // object bounding boxes (when shown): nodes inside a box were tried first;
      // overlapping boxes resolve to the smallest one
      if (useViewer.getState().showBBoxes) {
        const o = ray.ray.origin;
        const d = ray.ray.direction;
        let hit: ReturnType<typeof pickBox<string>> = null;
        for (const [src, layers] of pickRegistry) {
          if (!enabled[src]?.enabled || !enabled[src]?.showGraph) continue;
          const sets: [string, BoxSet][] = [];
          for (const l of layers.values()) if (l.boxes) sets.push([src, {syms: l.syms, boxes: l.boxes}]);
          if (!sets.length) continue;
          const h = pickBox([o.x, o.y, o.z], [d.x, d.y, d.z], sets);
          if (h && (!hit || h.volume < hit.volume * 0.999 || (h.volume <= hit.volume * 1.001 && h.t < hit.t))) hit = h;
        }
        if (hit) {
          cbRef.current.onNode(hit.key, hit.sym);
          return;
        }
      }
      const groups: THREE.Object3D[] = [];
      scene.traverse(o => {
        if (o.userData.meshGroup && o.visible) groups.push(o);
      });
      const hit = ray.intersectObjects(groups, true)[0];
      if (hit) cbRef.current.onPoint([hit.point.x, hit.point.y, hit.point.z]);
      else {
        // no mesh: intersect the ground plane (graph-only sources)
        const plane = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0);
        const p = new THREE.Vector3();
        if (ray.ray.intersectPlane(plane, p)) cbRef.current.onPoint([p.x, p.y, p.z]);
        else cbRef.current.onMiss();
      }
    };
    el.addEventListener("pointerdown", pd);
    el.addEventListener("pointerup", pu);
    el.addEventListener("pointercancel", pc);
    return () => {
      el.removeEventListener("pointerdown", pd);
      el.removeEventListener("pointerup", pu);
      el.removeEventListener("pointercancel", pc);
    };
  }, [gl, camera, scene]);
  return null;
}

/** Marker colours that stand out on the current background: white / yellow on
 *  dark (the historical look), near-black / amber on a light background. */
function useMarkerInk(): {hi: string; search: string} {
  const light = contrastFor(useBackground()).light;
  return light ? {hi: "#111111", search: "#e08a00"} : {hi: "#ffffff", search: "#ffe14d"};
}

function Markers() {
  const {selected, highlight, sources, flat, stackHeight, vizStyle} = useViewer();
  const ink = useMarkerInk();
  const heights = useViewStyle(s => s.heights);
  const findPos = (src: string, sym: string): [number, number, number] | null => {
    const n = sources[src]?.model?.nodes.get(sym);
    return n ? displayPos(n, flat, stackHeight, vizStyle, heights) : null;
  };
  const sel = selected ? findPos(selected.src, selected.sym) : null;
  const selNode = selected ? sources[selected.src]?.model?.nodes.get(selected.sym) : null;
  return (
    <>
      {highlight.slice(0, 60).map(h => {
        const p = findPos(h.src, h.sym);
        if (!p) return null;
        return (
          <mesh key={h.src + h.sym} position={p}>
            <torusGeometry args={[0.55, 0.06, 6, 28]} />
            <meshBasicMaterial color={ink.search} depthTest={false} transparent />
          </mesh>
        );
      })}
      {sel && selNode && (
        <group position={sel}>
          <mesh>
            <torusGeometry args={[0.8, 0.09, 8, 36]} />
            <meshBasicMaterial color={ink.hi} depthTest={false} transparent />
          </mesh>
          <Html position={[0, 0, 1.2]} center zIndexRange={[30, 0]}>
            <div className="node-tag">{sources[selected!.src]?.model?.displayName(selNode)}</div>
          </Html>
        </group>
      )}
    </>
  );
}

function CameraRig({bounds}: {bounds: {min: number[]; max: number[]} | null}) {
  const ref = useRef<CameraControlsImpl>(null);
  const {topDown, flyTo} = useViewer();
  const fitted = useRef(false);
  // initial framing once something with bounds is loaded
  useEffect(() => {
    const c = ref.current;
    if (!c || !bounds || fitted.current) return;
    fitted.current = true;
    const cx = (bounds.min[0] + bounds.max[0]) / 2;
    const cy = (bounds.min[1] + bounds.max[1]) / 2;
    const span = Math.max(bounds.max[0] - bounds.min[0], bounds.max[1] - bounds.min[1], 10);
    const cz = bounds.min[2];
    // already in the top view: frame from straight above
    if (useViewer.getState().topDown) void c.setLookAt(cx, cy, cz + span * 1.1, cx, cy, cz, false);
    else void c.setLookAt(cx - span * 0.2, cy - span * 1.0, cz + span * 0.9, cx, cy, cz, false);
  }, [bounds]);
  useEffect(() => {
    const c = ref.current;
    if (!c) return;
    if (topDown) {
      c.minPolarAngle = 0;
      c.maxPolarAngle = 0;
      c.mouseButtons.left = CameraControlsImpl.ACTION.TRUCK;
      // with the polar angle pinned straight down, orbiting only spins the map in plane
      c.mouseButtons.right = CameraControlsImpl.ACTION.ROTATE;
      c.touches.one = CameraControlsImpl.ACTION.TOUCH_TRUCK;
      c.touches.two = CameraControlsImpl.ACTION.TOUCH_DOLLY_ROTATE;
      void c.rotatePolarTo(0, true);
      void c.rotateAzimuthTo(0, true);
    } else {
      c.minPolarAngle = 0;
      c.maxPolarAngle = Math.PI * 0.49;
      c.mouseButtons.left = CameraControlsImpl.ACTION.ROTATE;
      c.mouseButtons.right = CameraControlsImpl.ACTION.TRUCK;
      c.touches.one = CameraControlsImpl.ACTION.TOUCH_ROTATE;
      c.touches.two = CameraControlsImpl.ACTION.TOUCH_DOLLY_TRUCK;
      void c.rotatePolarTo(Math.PI / 4, true);
    }
  }, [topDown]);
  // top view: Q / E rotate the map 90° left / right, R puts north back up
  useEffect(() => {
    if (!topDown) return;
    const onKey = (e: KeyboardEvent) => {
      const c = ref.current;
      const t = e.target as HTMLElement | null;
      if (!c || e.ctrlKey || e.metaKey || e.altKey) return;
      if (t && (t.isContentEditable || /^(INPUT|TEXTAREA|SELECT)$/.test(t.tagName))) return;
      const k = e.key.toLowerCase();
      if (k === "q") void c.rotate(Math.PI / 2, 0, true);
      else if (k === "e") void c.rotate(-Math.PI / 2, 0, true);
      else if (k === "r") void c.rotateAzimuthTo(0, true);
    };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, [topDown]);
  // ignore a fly-to request that predates this mount (store outlives the page)
  const flyNonce = useRef(flyTo?.nonce ?? 0);
  useEffect(() => {
    const c = ref.current;
    if (!c || !flyTo || flyTo.nonce === flyNonce.current) return;
    flyNonce.current = flyTo.nonce;
    const [x, y, z] = flyTo.target;
    const dist = Math.min(Math.max(c.distance, 6), 14);
    if (useViewer.getState().topDown) void c.setLookAt(x, y, z + dist * 1.5, x, y, z, true);
    else void c.setLookAt(x - dist * 0.3, y - dist * 0.7, z + dist * 0.65, x, y, z, true);
  }, [flyTo]);
  return <CameraControls ref={ref} makeDefault maxPolarAngle={Math.PI * 0.49} dollyToCursor minDistance={1} maxDistance={2000} />;
}

/** RViz look for the hydra style: grey background, light grid. */
export const BACKGROUND = STYLE_BACKGROUND;

function Background() {
  const bg = useBackground();
  const {gl, invalidate} = useThree();
  useEffect(() => {
    gl.setClearColor(bg, 1);
    invalidate();
  }, [gl, bg, invalidate]);
  return null;
}

/** Hands the renderer to the screenshot code (outside the R3F tree). */
function CaptureBridge() {
  const {gl, scene, camera, invalidate, controls} = useThree();
  useEffect(() => {
    captureBridge.current = {gl, scene, camera, invalidate, controls: controls as unknown as CameraControlsImpl | null};
    return () => {
      if (captureBridge.current?.gl === gl) captureBridge.current = null;
    };
  }, [gl, scene, camera, invalidate, controls]);
  return null;
}

function Ground({bounds}: {bounds: {min: number[]; max: number[]} | null}) {
  const style = useViewer(s => s.vizStyle);
  const show = useViewStyle(s => s.grid);
  const bg = useBackground();
  const ct = contrastFor(bg, style);
  const size = bounds ? Math.ceil(Math.max(bounds.max[0] - bounds.min[0], bounds.max[1] - bounds.min[1], 20) / 10) * 10 + 40 : 100;
  const cx = bounds ? Math.round((bounds.min[0] + bounds.max[0]) / 2) : 0;
  const cy = bounds ? Math.round((bounds.min[1] + bounds.max[1]) / 2) : 0;
  const grid = useMemo(() => {
    const g = new THREE.GridHelper(size, size / 2, ct.grid[0], ct.grid[1]);
    g.rotation.x = Math.PI / 2;
    (g.material as THREE.Material).transparent = true;
    (g.material as THREE.Material).opacity = ct.gridOpacity;
    return g;
  }, [size, ct.grid[0], ct.grid[1], ct.gridOpacity]);
  return show ? <primitive object={grid} position={[cx, cy, -0.05]} /> : null;
}

// Phones / tablets: tile GPUs with little memory. Cap the pixel ratio, skip
// MSAA and let the browser discard the drawing buffer (keeping it costs a copy
// per frame there). Desktop keeps it so headless screenshot tours see pixels.
const COARSE = typeof window !== "undefined" && window.matchMedia?.("(pointer: coarse)").matches === true;
const DPR: [number, number] = COARSE ? [1, 1.5] : [1, 2];
const GL = {antialias: !COARSE, powerPreference: "high-performance" as const, preserveDrawingBuffer: !COARSE};

export function Scene({sources, bounds, cb}: {sources: SourceView[]; bounds: {min: number[]; max: number[]} | null; cb: SceneCallbacks}) {
  const bg = useBackground();
  const style = useViewer(s => s.vizStyle);
  const ct = contrastFor(bg, style);
  return (
    <Canvas
      className="scene-canvas"
      style={{"--viz-text": ct.text, "--viz-halo": ct.halo} as CSSProperties}
      frameloop="demand"
      dpr={DPR}
      gl={GL}
      camera={{fov: 50, near: 0.1, far: 5000, position: [-10, -30, 30], up: [0, 0, 1]}}
      onCreated={({gl}) => {
        gl.setClearColor(useViewStyle.getState().background ?? defaultBackground(useViewer.getState().vizStyle, useTheme.getState().theme));
        gl.localClippingEnabled = true; // mesh "max height" clip
      }}
      data-testid="viewer-canvas"
    >
      <ambientLight intensity={0.85} />
      <directionalLight position={[30, -20, 60]} intensity={1.0} />
      <directionalLight position={[-40, 30, 20]} intensity={0.35} />
      <Background />
      <CaptureBridge />
      <Ground bounds={bounds} />
      <SatelliteGround floorZ={bounds ? bounds.min[2] : null} />
      {sources.map(s => (
        <SourceContent key={s.id} src={s} cb={cb} />
      ))}
      <Markers />
      <Picker cb={cb} />
      <CameraRig bounds={bounds} />
    </Canvas>
  );
}
