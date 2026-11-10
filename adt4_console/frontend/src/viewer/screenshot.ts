// Save the current 3D view as PNG / JPEG: render on demand into the WebGL
// canvas (optionally at 2x), copy it into a 2D canvas the same frame (works
// without preserveDrawingBuffer), composite the HTML text labels drawn over the
// canvas, add a legend (sources, time, scale bar) and download.
import * as THREE from "three";
import type CameraControlsImpl from "camera-controls";
import {contrastFor, effectiveShot, niceLength, shotFilename, type ShotOpts} from "./style";

export interface CaptureTarget {
  gl: THREE.WebGLRenderer;
  scene: THREE.Scene;
  camera: THREE.Camera;
  invalidate: () => void;
  controls: CameraControlsImpl | null;
}

export const captureBridge: {current: CaptureTarget | null} = {current: null};

export interface ShotContext {
  /** enabled source ids (filename) */
  sourceIds: string[];
  /** human labels (legend) */
  sourceLabels: string[];
  background: string;
  style: "hydra" | "classic";
  now?: Date;
}

/** Overlay label elements drawn on top of the canvas (drei <Html> portals). */
const LABEL_SELECTOR = ".viz-text, .node-tag";

function roundRect(ctx: CanvasRenderingContext2D, x: number, y: number, w: number, h: number, r: number) {
  ctx.beginPath();
  ctx.moveTo(x + r, y);
  ctx.arcTo(x + w, y, x + w, y + h, r);
  ctx.arcTo(x + w, y + h, x, y + h, r);
  ctx.arcTo(x, y + h, x, y, r);
  ctx.arcTo(x, y, x + w, y, r);
  ctx.closePath();
}

const transparentCss = (c: string) => !c || c === "transparent" || /rgba\([^)]*,\s*0\)$/.test(c);

/** Draw the DOM labels sitting over `canvas` into ctx (scale k = output px per CSS px). */
export function compositeLabels(ctx: CanvasRenderingContext2D, canvas: HTMLCanvasElement, k: number): number {
  const host = canvas.parentElement?.parentElement ?? canvas.parentElement;
  if (!host) return 0;
  const base = canvas.getBoundingClientRect();
  let n = 0;
  host.querySelectorAll<HTMLElement>(LABEL_SELECTOR).forEach(el => {
    const r = el.getBoundingClientRect();
    if (r.width === 0 || r.right < base.left || r.left > base.right || r.bottom < base.top || r.top > base.bottom) return;
    const cs = getComputedStyle(el);
    if (cs.visibility === "hidden" || cs.display === "none" || Number(cs.opacity) === 0) return;
    // wrappers hidden by drei occlusion / behind-camera get display:none on an ancestor
    if (el.offsetParent === null && cs.position !== "fixed") return;
    const x = (r.left - base.left) * k;
    const y = (r.top - base.top) * k;
    const w = r.width * k;
    const h = r.height * k;
    ctx.save();
    ctx.globalAlpha = Number(cs.opacity) || 1;
    if (!transparentCss(cs.backgroundColor)) {
      ctx.fillStyle = cs.backgroundColor;
      roundRect(ctx, x, y, w, h, (parseFloat(cs.borderTopLeftRadius) || 0) * k);
      ctx.fill();
    }
    const bw = parseFloat(cs.borderTopWidth) || 0;
    if (bw > 0 && !transparentCss(cs.borderTopColor)) {
      ctx.strokeStyle = cs.borderTopColor;
      ctx.lineWidth = bw * k;
      roundRect(ctx, x + (bw * k) / 2, y + (bw * k) / 2, w - bw * k, h - bw * k, (parseFloat(cs.borderTopLeftRadius) || 0) * k);
      ctx.stroke();
    }
    const fontSize = parseFloat(cs.fontSize) || 11;
    ctx.font = `${cs.fontStyle} ${cs.fontWeight} ${fontSize * k}px ${cs.fontFamily}`;
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    const text = el.textContent ?? "";
    const cx = x + w / 2;
    const cy = y + h / 2;
    const shadow = /(rgba?\([^)]*\)|#[0-9a-f]{3,8})/i.exec(cs.textShadow)?.[1];
    if (shadow && cs.textShadow !== "none") {
      ctx.strokeStyle = shadow;
      ctx.lineWidth = 2.5 * k;
      ctx.lineJoin = "round";
      ctx.strokeText(text, cx, cy);
    }
    ctx.fillStyle = cs.color;
    ctx.fillText(text, cx, cy);
    ctx.restore();
    n++;
  });
  return n;
}

/** Metres per output pixel at the orbit target (perspective camera). */
function metresPerPixel(t: CaptureTarget, heightPx: number): number | null {
  const cam = t.camera as THREE.PerspectiveCamera;
  if (!cam.isPerspectiveCamera) return null;
  const target = new THREE.Vector3();
  if (t.controls?.getTarget) t.controls.getTarget(target);
  const d = cam.position.distanceTo(target);
  if (!(d > 0)) return null;
  return (2 * d * Math.tan(THREE.MathUtils.degToRad(cam.fov) / 2)) / heightPx;
}

function drawLegend(ctx: CanvasRenderingContext2D, w: number, h: number, k: number, info: ShotContext, mpp: number | null, onTransparent: boolean) {
  const ct = contrastFor(info.background, info.style);
  const ink = onTransparent ? "#000000" : ct.ink;
  const plate = onTransparent ? "rgba(255,255,255,0.75)" : ct.light ? "rgba(255,255,255,0.7)" : "rgba(0,0,0,0.45)";
  const now = info.now ?? new Date();
  const line1 = `ADT4 · ${info.sourceLabels.length ? info.sourceLabels.join(" + ") : "no source"}`;
  const line2 = now.toLocaleString(undefined, {year: "numeric", month: "2-digit", day: "2-digit", hour: "2-digit", minute: "2-digit", second: "2-digit"});
  const pad = 10 * k;
  const fs = 12 * k;
  ctx.save();
  ctx.font = `600 ${fs}px system-ui, sans-serif`;
  const tw = Math.max(ctx.measureText(line1).width, ctx.measureText(line2).width);
  let barPx = 0;
  let barM = 0;
  if (mpp) {
    barM = niceLength(mpp * 140 * k);
    barPx = barM / mpp;
  }
  const boxW = Math.max(tw, barPx) + pad * 2;
  const boxH = fs * 2.6 + (barPx ? fs * 2 : 0) + pad;
  const x0 = pad;
  const y0 = h - boxH - pad;
  ctx.fillStyle = plate;
  roundRect(ctx, x0, y0, boxW, boxH, 6 * k);
  ctx.fill();
  ctx.fillStyle = ink;
  ctx.textBaseline = "top";
  ctx.fillText(line1, x0 + pad, y0 + pad * 0.8);
  ctx.font = `400 ${fs * 0.95}px system-ui, sans-serif`;
  ctx.fillText(line2, x0 + pad, y0 + pad * 0.8 + fs * 1.3);
  if (barPx) {
    const by = y0 + pad * 0.8 + fs * 3.1;
    ctx.fillRect(x0 + pad, by, barPx, 3 * k);
    ctx.fillRect(x0 + pad, by - 4 * k, 1.5 * k, 11 * k);
    ctx.fillRect(x0 + pad + barPx - 1.5 * k, by - 4 * k, 1.5 * k, 11 * k);
    ctx.font = `400 ${fs * 0.9}px system-ui, sans-serif`;
    ctx.fillText(barM >= 1 ? `${barM} m` : `${Math.round(barM * 100)} cm`, x0 + pad, by + 6 * k);
  }
  void w;
  ctx.restore();
}

/** Render + composite the current view; resolves to the encoded image. */
export async function captureView(opts: ShotOpts, info: ShotContext): Promise<{blob: Blob; filename: string; width: number; height: number}> {
  const t = captureBridge.current;
  if (!t) throw new Error("3D view not ready");
  const o = effectiveShot(opts);
  const {gl, scene, camera} = t;
  const canvas = gl.domElement;
  const basePR = gl.getPixelRatio();
  const prevClear = new THREE.Color();
  gl.getClearColor(prevClear);
  const prevAlpha = gl.getClearAlpha();
  const out = document.createElement("canvas");
  try {
    if (o.scale !== 1) gl.setPixelRatio(basePR * o.scale);
    if (o.transparent) gl.setClearColor(prevClear, 0);
    gl.render(scene, camera);
    out.width = canvas.width;
    out.height = canvas.height;
    const ctx = out.getContext("2d");
    if (!ctx) throw new Error("no 2D canvas");
    if (!o.transparent) {
      ctx.fillStyle = info.background;
      ctx.fillRect(0, 0, out.width, out.height);
    }
    ctx.drawImage(canvas, 0, 0);
    const k = out.width / Math.max(1, canvas.getBoundingClientRect().width);
    if (o.labels) compositeLabels(ctx, canvas, k);
    if (o.legend) drawLegend(ctx, out.width, out.height, k, info, metresPerPixel(t, out.height), o.transparent);
  } finally {
    if (o.scale !== 1) gl.setPixelRatio(basePR);
    gl.setClearColor(prevClear, prevAlpha);
    t.invalidate();
  }
  const blob = await new Promise<Blob | null>(res => out.toBlob(res, o.mime, o.format === "jpeg" ? o.quality : undefined));
  if (!blob) throw new Error("image encoding failed");
  return {blob, filename: shotFilename(info.sourceIds, info.now ?? new Date(), o.ext), width: out.width, height: out.height};
}

export function downloadBlob(blob: Blob, filename: string) {
  const url = URL.createObjectURL(blob);
  const a = document.createElement("a");
  a.href = url;
  a.download = filename;
  document.body.appendChild(a);
  a.click();
  a.remove();
  window.setTimeout(() => URL.revokeObjectURL(url), 5000);
}
