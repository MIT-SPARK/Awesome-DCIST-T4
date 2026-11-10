// Tap vs drag on the map: a tap places a corner / picks a node, a drag (or a
// pinch) moves the camera. Fingers wobble more than a mouse, a second finger
// makes the gesture a pinch (never a tap, even when that finger lifts first),
// and a long still press is not a tap either (it reads as "hold to look").

export const TAP_SLOP_PX = {mouse: 5, touch: 12, pen: 8} as const;
export const TAP_MAX_MS = 700;
/** how far (screen px) from a node a tap still picks it */
export const PICK_RADIUS_PX = {mouse: 10, touch: 22, pen: 14} as const;

type Kind = keyof typeof TAP_SLOP_PX;
const kind = (t: string): Kind => (t === "touch" || t === "pen" ? t : "mouse");

export interface TapPointer {
  pointerId: number;
  pointerType: string;
  clientX: number;
  clientY: number;
  button: number;
  timeStamp: number;
}

/** Feed it the canvas' pointerdown / pointerup / pointercancel; `up` says
 *  whether the gesture that just ended was a tap. */
export class TapTracker {
  private down = new Map<number, {x: number; y: number; t: number}>();
  /** set once two pointers were down together; cleared when all are up */
  private multi = false;

  onDown(e: TapPointer) {
    if (e.pointerType === "mouse" && e.button !== 0) return;
    this.down.set(e.pointerId, {x: e.clientX, y: e.clientY, t: e.timeStamp});
    if (this.down.size > 1) this.multi = true;
  }

  onCancel(e: Pick<TapPointer, "pointerId">) {
    this.down.delete(e.pointerId);
    if (!this.down.size) this.multi = false;
  }

  onUp(e: TapPointer): boolean {
    const d = this.down.get(e.pointerId);
    if (!d) return false;
    this.down.delete(e.pointerId);
    const multi = this.multi;
    if (!this.down.size) this.multi = false;
    if (multi) return false;
    if (e.pointerType === "mouse" && e.button !== 0) return false;
    const k = kind(e.pointerType);
    if (Math.hypot(e.clientX - d.x, e.clientY - d.y) > TAP_SLOP_PX[k]) return false;
    return k === "mouse" || e.timeStamp - d.t <= TAP_MAX_MS;
  }
}

export const pickRadius = (pointerType: string) => PICK_RADIUS_PX[kind(pointerType)];
