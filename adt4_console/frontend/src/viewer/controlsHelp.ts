// Operator controls legend for the map viewer: the single place to update when
// a control changes (Scene.tsx camera bindings, picking, Panes.tsx windows).
export type HelpMode = "3d" | "top";
export interface HelpEntry {
  /** the input, e.g. "Left-drag" or a key */
  keys: string;
  /** what it does */
  does: string;
}
export interface HelpSection {
  title: string;
  entries: HelpEntry[];
}

/** Camera controls per view mode (Scene.tsx CameraControls bindings). */
export const MODE_HELP: Record<HelpMode, HelpSection> = {
  "3d": {
    title: "3D view",
    entries: [
      {keys: "Left-drag", does: "Orbit"},
      {keys: "Right-drag", does: "Pan"},
      {keys: "Wheel / pinch", does: "Zoom"},
      {keys: "One-finger drag", does: "Orbit (touch)"},
      {keys: "Two-finger drag", does: "Pan + zoom (touch)"},
    ],
  },
  top: {
    title: "Top view",
    entries: [
      {keys: "Left-drag", does: "Pan"},
      {keys: "Right-drag", does: "Rotate the map in plane"},
      {keys: "Wheel / pinch", does: "Zoom"},
      {keys: "Two-finger twist", does: "Rotate (touch)"},
      {keys: "Q / E", does: "Rotate 90° left / right"},
      {keys: "R", does: "North up"},
    ],
  },
};

/** Picking and other controls that work in both modes. */
export const COMMON_HELP: HelpSection[] = [
  {
    title: "Picking",
    entries: [
      {keys: "Click node", does: "Select it (Inspect tab)"},
      {keys: "Click bounding box", does: "Select that object"},
      {keys: "Click empty space", does: "Clear the selection"},
      {keys: "Search result", does: "Select + fly to the node"},
      {keys: "Fly to (Inspect)", does: "Fly the camera to the selection"},
    ],
  },
  {
    title: "Keys",
    entries: [
      {keys: "Enter (search)", does: "Pick the first hit"},
      {keys: "P", does: "Save a screenshot of the 3D view"},
      {keys: "M", does: "Full map: the map alone, full screen (M / Esc to leave)"},
      {keys: "Esc", does: "Clear search (in the box) · close this card / dialogs"},
      {keys: "/", does: "Focus search (Maps page)"},
    ],
  },
];

/** Pane layout controls (wide screens only; Panes.tsx). */
export const LAYOUT_HELP: HelpSection = {
  title: "Layout",
  entries: [
    {keys: "Drag pane edge", does: "Resize panel · double-click resets"},
    {keys: "⧉ / drag a tab out", does: "Pop a side-panel tab into a window"},
    {keys: "Drag title bar", does: "Move window (snaps to edges)"},
    {keys: "Drag window edge", does: "Resize window"},
    {keys: "⇥ / drop on tab strip", does: "Dock the window again"},
    {keys: "– / ×", does: "Minimize / close window"},
    {keys: "Reset layout", does: "Defaults (Layout section, left panel)"},
  ],
};

/** One-time hint shown the first time the operator switches to Top. */
export const TOP_HINT = "Right-drag to rotate · Q/E 90° · R north up";
/** the same hint for touch screens (no right button, no keys) */
export const TOP_HINT_TOUCH = "One finger pans · two fingers pinch to zoom, drag to rotate";
export const TOP_HINT_KEY = "adt4.viewer.topHintDismissed";

/** Sections for the help card in the given mode. */
export function helpSections(mode: HelpMode, withLayout: boolean): HelpSection[] {
  return [MODE_HELP[mode], ...COMMON_HELP, ...(withLayout ? [LAYOUT_HELP] : [])];
}

export function hintDismissed(storage: Pick<Storage, "getItem"> | null | undefined): boolean {
  try {
    return storage?.getItem(TOP_HINT_KEY) === "1";
  } catch {
    return false;
  }
}

export function dismissHint(storage: Pick<Storage, "setItem"> | null | undefined) {
  try {
    storage?.setItem(TOP_HINT_KEY, "1");
  } catch {
    /* storage unavailable: the hint just comes back next visit */
  }
}
