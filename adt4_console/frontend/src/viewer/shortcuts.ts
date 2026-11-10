// Viewer keyboard shortcuts (listed in the "?" help, controlsHelp.ts): M toggles
// the full-screen map (mapFull.ts). They never fire while typing in a field.
import {useEffect} from "react";
import {toggleMapFull} from "./mapFull";

export const isTyping = (t: EventTarget | null) => {
  const el = t as HTMLElement | null;
  return !!el && (el.isContentEditable || /^(INPUT|TEXTAREA|SELECT)$/.test(el.tagName));
};

export type Shortcut = "fullmap";

/** The shortcut a key press asks for (null: not one of ours, or typing). */
export function shortcutOf(e: Pick<KeyboardEvent, "key" | "ctrlKey" | "metaKey" | "altKey" | "shiftKey" | "target">): Shortcut | null {
  if (isTyping(e.target)) return null;
  if (e.ctrlKey || e.metaKey || e.altKey) return null;
  if (e.key === "m" || e.key === "M") return "fullmap";
  return null;
}

/** Run a shortcut; false when it does not apply now (the key stays the browser's). */
export function runShortcut(s: Shortcut): boolean {
  switch (s) {
    case "fullmap":
      toggleMapFull();
      return true;
  }
}

export function useMapShortcuts() {
  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      if (e.defaultPrevented) return;
      const s = shortcutOf(e);
      if (s && runShortcut(s)) e.preventDefault();
    };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, []);
}
