// "Full map": the 3D map alone on the screen — the app's nav rail and top bar are
// hidden (body.map-full, styles.css), the side panes close (ViewerPage restores
// them on exit) and the browser goes full screen where it allows. Leaving browser
// full screen (Esc) leaves full map too.
import {create} from "zustand";

/** Browser full screen for the whole page (needs a user gesture; ignored where not allowed). */
export function requestPageFullscreen() {
  const el = document.documentElement as HTMLElement & {webkitRequestFullscreen?: () => void};
  try {
    if (document.fullscreenElement) return;
    if (el.requestFullscreen) void el.requestFullscreen({navigationUI: "hide"}).catch(() => undefined);
    else el.webkitRequestFullscreen?.();
  } catch {
    /* not allowed: the CSS still hides the page chrome */
  }
}

export const useMapFull = create<{on: boolean}>(() => ({on: false}));

function apply(on: boolean) {
  document.body.classList.toggle("map-full", on);
  useMapFull.setState({on});
}

export function enterMapFull() {
  if (useMapFull.getState().on) return;
  apply(true);
  requestPageFullscreen(); // from the click / key handler: browsers need a user gesture
}

export function exitMapFull() {
  if (!useMapFull.getState().on) return;
  apply(false);
  try {
    if (document.fullscreenElement) void document.exitFullscreen().catch(() => undefined);
  } catch {
    /* already out */
  }
}

export const toggleMapFull = () => (useMapFull.getState().on ? exitMapFull() : enterMapFull());

let wasFullscreen = false;
if (typeof document !== "undefined") {
  document.addEventListener("fullscreenchange", () => {
    const fs = !!document.fullscreenElement;
    // Esc (or the browser's own exit) ends browser full screen: end full map with it
    if (wasFullscreen && !fs) exitMapFull();
    wasFullscreen = fs;
  });
}
