// Console colour theme (dark / light), remembered per browser. The resolved
// theme is mirrored onto <html data-theme="..."> so CSS variables in
// styles.css switch app-wide; the 3D viewer reads `useTheme` for its default
// background (an explicit background picked in the Display panel still wins).
import {create} from "zustand";

export type Theme = "dark" | "light";

export const THEME_STORAGE_KEY = "adt4.theme.v1";
/** Dark is the operator default; light is opt-in. */
export const DEFAULT_THEME: Theme = "dark";

export function parseStoredTheme(raw: string | null): Theme {
  return raw === "light" || raw === "dark" ? raw : DEFAULT_THEME;
}

export function loadTheme(): Theme {
  try {
    return parseStoredTheme(window.localStorage.getItem(THEME_STORAGE_KEY));
  } catch {
    return DEFAULT_THEME;
  }
}

export function saveTheme(t: Theme): void {
  try {
    window.localStorage.setItem(THEME_STORAGE_KEY, t);
  } catch {
    /* private window / blocked storage: the choice lasts for this page only */
  }
}

/** Put the theme on the document root (data-theme + color-scheme for native widgets). */
export function applyTheme(t: Theme, root: HTMLElement | null = typeof document !== "undefined" ? document.documentElement : null): void {
  if (!root) return;
  root.dataset.theme = t;
  root.style.colorScheme = t;
}

interface ThemeStore {
  theme: Theme;
  setTheme: (t: Theme) => void;
  toggle: () => void;
}

export const useTheme = create<ThemeStore>((set, get) => ({
  theme: typeof window === "undefined" ? DEFAULT_THEME : loadTheme(),
  setTheme: t => {
    if (t === get().theme) return;
    set({theme: t});
    saveTheme(t);
    applyTheme(t);
  },
  toggle: () => get().setTheme(get().theme === "dark" ? "light" : "dark"),
}));

/** Call once at startup (before first render) so the first paint is in the right theme. */
export function bindTheme(): void {
  applyTheme(useTheme.getState().theme);
}
