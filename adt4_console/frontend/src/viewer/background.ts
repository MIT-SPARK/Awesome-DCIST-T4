// The 3D view's effective background and colours derived from it (kept out of
// Scene.tsx so overlays can use them without an import cycle).
import {useViewer} from "../stores/viewer";
import {useViewStyle} from "../stores/viewStyle";
import {useTheme} from "../stores/theme";
import {defaultBackground} from "./style";

/** Effective background colour (user choice, else the style default for the console theme). */
export function useBackground(): string {
  const style = useViewer(s => s.vizStyle);
  const bg = useViewStyle(s => s.background);
  const theme = useTheme(s => s.theme);
  return bg ?? defaultBackground(style, theme);
}
