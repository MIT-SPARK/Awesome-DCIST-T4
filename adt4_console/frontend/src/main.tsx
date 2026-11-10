import {StrictMode} from "react";
import {createRoot} from "react-dom/client";
import {App} from "./App";
import {initToken} from "./lib/auth";
import {ws} from "./ws";
import {bindHealth, useHealthStore} from "./stores/health";
import {bindJobs, useJobs} from "./stores/jobs";
import {bindTheme} from "./stores/theme";
import "./styles.css";
import "./theme.css";

bindTheme();
initToken();
bindJobs();
bindHealth();
ws.start();
void useJobs.getState().load();
void useHealthStore.getState().load();
// health is cheap: keep the top-bar indicators fresh even without WS traffic
window.setInterval(() => void useHealthStore.getState().load(), 15_000);

createRoot(document.getElementById("root")!).render(
  <StrictMode>
    <App />
  </StrictMode>,
);
