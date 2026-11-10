// The live map (API.md §6.7): picking a run loads it into the live Neo4j in stages;
// the viewer switches as soon as the scene graph is in and shows the rest loading.
import {afterEach, describe, expect, it, vi} from "vitest";
import {act} from "react";
import {createRoot, type Root} from "react-dom/client";
import {_resetLiveMap, applyLiveMap, mapLoaded, mapProgress, useLiveMap, type LiveMap, type MapStage} from "../src/stores/liveMap";
import {useViewer} from "../src/stores/viewer";

(globalThis as unknown as {IS_REACT_ACT_ENVIRONMENT: boolean}).IS_REACT_ACT_ENVIRONMENT = true;

let root: Root | null = null;
let host: HTMLDivElement | null = null;
function render(el: React.ReactElement) {
  host = document.createElement("div");
  document.body.appendChild(host);
  root = createRoot(host);
  act(() => root!.render(el));
  return host;
}
afterEach(() => {
  act(() => root?.unmount());
  host?.remove();
  root = null;
  host = null;
  vi.restoreAllMocks();
  useLiveMap.setState({map: null, choices: [], error: null});
  _resetLiveMap();
});

const json = (body: unknown) => new Response(JSON.stringify(body), {status: 200, headers: {"Content-Type": "application/json"}});
const stage = (id: MapStage["id"], state: MapStage["state"], pct = 0): MapStage => ({id, label: id, state, pct});
const live = (name: string, ready: boolean, stages: MapStage[], has_images = true): LiveMap => ({
  target: "live", run: `/runs/${name}`, name, dsg: `/runs/${name}/hydra/backend/dsg.json`, nodes: 10,
  has_images, has_features: false, ready, job_id: ready && stages.every(s => s.state !== "running") ? null : "j1", stages,
}); // prettier-ignore

describe("live map progress", () => {
  it("counts only the stages that run", () => {
    const m = live("a", true, [stage("graph", "done", 100), stage("images", "running", 50), stage("features", "skipped", 100),
      stage("keyframes", "pending")]); // prettier-ignore
    expect(mapProgress(m)).toBe(50);
    expect(mapLoaded(m)).toBe(false);
    expect(mapLoaded(live("a", true, [stage("graph", "done", 100), stage("images", "skipped", 100)], false))).toBe(true);
  });

  it("switches the viewer to the live source once the graph is in", async () => {
    const loadCatalog = vi.fn(async () => {});
    const enableSource = vi.fn();
    useViewer.setState({
      loadCatalog, enableSource, reloadSource: vi.fn(async () => {}),
      order: ["file:old", "neo4j:live"],
      sources: {"file:old": {kind: "file", enabled: true}, "neo4j:live": {kind: "neo4j", enabled: false}} as never,
    } as never); // prettier-ignore
    applyLiveMap(live("old", true, [stage("graph", "done", 100)]));
    expect(loadCatalog).not.toHaveBeenCalled(); // the map the page opened on
    applyLiveMap(live("new", false, [stage("graph", "running", 40), stage("images", "pending")]));
    expect(loadCatalog).not.toHaveBeenCalled();
    await act(async () => {
      applyLiveMap(live("new", true, [stage("graph", "done", 100), stage("images", "running", 5)]));
      await new Promise(r => setTimeout(r, 0));
    });
    expect(loadCatalog).toHaveBeenCalledTimes(1);
    expect(enableSource).toHaveBeenCalledWith("file:old", false);
    expect(enableSource).toHaveBeenCalledWith("neo4j:live", true);
  });
});

describe("LiveMapPanel", () => {
  it("lists this machine's runs, loads one and shows its stages", async () => {
    const {LiveMapPanel, MapLoadPill} = await import("../src/viewer/LiveMapPanel");
    const posted: unknown[] = [];
    vi.spyOn(globalThis, "fetch").mockImplementation(async (url: RequestInfo | URL, init?: RequestInit) => {
      const u = String(url);
      if (u.endsWith("api/maps/active")) return json(live("killian", true, [stage("graph", "done", 100), stage("images", "skipped", 100)], false));
      if (u.endsWith("api/maps?local=true"))
        return json({base: "here", machines: {here: [
          {name: "killian", path: "~/adt4_output/killian", abs_path: "/runs/killian", has_dsg: true, has_image_store: false, size: 1},
          {name: "penn", path: "~/adt4_output/penn", abs_path: "/runs/penn", has_dsg: true, has_image_store: true, size: 1},
          {name: "bag_only", path: "~/adt4_output/bag_only", has_dsg: false, size: 1},
        ]}}); // prettier-ignore
      if (u.endsWith("api/maps/activate")) {
        posted.push(JSON.parse(String(init?.body)));
        return json({...live("penn", false, [stage("graph", "running", 10), stage("images", "pending"), stage("features", "skipped", 100)]), job_id: "j9"});
      }
      return json({});
    });
    const el = render(
      <>
        <LiveMapPanel />
        <MapLoadPill />
      </>,
    );
    await act(async () => {
      await new Promise(r => setTimeout(r, 0));
    });
    const select = el.querySelector<HTMLSelectElement>('select[aria-label="Live map"]')!;
    expect([...select.options].map(o => o.textContent)).toEqual(["killian (no images)", "penn"]);
    expect(el.textContent).toContain("Graph loaded");
    expect(el.querySelector('[data-testid="map-load-pill"]')).toBeNull();
    await act(async () => {
      select.value = "/runs/penn";
      select.dispatchEvent(new Event("change", {bubbles: true}));
      await new Promise(r => setTimeout(r, 0));
    });
    expect(posted).toEqual([{path: "/runs/penn"}]);
    const pill = el.querySelector('[data-testid="map-load-pill"]')!;
    expect(pill.textContent).toContain("loading the scene graph");
    expect(pill.querySelector('[data-testid="map-stage-features"]')).toBeNull(); // skipped: not shown in the pill
    expect(el.querySelector('[data-testid="map-stage-graph"] [role="progressbar"]')!.getAttribute("aria-valuenow")).toBe("10");
  });
});
