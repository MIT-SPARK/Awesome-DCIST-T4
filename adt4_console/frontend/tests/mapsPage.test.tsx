// Maps page: this host's runs + the graph sources, open in viewer, load into Neo4j.
import {afterEach, beforeEach, describe, expect, it, vi} from "vitest";
import {act} from "react";
import {createRoot, type Root} from "react-dom/client";
import {MemoryRouter} from "react-router";
import {MapsPage} from "../src/pages/Maps";
import {useLiveMap} from "../src/stores/liveMap";

(globalThis as unknown as {IS_REACT_ACT_ENVIRONMENT: boolean}).IS_REACT_ACT_ENVIRONMENT = true;

const json = (body: unknown) => new Response(JSON.stringify(body), {status: 200, headers: {"Content-Type": "application/json"}});
const RUN = {name: "penn_courtyard", size: "1 GB", path: "penn_courtyard", abs_path: "/out/penn_courtyard", has_dsg: true, has_dsg_mesh: true, has_mesh: true, mtime: 1_760_000_000};
const BROKEN = {name: "crashed", size: "1 MB", path: "crashed", abs_path: "/out/crashed", has_dsg: false, has_dsg_mesh: false, has_mesh: false};

describe("MapsPage", () => {
  let host: HTMLDivElement;
  let root: Root;
  let calls: {url: string; method: string; body: unknown}[];
  beforeEach(() => {
    calls = [];
    useLiveMap.setState({map: null, choices: [], error: null});
    vi.spyOn(globalThis, "fetch").mockImplementation(async (input, init) => {
      const url = String(input);
      const method = init?.method ?? "GET";
      calls.push({url, method, body: init?.body ? JSON.parse(String(init.body)) : null});
      if (url.includes("api/maps?local=true")) return json({machines: {base: [RUN, BROKEN]}, base: "base"});
      if (url.includes("api/graph/sources")) return json([{id: "neo4j:live", kind: "neo4j", label: "live", version: 3, nodes: 1200}]);
      if (url.includes("api/maps/active")) return json({target: "live", run: null, name: null, has_images: false, has_features: false, ready: false, job_id: null, stages: []});
      if (url.includes("api/maps/activate")) return json({already: true});
      return json({});
    });
    host = document.createElement("div");
    document.body.appendChild(host);
    root = createRoot(host);
  });
  afterEach(() => {
    act(() => root.unmount());
    host.remove();
    vi.restoreAllMocks();
  });

  const render = async () => {
    await act(async () => {
      root.render(
        <MemoryRouter>
          <MapsPage />
        </MemoryRouter>,
      );
    });
    await act(async () => new Promise(r => setTimeout(r, 0)));
  };

  it("lists the local runs and the graph sources", async () => {
    await render();
    expect(calls.some(c => c.url.includes("api/maps?local=true"))).toBe(true);
    const rows = host.querySelectorAll('[data-testid="map-row"]');
    expect(rows).toHaveLength(2);
    expect(host.textContent).toContain("penn_courtyard");
    const sources = host.querySelectorAll('[data-testid="graph-source"]');
    expect(sources).toHaveLength(1);
    expect(sources[0].textContent).toContain("neo4j:live");
    // no fleet actions on this page
    expect(host.textContent).not.toMatch(/Transfer|Verify|Launch with map/);
  });

  it("loads a run into Neo4j through api/maps/activate; a run without a graph cannot", async () => {
    await render();
    const btn = (name: string) =>
      [...host.querySelectorAll<HTMLTableRowElement>('[data-testid="map-row"]')]
        .find(r => r.textContent?.includes(name))!
        .querySelector<HTMLButtonElement>("button")!;
    expect(btn("crashed").disabled).toBe(true);
    await act(async () => btn("penn_courtyard").click());
    const post = calls.find(c => c.url.includes("api/maps/activate"));
    expect(post?.method).toBe("POST");
    expect(post?.body).toEqual({path: "/out/penn_courtyard"});
  });
});
