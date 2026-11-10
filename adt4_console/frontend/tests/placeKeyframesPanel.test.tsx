import {afterEach, describe, expect, it, vi} from "vitest";
import {act} from "react";
import {createRoot, type Root} from "react-dom/client";
import {PlaceKeyframes, usePlaceReturn} from "../src/viewer/PlaceKeyframes";

(globalThis as unknown as {IS_REACT_ACT_ENVIRONMENT: boolean}).IS_REACT_ACT_ENVIRONMENT = true;

const json = (o: unknown) => new Response(JSON.stringify(o), {status: 200, headers: {"Content-Type": "application/json"}});
const flush = () => act(async () => void (await new Promise(r => setTimeout(r, 0))));
const T = Date.UTC(2026, 9, 22, 12) * 1e6;
const kf = (frame_id: number, robot_key: string, dt: number, visible: number) => ({frame_id, agent: `${robot_key}(${frame_id})`, robot_key, t_ns: T + dt * 1e9, visible, dist_m: 5, image: `/api/graph/file:x/frame/${frame_id}`});
const RESP = {
  src: "file:x", sym: "t(3)", version: 1, sort: "newest", n: 3, keyframes_total: 40, table: "ready", seconds: 0.01,
  keyframes: [kf(3, "b", 300, 0.4), kf(2, "a", 200, 1), kf(1, "a", 100, 0.6)],
  sessions: [
    {key: "b", session: "euclid", robot: "euclid", bag: "/bags/hard_robot_4_euclid/recorded_data", bag_short: "hard_robot_4_euclid", n: 1, first_ns: T + 300e9, last_ns: T + 300e9},
    {key: "a", session: "hamilton", robot: "hamilton", bag: null, bag_short: null, n: 2, first_ns: T + 100e9, last_ns: T + 200e9},
  ],
};

describe("PlaceKeyframes", () => {
  let root: Root | null = null;
  let host: HTMLDivElement | null = null;
  afterEach(() => {
    act(() => root?.unmount());
    host?.remove();
    vi.unstubAllGlobals();
    localStorage.clear();
  });

  async function mount(onShowPose = vi.fn(), resp: () => Response = () => json(RESP)) {
    const fetchMock = vi.fn(async () => resp());
    vi.stubGlobal("fetch", fetchMock);
    host = document.createElement("div");
    document.body.appendChild(host);
    root = createRoot(host);
    act(() => root!.render(<PlaceKeyframes src="file:x" sym="t(3)" onShowPose={onShowPose} hasPose={() => true} />));
    await flush();
    return {fetchMock, onShowPose};
  }
  const thumbs = () => [...host!.querySelectorAll<HTMLImageElement>(".place-kf-grid img")].map(i => i.getAttribute("src")!.match(/frame\/(\d+)/)![1]);
  const click = (el: Element) => act(() => (el as HTMLElement).click());
  const button = (text: string) => [...document.querySelectorAll("button")].find(b => b.textContent === text)!;

  it("lists the keyframes, newest first, with lazy thumbnails", async () => {
    const {fetchMock} = await mount();
    expect(String(fetchMock.mock.calls[0][0])).toBe("api/graph/file%3Ax/place/t(3)/keyframes?sort=newest");
    expect(host!.querySelector("h4")!.textContent).toBe("Keyframes (3)");
    expect(thumbs()).toEqual(["3", "2", "1"]);
    expect(host!.querySelector(".place-kf-grid img")!.getAttribute("loading")).toBe("lazy");
  });

  it("re-sorts without a request and groups by session", async () => {
    const {fetchMock} = await mount();
    click(button("Most visible"));
    expect(thumbs()).toEqual(["2", "1", "3"]);
    click(button("Oldest"));
    expect(thumbs()).toEqual(["1", "2", "3"]);
    expect(fetchMock).toHaveBeenCalledTimes(1);
    click(host!.querySelector(".toggle input")!);
    const heads = [...host!.querySelectorAll(".place-kf-session-name")].map(e => e.textContent);
    expect(heads).toEqual([expect.stringMatching(/ · hamilton$/), expect.stringMatching(/ · euclid · hard_robot_4_euclid$/)]);
    expect(localStorage.getItem("adt4.placeKf.group")).toBe("1");
  });

  it("the lightbox selects the keyframe's pose and remembers the place", async () => {
    const {onShowPose} = await mount();
    click(host!.querySelectorAll(".place-kf-grid button")[1]);
    expect(document.querySelector(".modal")!.textContent).toContain("keyframe 2/3");
    click(button("Next"));
    expect(document.querySelector(".modal")!.textContent).toContain("keyframe 3/3");
    click(button("Show pose on map"));
    expect(onShowPose).toHaveBeenCalledWith("a(1)");
    expect(usePlaceReturn.getState()).toMatchObject({from: {src: "file:x", sym: "t(3)"}, pose: "a(1)"});
    expect(document.querySelector(".modal")).toBeNull();
  });

  it("errors show, but a server without the route shows nothing", async () => {
    await mount(vi.fn(), () => new Response(JSON.stringify({detail: "t(3) is not a traversability place"}), {status: 404}));
    expect(host!.textContent).toContain("Keyframes: t(3) is not a traversability place");
    act(() => root?.unmount());
    host?.remove();
    await mount(vi.fn(), () => new Response(JSON.stringify({detail: "Not Found"}), {status: 404}));
    expect(host!.textContent).toBe("");
  });
});
