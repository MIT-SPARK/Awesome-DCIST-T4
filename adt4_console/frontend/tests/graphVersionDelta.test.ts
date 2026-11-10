// A graph/<src>/version push that carries a delta (a change the server folded
// in) is applied in place; anything else reloads the snapshot.
import {afterEach, beforeEach, describe, expect, it, vi} from "vitest";
import {readFileSync} from "node:fs";
import {join} from "node:path";
import {decodeGraph, unpack} from "../src/codec";
import {GraphModel} from "../src/lib/graphModel";
import {useViewer} from "../src/stores/viewer";
import {ws} from "../src/ws";

const FIX = join(__dirname, "fixtures");
const bin = (n: string) => new Uint8Array(readFileSync(join(FIX, n)));
const js = (n: string) => JSON.parse(readFileSync(join(FIX, n), "utf8"));
const ID = "neo4j:penn";
const DELTA = unpack(bin("delta_small.msgpack")); // removes O(2), see codec.test.ts

function push(data: unknown) {
  ws.inject({topic: `graph/${ID}/version`, seq: 1, data} as never);
}

describe("graph version push with a delta", () => {
  let reload: ReturnType<typeof vi.fn>;
  beforeEach(() => {
    reload = vi.fn(async () => {});
    const model = GraphModel.fromSnapshot(decodeGraph(unpack(bin("graph_small.msgpack"))));
    useViewer.setState({
      reloadSource: reload,
      order: [ID],
      sources: {[ID]: {id: ID, kind: "neo4j", enabled: false, status: "ready", model, rev: model.rev, version: 5}},
    } as never); // prettier-ignore
    useViewer.getState().enableSource(ID, true); // subscribes graph/<ID>/
  });
  afterEach(() => useViewer.getState().enableSource(ID, false));

  it("applies a delta made against the version it holds, without a reload", () => {
    const before = useViewer.getState().sources[ID].rev;
    push({version: 6, base: 5, reason: "sam3", delta: DELTA});
    const src = useViewer.getState().sources[ID];
    expect(reload).not.toHaveBeenCalled();
    expect(src.version).toBe(6);
    expect(src.rev).toBeGreaterThan(before);
    expect(src.model!.nodes.has("O(2)")).toBe(false);
    expect([...src.model!.nodes.keys()].sort()).toEqual(Object.keys(js("delta_small.json").nodes).sort());
    // the same version again (e.g. a late plain push) changes nothing
    push({version: 6, reason: "neo4j"});
    expect(reload).not.toHaveBeenCalled();
  });

  it("reloads when the delta was made against another version (a frame was missed)", () => {
    push({version: 8, base: 7, reason: "sam3", delta: DELTA});
    expect(reload).toHaveBeenCalledTimes(1);
    const src = useViewer.getState().sources[ID];
    expect(src.version).toBe(8);
    expect(src.model!.nodes.has("O(2)")).toBe(true); // untouched: the snapshot replaces it
  });

  it("reloads on a plain version push, as before", () => {
    push({version: 6, reason: "neo4j"});
    expect(reload).toHaveBeenCalledTimes(1);
  });

  it("reloads when the source is still loading", () => {
    useViewer.getState().patchSource(ID, {status: "loading"});
    push({version: 6, base: 5, reason: "sam3", delta: DELTA});
    expect(reload).toHaveBeenCalledTimes(1);
  });
});
