import {describe, expect, it} from "vitest";
import {groupBySession, kfBadge, kfClock, kfDate, sessionName, sessionTitle, sortKeyframes, type PlaceKeyframe, type PlaceKfSession} from "../src/lib/placeKeyframes";

const S = 1e9;
const DAY1 = Date.UTC(2026, 9, 22, 12) * 1e6; // ns
const DAY2 = Date.UTC(2026, 9, 24, 12) * 1e6;
const kf = (frame_id: number, robot_key: string, t_ns: number, visible: number, dist_m: number): PlaceKeyframe => ({frame_id, agent: `${robot_key}(${frame_id})`, robot_key, t_ns, visible, dist_m, image: ""});
const KFS = [kf(1, "a", DAY1 + 10 * S, 0.5, 4), kf(2, "a", DAY1 + 20 * S, 1, 6), kf(3, "b", DAY2 + 5 * S, 1, 3), kf(4, "c", DAY2 + 50 * S, 0.2, 9)];
const SESSIONS: PlaceKfSession[] = [
  {key: "a", session: "hamilton", robot: "hamilton", bag: "/nas/hard_robot_2_hamilton/recorded_data", bag_short: "hard_robot_2_hamilton", n: 2, first_ns: DAY1, last_ns: DAY1},
  {key: "b", session: "euclid", robot: "euclid", bag: null, bag_short: null, n: 1, first_ns: DAY2, last_ns: DAY2},
];
const ids = (ks: PlaceKeyframe[]) => ks.map(k => k.frame_id);

describe("place keyframes: sorting", () => {
  it("sorts by date both ways and by visibility (nearer first on ties)", () => {
    expect(ids(sortKeyframes(KFS, "newest"))).toEqual([4, 3, 2, 1]);
    expect(ids(sortKeyframes(KFS, "oldest"))).toEqual([1, 2, 3, 4]);
    expect(ids(sortKeyframes(KFS, "visible"))).toEqual([3, 2, 1, 4]);
  });

  it("does not modify its input", () => {
    const copy = KFS.slice();
    sortKeyframes(KFS, "oldest");
    expect(KFS).toEqual(copy);
  });
});

describe("place keyframes: grouping by session", () => {
  it("groups by robot key, newest session first, each group sorted", () => {
    const g = groupBySession(KFS, SESSIONS, "newest");
    expect(g.map(x => x.session.key)).toEqual(["c", "b", "a"]);
    expect(ids(g[2].items)).toEqual([2, 1]);
    expect(g[2].session.robot).toBe("hamilton");
    expect(g[2].session.n).toBe(2);
    expect(g[2].session.first_ns).toBe(DAY1 + 10 * S); // from this place's keyframes
  });

  it("oldest first, and by best visibility", () => {
    expect(groupBySession(KFS, SESSIONS, "oldest").map(x => x.session.key)).toEqual(["a", "b", "c"]);
    const v = groupBySession(KFS, SESSIONS, "visible");
    expect(v.map(x => x.session.key)).toEqual(["a", "b", "c"]); // a and b tie at 100%: earlier session first
    expect(ids(v[0].items)).toEqual([2, 1]);
  });

  it("a robot without a session still gets a group", () => {
    const g = groupBySession(KFS, SESSIONS, "newest");
    expect(g[0].session).toMatchObject({key: "c", n: 1});
    expect(sessionTitle(g[0].session)).toBe(`${kfDate(DAY2)} · robot c`);
  });

  it("is empty without keyframes", () => {
    expect(groupBySession([], SESSIONS, "newest")).toEqual([]);
  });
});

describe("place keyframes: labels", () => {
  it("session titles: date, robot, bag short name (else session)", () => {
    expect(sessionTitle(SESSIONS[0])).toBe(`${kfDate(DAY1)} · hamilton · hard_robot_2_hamilton`);
    expect(sessionTitle(SESSIONS[1])).toBe(`${kfDate(DAY2)} · euclid`); // the session is the robot's name
    expect(sessionName(SESSIONS[0])).toBe("hamilton · hard_robot_2_hamilton");
    expect(sessionName({key: "b", n: 0, first_ns: 0, last_ns: 0})).toBe("robot b");
    expect(sessionTitle({key: "b", n: 0, first_ns: 0, last_ns: 0})).toBe("robot b"); // no date
  });

  it("dates only for clock times", () => {
    expect(kfDate(DAY1)).toMatch(/^2026-10-2[12]$/);
    expect(kfClock(DAY1)).toMatch(/^\d\d:\d\d:\d\d$/);
    expect(kfDate(42 * S)).toBe("");
    expect(kfClock(42 * S)).toBe("#42");
  });

  it("badge: visibility and distance", () => {
    expect(kfBadge(KFS[0])).toBe("50% · 4.0 m");
  });
});
