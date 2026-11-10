import {beforeEach, describe, expect, it, vi} from "vitest";

const KEY = "adt4_console_token";

async function freshAuth() {
  vi.resetModules();
  return import("../src/lib/auth");
}

describe("auth token storage", () => {
  beforeEach(() => {
    localStorage.clear();
    sessionStorage.clear();
    window.history.replaceState(null, "", "/");
  });

  it("keeps a ?token= in localStorage (survives an iOS home-screen app restart) and strips it from the URL", async () => {
    window.history.replaceState(null, "", "/?token=abc#/fleet");
    const auth = await freshAuth();
    expect(auth.initToken()).toBe("abc");
    expect(localStorage.getItem(KEY)).toBe("abc");
    expect(window.location.search).toBe("");
    expect(window.location.hash).toBe("#/fleet");
    const again = await freshAuth();
    expect(again.initToken()).toBe("abc");
  });

  it("migrates a token an older build left in sessionStorage", async () => {
    sessionStorage.setItem(KEY, "old");
    const auth = await freshAuth();
    expect(auth.getToken()).toBe("old");
    expect(localStorage.getItem(KEY)).toBe("old");
    expect(sessionStorage.getItem(KEY)).toBeNull();
  });

  it("setToken(null) forgets the token everywhere", async () => {
    const auth = await freshAuth();
    auth.setToken("x");
    expect(auth.withToken("api/ws")).toBe("api/ws?token=x");
    auth.setToken(null);
    expect(localStorage.getItem(KEY)).toBeNull();
    expect(auth.withToken("api/ws")).toBe("api/ws");
  });
});
