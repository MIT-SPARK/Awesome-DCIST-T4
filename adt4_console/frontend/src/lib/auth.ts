// Token handling: `?token=` is read once from the URL and kept in localStorage.
// localStorage, not sessionStorage: an iOS home-screen web app loses
// sessionStorage every time it is closed. Tokens saved by older builds in
// sessionStorage are migrated on first read.
const KEY = "adt4_console_token";
let cached: string | null | undefined;

function safeGet(): string | null {
  try {
    const t = localStorage.getItem(KEY);
    if (t) return t;
    const legacy = sessionStorage.getItem(KEY);
    if (legacy) {
      localStorage.setItem(KEY, legacy);
      sessionStorage.removeItem(KEY);
    }
    return legacy;
  } catch {
    return null;
  }
}

function safeSet(t: string | null) {
  try {
    if (t) localStorage.setItem(KEY, t);
    else localStorage.removeItem(KEY);
    sessionStorage.removeItem(KEY);
  } catch {
    /* private mode: keep in memory */
  }
}

export function initToken(): string | null {
  const url = new URL(window.location.href);
  const t = url.searchParams.get("token");
  if (t) {
    safeSet(t);
    cached = t;
    url.searchParams.delete("token");
    window.history.replaceState(window.history.state, "", url.toString());
    return t;
  }
  cached = safeGet();
  return cached;
}

export function getToken(): string | null {
  if (cached === undefined) cached = safeGet();
  return cached;
}

export function setToken(t: string | null) {
  cached = t;
  safeSet(t);
}

/** Append ?token= to a same-origin URL (images, websocket). */
export function withToken(url: string): string {
  const t = getToken();
  if (!t) return url;
  return url + (url.includes("?") ? "&" : "?") + "token=" + encodeURIComponent(t);
}
