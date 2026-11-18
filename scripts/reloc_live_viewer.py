#!/usr/bin/env python3
"""Live relocalization viewer: horizontal error vs GPS (or a GT CSV) over time, and the reference
and estimated tracks on the aerial ortho, served as a small self-contained web page.

    source $ADT4_WS/install/setup.zsh
    # live robot: GPS from /<robot>/fix, prior sessions' trajectories from the prior map
    python3 $ADT4_WS/src/awesome_dcist_t4/scripts/reloc_live_viewer.py <robot> \
        --origin 482937.0 4421340.0 --prior-map <prior map dir>
    # bag replay scored against a GT CSV instead of GPS
    python3 .../reloc_live_viewer.py <robot> --sim-time --gt-csv gt.csv
then open http://localhost:8765 (or http://<this machine>:8765 from another computer).

Estimate: TF map_frame <- base_frame, read from /<robot>/tf and /<robot>/tf_static (no remaps
needed). Reference: NavSatFix on --gps-topic converted to UTM minus --origin (the ADT4 map frame),
or --gt-csv (UTM, #timestamp_kf in ns) interpolated at the estimate's stamp. Error is horizontal
only and ignores the GPS antenna's offset from base_link. Status/inliers come from the robot's
ros_system_monitor messages whose nickname contains --status-node.

No third-party JS: works offline. Python deps: rclpy, tf2_ros, Pillow (for the ortho).
"""

import argparse
import bisect
import csv
import io
import json
import math
import os
import re
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import rclpy
import rclpy.duration
import rclpy.time
import tf2_ros
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import NavSatFix
from tf2_msgs.msg import TFMessage

try:
    from ros_system_monitor_msgs.msg import NodeInfoMsg
except ImportError:
    NodeInfoMsg = None

DEFAULT_ORTHO = os.path.expanduser("~/data/aerial/pennovation/penn_2024.tif")


def latlon_to_utm(lat, lon, zone=None):
    """WGS84 lat/lon (deg) -> UTM easting/northing (m), northern hemisphere aware."""
    a, f, k0 = 6378137.0, 1 / 298.257223563, 0.9996
    e2 = f * (2 - f)
    ep2 = e2 / (1 - e2)
    if zone is None:
        zone = int((lon + 180) / 6) + 1
    lon0 = math.radians((zone - 1) * 6 - 180 + 3)
    phi, lam = math.radians(lat), math.radians(lon)
    N = a / math.sqrt(1 - e2 * math.sin(phi) ** 2)
    T = math.tan(phi) ** 2
    C = ep2 * math.cos(phi) ** 2
    A = math.cos(phi) * (lam - lon0)
    M = a * (
        (1 - e2 / 4 - 3 * e2**2 / 64 - 5 * e2**3 / 256) * phi
        - (3 * e2 / 8 + 3 * e2**2 / 32 + 45 * e2**3 / 1024) * math.sin(2 * phi)
        + (15 * e2**2 / 256 + 45 * e2**3 / 1024) * math.sin(4 * phi)
        - (35 * e2**3 / 3072) * math.sin(6 * phi)
    )
    x = (
        k0
        * N
        * (
            A
            + (1 - T + C) * A**3 / 6
            + (5 - 18 * T + T**2 + 72 * C - 58 * ep2) * A**5 / 120
        )
    )
    y = k0 * (
        M
        + N
        * math.tan(phi)
        * (
            A**2 / 2
            + (5 - T + 9 * C + 4 * C**2) * A**4 / 24
            + (61 - 58 * T + T**2 + 600 * C - 330 * ep2) * A**6 / 720
        )
    )
    x += 500000.0
    if lat < 0:
        y += 10000000.0
    return x, y


def load_ortho(path, origin, max_px):
    """Downsample a GeoTIFF ortho (north-up, ModelTiepoint + ModelPixelScale tags) to a JPEG and
    return (jpeg bytes, [xmin, xmax, ymin, ymax] in the map frame)."""
    from PIL import Image

    Image.MAX_IMAGE_PIXELS = None
    im = Image.open(path)
    _, _, _, X0, Y0, _ = im.tag_v2[33922]
    sx, sy = im.tag_v2[33550][:2]
    w, h = im.size
    factor = max(1, math.ceil(max(w, h) / max_px))
    small = im.reduce(factor).convert("RGB")
    buf = io.BytesIO()
    small.save(buf, "JPEG", quality=85)
    extent = [
        X0 - origin[0],
        X0 + w * sx - origin[0],
        Y0 - h * sy - origin[1],
        Y0 - origin[1],
    ]
    return buf.getvalue(), extent


def load_priors(prior_dir, spacing_m=1.0):
    """Prior sessions' trajectories (already in the map frame) from a fused prior map:
    <prior>/hydra/backend/robot_<id>_trajectory.csv, named from <prior>/robots.yaml if present."""
    import glob

    names = {}
    try:
        import yaml

        for r in yaml.safe_load(open(os.path.join(prior_dir, "robots.yaml")))["robots"]:
            names[int(r["id"])] = r["name"]
    except Exception:
        pass
    # only sessions ROMAN can match against (a robot_N trajectory without roman/N.roman_map.pkl
    # cannot produce loop closures)
    roman_dir = os.path.join(prior_dir, "roman")
    have_roman = os.path.isdir(roman_dir)
    out = []
    for f in sorted(
        glob.glob(os.path.join(prior_dir, "hydra", "backend", "robot_*_trajectory.csv"))
    ):
        rid = int(re.search(r"robot_(\d+)_trajectory", f).group(1))
        if have_roman and not os.path.exists(
            os.path.join(roman_dir, f"{rid}.roman_map.pkl")
        ):
            continue
        xy, last = [], None
        with open(f) as fh:
            for r in csv.DictReader(fh):
                p = (float(r["x"]), float(r["y"]))
                if last is None or math.dist(p, last) >= spacing_m:
                    xy.append([round(p[0], 2), round(p[1], 2)])
                    last = p
        out.append(dict(id=rid, name=names.get(rid, f"robot {rid}"), xy=xy))
    return out


def load_gt(path, origin):
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            rows.append(
                (
                    int(r["#timestamp_kf"]) * 1e-9,
                    float(r["x"]) - origin[0],
                    float(r["y"]) - origin[1],
                )
            )
    rows.sort()
    return rows


class Viewer(Node):
    def __init__(self, a):
        super().__init__(
            "reloc_live_viewer",
            parameter_overrides=[
                Parameter("use_sim_time", Parameter.Type.BOOL, a.sim_time)
            ],
        )
        self.a = a
        self.lock = threading.Lock()
        self.origin = a.origin
        self.base_frame = a.base_frame or f"{a.robot}/base_link"
        self.est = []  # [t, x, y, yaw_deg]
        self.ref = []  # [t, x, y, fix_status]
        self.err = []  # [t, err_m]
        self.inl = []  # [t, inliers, candidates]
        self.status = dict(code=None, note="no status yet", wall=None)
        self.last_ref_wall = None
        self.t0 = None
        self.log = open(a.log_csv, "a", buffering=1) if a.log_csv else None
        if self.log and os.path.getsize(a.log_csv) == 0:
            self.log.write("kind,t,a,b,c\n")

        self.buf = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=600))
        self.create_subscription(TFMessage, f"/{a.robot}/tf", self.tf_cb, 200)
        self.create_subscription(
            TFMessage,
            f"/{a.robot}/tf_static",
            self.tf_static_cb,
            QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.gt = load_gt(a.gt_csv, self.origin) if a.gt_csv else None
        self.gt_t = [r[0] for r in self.gt] if self.gt else None
        if not self.gt:
            self.create_subscription(
                NavSatFix, a.gps_topic or f"/{a.robot}/fix", self.fix_cb, 50
            )
        if NodeInfoMsg is not None:
            self.create_subscription(
                NodeInfoMsg,
                f"/{a.robot}/ros_system_monitor/node_diagnostic_collector",
                self.status_cb,
                100,
            )
        # event timers (ROS clock, so bag replays work); raw subscriptions: only arrivals matter
        self.odom_frame = a.odom_frame or f"{a.robot}/odom"
        self.ev = dict(lc=None, submap=None, inl_up=None)
        self.counts = dict(lc=0, submap=0)
        self.inl_prev = None
        self.start_clock = None
        self.dist = 0.0
        self.odom_anchor = None
        self.first_loc = None
        try:
            from roman_msgs.msg import LoopClosure, Submap

            self.create_subscription(
                LoopClosure,
                f"/{a.robot}/roman/roman_lc/loop_closure",
                lambda _: self.event("lc"),
                100,
                raw=True,
            )
            self.create_subscription(
                Submap,
                f"/{a.robot}/roman/submap",
                lambda _: self.event("submap"),
                20,
                raw=True,
            )
        except ImportError:
            self.get_logger().warn(
                "roman_msgs not found: no loop closure / submap timers"
            )
        self.create_timer(0.2, self.est_cb)
        self.create_timer(0.2, self.odom_cb)

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def event(self, kind):
        with self.lock:
            self.ev[kind] = self.now_s()
            self.counts[kind] += 1

    def odom_cb(self):
        now = self.now_s()
        if now <= 0:
            return  # sim time without /clock yet
        if self.start_clock is None:
            self.start_clock = now
        try:
            tf = self.buf.lookup_transform(
                self.odom_frame, self.base_frame, rclpy.time.Time()
            )
        except tf2_ros.TransformException:
            return
        p = (tf.transform.translation.x, tf.transform.translation.y)
        if self.odom_anchor is None:
            self.odom_anchor = p
        elif math.dist(p, self.odom_anchor) > 0.1:  # ignore jitter while stationary
            with self.lock:
                self.dist += math.dist(p, self.odom_anchor)
            self.odom_anchor = p

    # --- inputs -----------------------------------------------------------------------------
    def tf_cb(self, msg):
        for tr in msg.transforms:
            self.buf.set_transform(tr, "viewer")

    def tf_static_cb(self, msg):
        for tr in msg.transforms:
            self.buf.set_transform_static(tr, "viewer")

    def lookup(self, stamp=None):
        t = rclpy.time.Time() if stamp is None else rclpy.time.Time(seconds=stamp)
        tf = self.buf.lookup_transform(self.a.map_frame, self.base_frame, t)
        p, q = tf.transform.translation, tf.transform.rotation
        yaw = math.degrees(
            math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))
        )
        st = tf.header.stamp.sec + 1e-9 * tf.header.stamp.nanosec
        return st, p.x, p.y, yaw

    def _t(self, t):
        if self.t0 is None:
            self.t0 = t
        return round(t - self.t0, 3)

    def est_cb(self):
        try:
            st, x, y, yaw = self.lookup()
        except tf2_ros.TransformException:
            # not localized yet: still draw the GT track (at the current clock) if scoring vs GT
            if self.gt:
                now = self.get_clock().now().nanoseconds * 1e-9
                i = bisect.bisect_left(self.gt_t, now)
                if 0 < i < len(self.gt):
                    with self.lock:
                        if not self.ref or now - self.t0 > self.ref[-1][0] + 0.15:
                            t = self._t(now)
                            self.ref.append(
                                [t, round(self.gt[i][1], 3), round(self.gt[i][2], 3), 2]
                            )
                            self.last_ref_wall = time.time()
            return
        with self.lock:
            if self.first_loc is None and self.start_clock is not None:
                self.first_loc = dict(
                    after_s=self.now_s() - self.start_clock, dist_m=self.dist
                )
            if self.est and st <= self.est[-1][0] + self.t0:
                return  # no new TF since the last sample
            t = self._t(st)
            self.est.append([t, round(x, 3), round(y, 3), round(yaw, 2)])
            if self.gt:
                i = bisect.bisect_left(self.gt_t, st)
                if 0 < i < len(self.gt):
                    (ta, xa, ya), (tb, xb, yb) = self.gt[i - 1], self.gt[i]
                    w = (st - ta) / (tb - ta) if tb > ta else 0
                    rx, ry = xa + w * (xb - xa), ya + w * (yb - ya)
                    self.ref.append([t, round(rx, 3), round(ry, 3), 2])
                    e = math.hypot(x - rx, y - ry)
                    self.err.append([t, round(e, 3)])
                    self.last_ref_wall = time.time()
                    if self.log:
                        self.log.write(f"gt_err,{st:.3f},{e:.4f},{x:.3f},{y:.3f}\n")
            if self.log:
                self.log.write(f"est,{st:.3f},{x:.3f},{y:.3f},{yaw:.2f}\n")

    def fix_cb(self, msg):
        if msg.status.status < self.a.min_fix_status or math.isnan(msg.latitude):
            return
        e, n = latlon_to_utm(msg.latitude, msg.longitude, self.a.utm_zone)
        rx, ry = e - self.origin[0], n - self.origin[1]
        st = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        if st == 0:
            st = self.get_clock().now().nanoseconds * 1e-9
        try:
            _, x, y, _ = self.lookup(st)
        except tf2_ros.TransformException:
            try:  # fall back to the latest estimate if it is close in time
                se, x, y, _ = self.lookup()
                if abs(se - st) > 0.5:
                    x = None
            except tf2_ros.TransformException:
                x = None
        # horizontal 1-sigma from the receiver's covariance (the larger of east/north), if reported
        cov = msg.position_covariance
        sigma = None
        if (
            msg.position_covariance_type != NavSatFix.COVARIANCE_TYPE_UNKNOWN
            and max(cov[0], cov[4]) > 0
        ):
            sigma = round(math.sqrt(max(cov[0], cov[4])), 3)
        with self.lock:
            t = self._t(st)
            self.ref.append(
                [t, round(rx, 3), round(ry, 3), int(msg.status.status), sigma]
            )
            self.last_ref_wall = time.time()
            if x is not None:
                e_m = math.hypot(x - rx, y - ry)
                self.err.append([t, round(e_m, 3), sigma])
                if self.log:
                    self.log.write(
                        f"gps_err,{st:.3f},{e_m:.4f},{rx:.3f},{ry:.3f},{sigma}\n"
                    )

    def status_cb(self, msg):
        if self.a.status_node not in msg.nickname:
            return
        m = re.search(r"(\d+) / (\d+) inliers", msg.notes)
        with self.lock:
            self.status = dict(code=int(msg.status), note=msg.notes, wall=time.time())
            if m:
                n_inl = int(m.group(1))
                if self.inl_prev is not None and n_inl > self.inl_prev:
                    self.ev["inl_up"] = self.now_s()
                self.inl_prev = n_inl
            if m and self.t0 is not None:
                self.inl.append(
                    [self._t(self.now_s()), int(m.group(1)), int(m.group(2))]
                )

    # --- web --------------------------------------------------------------------------------
    def snapshot(self, since):
        with self.lock:
            out = {
                k: getattr(self, k)[since.get(k, 0) :]
                for k in ("est", "ref", "err", "inl")
            }
            out["n"] = {k: len(getattr(self, k)) for k in ("est", "ref", "err", "inl")}
            out["status"] = self.status
            out["ref_age_s"] = (
                (time.time() - self.last_ref_wall) if self.last_ref_wall else None
            )
            out["mode"] = "gt" if self.gt else "gps"
            now = self.now_s()
            out["ages"] = {
                k: (None if v is None else round(now - v, 1))
                for k, v in self.ev.items()
            }
            out["counts"] = dict(self.counts)
            out["dist_m"] = round(self.dist, 1)
            out["uptime_s"] = (
                None if self.start_clock is None else round(now - self.start_clock, 1)
            )
            out["first_loc"] = self.first_loc
            return out


PAGE = r"""<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Relocalization Live</title>
<style>
:root{--bg:#0f1115;--panel:#181b22;--ink:#e6e8ee;--muted:#9aa3b2;--line:#2a2f3a;--est:#22d3ee;
--ref:#ffffff;--bad:#f43f5e;--ok:#34d399;--warn:#fbbf24;--inl:#a78bfa}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);
font:14px/1.4 system-ui,-apple-system,Segoe UI,Roboto,sans-serif}
header{display:flex;flex-wrap:wrap;gap:8px;padding:10px 16px;border-bottom:1px solid var(--line)}
.tile{background:var(--panel);border:1px solid var(--line);border-radius:8px;padding:6px 12px;min-width:120px}
.tile .k{color:var(--muted);font-size:12px}.tile .v{font-size:20px;font-variant-numeric:tabular-nums}
main{display:grid;grid-template-columns:minmax(0,1.1fr) minmax(0,1fr);gap:12px;padding:12px 16px}
@media (max-width:900px){main{grid-template-columns:1fr}}
.card{background:var(--panel);border:1px solid var(--line);border-radius:8px;padding:10px;min-width:0}
.card h2{font-size:13px;font-weight:600;color:var(--muted);margin:0 0 6px}
canvas{width:100%;display:block;border-radius:4px}
.ctl{display:flex;flex-wrap:wrap;gap:8px;align-items:center;margin:6px 0;color:var(--muted);font-size:12px}
.ctl input[type=number]{width:70px;background:#0b0d11;color:var(--ink);border:1px solid var(--line);border-radius:4px;padding:2px 4px}
button,select{background:#0b0d11;color:var(--ink);border:1px solid var(--line);border-radius:4px;padding:3px 8px;cursor:pointer}
.legend span{margin-right:12px}.sw{display:inline-block;width:14px;height:3px;vertical-align:middle;margin-right:4px}
#note{color:var(--muted);font-size:12px;padding:0 16px 10px}
</style></head><body>
<header>
 <div class="tile"><div class="k">current error</div><div class="v" id="cur">–</div></div>
 <div class="tile"><div class="k">max error</div><div class="v" id="max">–</div></div>
 <div class="tile"><div class="k">median error</div><div class="v" id="med">–</div></div>
 <div class="tile"><div class="k">time over threshold</div><div class="v" id="over">–</div></div>
 <div class="tile"><div class="k">relocalization</div><div class="v" id="st">–</div></div>
 <div class="tile"><div class="k">inliers</div><div class="v" id="inl">–</div></div>
 <div class="tile"><div class="k" id="refk">reference age</div><div class="v" id="age">–</div></div>
</header>
<header>
 <div class="tile"><div class="k">since last loop closure</div><div class="v" id="a_lc">–</div></div>
 <div class="tile"><div class="k">since inliers increased</div><div class="v" id="a_inl">–</div></div>
 <div class="tile"><div class="k">since last submap</div><div class="v" id="a_sm">–</div></div>
 <div class="tile"><div class="k">distance traveled</div><div class="v" id="dist">–</div></div>
 <div class="tile"><div class="k">first relocalized</div><div class="v" id="floc">–</div></div>
 <div class="tile"><div class="k">viewer uptime</div><div class="v" id="up">–</div></div>
</header>
<main>
 <div class="card"><h2>Aerial map</h2>
  <div class="ctl"><label><input type="checkbox" id="follow" checked> follow robot</label>
   <button id="fit">fit tracks</button>
   <label>trail <select id="trail"><option value="0">all</option><option value="300">5 min</option>
   <option value="60">1 min</option></select></label>
   <label><input type="checkbox" id="priors" checked> prior trajectories</label>
   <label>GPS uncertainty <select id="sig"><option value="0">off</option><option value="1" selected>±1σ</option>
   <option value="2">±2σ</option></select></label>
   <span class="legend"><span><i class="sw" style="background:var(--ref)"></i><span id="refname">GPS</span></span>
   <span><i class="sw" style="background:var(--est)"></i>estimate</span></span></div>
  <canvas id="map" height="560"></canvas>
  <div class="ctl" id="plegend"></div>
  <div class="ctl">scroll to zoom, drag to pan</div></div>
 <div class="card"><h2>Horizontal error over time</h2>
  <div class="ctl">
   <label>x <select id="xmode"><option value="all">whole run</option><option value="last">last N s</option>
   <option value="range">range</option></select></label>
   <label>N <input type="number" id="xlast" value="300" min="10"></label>
   <label>from <input type="number" id="xmin" value="0"></label><label>to <input type="number" id="xmax" value="600"></label>
   <label>y max <input type="number" id="ymax" placeholder="auto" step="0.5"></label>
   <label><input type="checkbox" id="ylog"> log</label>
   <label>threshold <input type="number" id="thr" value="3" step="0.5"></label></div>
  <canvas id="err" height="330"></canvas>
  <h2 style="margin-top:10px">Inlier rate</h2>
  <canvas id="rate" height="130"></canvas></div>
</main>
<div id="note"></div>
<script>
const D={est:[],ref:[],err:[],inl:[]};let META=null,IMG=null,status={},mode="gps",refAge=null,EXTRA={},PRIORS=[];
// prior-session colors: fixed order, chosen to stay visible on imagery and distinct from the
// live tracks (white = reference, cyan = estimate)
const PCOL=["#ff6b6b","#ffd43b","#69db7c","#b197fc","#ff922b","#f783ac","#a9e34b","#e599f7","#ffa8a8","#c0eb75"];
function priorLegend(){const el=$("plegend");if(!PRIORS.length){el.textContent="no prior trajectories (pass --prior-map)";return}
 el.innerHTML=PRIORS.map((p,i)=>`<span><i class="sw" style="background:${PCOL[i%PCOL.length]}"></i>${p.id}: ${p.name}</span>`).join(" ");
 el.style.display=$("priors").checked?"":"none";}
const dur=s=>s==null?"–":s<60?s.toFixed(0)+" s":Math.floor(s/60)+" min "+String(Math.floor(s%60)).padStart(2,"0")+" s";
function ageTile(id,s,n){const el=$(id);el.textContent=s==null?"none yet":dur(s)+(n!=null?` (${n})`:"");
 el.style.color=s==null?"":s>180?css("--bad"):s>60?css("--warn"):"";}
function extraTiles(){const a=EXTRA.ages||{},c=EXTRA.counts||{};
 ageTile("a_lc",a.lc,c.lc);ageTile("a_inl",a.inl_up,null);ageTile("a_sm",a.submap,c.submap);
 $("dist").textContent=EXTRA.dist_m==null?"–":EXTRA.dist_m.toFixed(0)+" m";$("up").textContent=dur(EXTRA.uptime_s);
 const f=EXTRA.first_loc;$("floc").textContent=!f?"not yet":f.after_s<1?"before viewer start":`${dur(f.after_s)} / ${f.dist_m.toFixed(0)} m`;
 $("floc").style.color=f?css("--ok"):css("--warn");}
const $=id=>document.getElementById(id);
const css=n=>getComputedStyle(document.documentElement).getPropertyValue(n).trim();
function sizeCanvas(c){const r=c.getBoundingClientRect(),d=devicePixelRatio||1;
 const h=+c.getAttribute("height");c.width=r.width*d;c.height=h*d;c.style.height=h+"px";
 const g=c.getContext("2d");g.setTransform(d,0,0,d,0,0);return [g,r.width,h];}
async function poll(){try{
 const q=Object.keys(D).map(k=>k+"="+D[k].length).join("&");
 const r=await fetch("data?"+q);const j=await r.json();
 for(const k of Object.keys(D))D[k].push(...j[k]);
 status=j.status;mode=j.mode;refAge=j.ref_age_s;EXTRA=j;draw();}catch(e){$("note").textContent="connection lost: "+e}
 setTimeout(poll,1000);}
// ---------------- map -----------------
let view=null,drag=null;
function fitTracks(){const P=[...D.est,...D.ref];if(!P.length&&META){const e=META.extent;
 view={cx:(e[0]+e[1])/2,cy:(e[2]+e[3])/2,s:Math.max(e[1]-e[0],e[3]-e[2])/600};return}
 if(!P.length)return;let a=1e9,b=-1e9,c=1e9,d=-1e9;for(const p of P){a=Math.min(a,p[1]);b=Math.max(b,p[1]);c=Math.min(c,p[2]);d=Math.max(d,p[2])}
 const w=$("map").getBoundingClientRect().width,h=560;
 view={cx:(a+b)/2,cy:(c+d)/2,s:Math.max((b-a+40)/w,(d-c+40)/h,120/Math.min(w,h))};}
function drawMap(){const [g,w,h]=sizeCanvas($("map"));g.fillStyle="#000";g.fillRect(0,0,w,h);
 if(!view)fitTracks();if(!view)return;
 const last=D.est[D.est.length-1];if($("follow").checked&&last&&!drag){view.cx=last[1];view.cy=last[2];}
 const X=x=>w/2+(x-view.cx)/view.s,Y=y=>h/2-(y-view.cy)/view.s;
 if(IMG&&META){const e=META.extent;g.drawImage(IMG,X(e[0]),Y(e[3]),(e[1]-e[0])/view.s,(e[3]-e[2])/view.s)}
 if($("priors").checked)PRIORS.forEach((p,i)=>{g.lineWidth=3.5;g.strokeStyle="rgba(0,0,0,.55)";
  for(const pass of[0,1]){if(pass){g.lineWidth=1.6;g.strokeStyle=PCOL[i%PCOL.length]}g.beginPath();
   p.xy.forEach((q,k)=>{const gap=k>0&&Math.hypot(q[0]-p.xy[k-1][0],q[1]-p.xy[k-1][1])>15;
    if(k==0||gap)g.moveTo(X(q[0]),Y(q[1]));else g.lineTo(X(q[0]),Y(q[1]))});g.stroke()}});
 const trail=+$("trail").value,tEnd=last?last[0]:0;
 const path=(P,col,lw)=>{g.strokeStyle=col;g.lineWidth=lw;g.beginPath();let on=false;
  for(let i=0;i<P.length;i++){const p=P[i];if(trail&&p[0]<tEnd-trail)continue;
   const gap=i>0&&p[0]-P[i-1][0]>5;if(!on||gap){g.moveTo(X(p[1]),Y(p[2]));on=true}else g.lineTo(X(p[1]),Y(p[2]))}g.stroke();};
 path(D.ref,"rgba(0,0,0,.7)",5);path(D.ref,css("--ref"),2.5);path(D.est,"rgba(0,0,0,.6)",4);path(D.est,css("--est"),2);
 const r=D.ref[D.ref.length-1];
 if(r&&last){g.strokeStyle=css("--bad");g.lineWidth=1.5;g.setLineDash([4,3]);g.beginPath();g.moveTo(X(r[1]),Y(r[2]));g.lineTo(X(last[1]),Y(last[2]));g.stroke();g.setLineDash([])}
 const ks=+$("sig").value;
 if(r&&ks&&r[4]){const rad=ks*r[4]/view.s;g.fillStyle="rgba(255,255,255,.12)";g.strokeStyle="rgba(255,255,255,.8)";
  g.lineWidth=1.5;g.setLineDash([5,4]);g.beginPath();g.arc(X(r[1]),Y(r[2]),rad,0,7);g.fill();g.stroke();g.setLineDash([])}
 if(r){g.fillStyle=css("--ref");g.strokeStyle="#000";g.lineWidth=2;g.beginPath();g.arc(X(r[1]),Y(r[2]),6,0,7);g.fill();g.stroke()}
 if(last){const x=X(last[1]),y=Y(last[2]),th=last[3]*Math.PI/180;g.fillStyle=css("--est");g.strokeStyle="#000";g.lineWidth=2;
  g.beginPath();g.moveTo(x+13*Math.cos(th),y-13*Math.sin(th));g.lineTo(x+7*Math.cos(th+2.5),y-7*Math.sin(th+2.5));
  g.lineTo(x+7*Math.cos(th-2.5),y-7*Math.sin(th-2.5));g.closePath();g.fill();g.stroke()}
 // scale bar
 const m=[1,2,5,10,20,50,100,200,500].find(v=>v/view.s>60)||1000;g.fillStyle="#fff";g.fillRect(12,h-18,m/view.s,4);
 g.font="12px system-ui";g.fillText(m+" m",12,h-24);}
const mc=$("map");
mc.addEventListener("wheel",e=>{e.preventDefault();if(!view)return;const r=mc.getBoundingClientRect(),w=r.width,h=560;
 const mx=view.cx+(e.offsetX-w/2)*view.s,my=view.cy-(e.offsetY-h/2)*view.s,k=Math.exp(e.deltaY*0.0015);
 view.s*=k;view.cx=mx-(e.offsetX-w/2)*view.s;view.cy=my+(e.offsetY-h/2)*view.s;$("follow").checked=false;drawMap();},{passive:false});
mc.addEventListener("mousedown",e=>{drag={x:e.clientX,y:e.clientY,cx:view.cx,cy:view.cy};$("follow").checked=false});
addEventListener("mousemove",e=>{if(!drag)return;view.cx=drag.cx-(e.clientX-drag.x)*view.s;view.cy=drag.cy+(e.clientY-drag.y)*view.s;drawMap()});
addEventListener("mouseup",()=>drag=null);
$("fit").onclick=()=>{$("follow").checked=false;fitTracks();drawMap()};
// ---------------- plots ---------------
function niceTicks(a,b,n){const s=(b-a)/n,p=Math.pow(10,Math.floor(Math.log10(s))),m=[1,2,5,10].find(v=>v*p>=s)*p;
 const t=[];for(let v=Math.ceil(a/m)*m;v<=b+1e-9;v+=m)t.push(+v.toFixed(6));return t}
function xRange(){const tEnd=Math.max(D.err.length?D.err[D.err.length-1][0]:0,D.est.length?D.est[D.est.length-1][0]:0,1);
 const md=$("xmode").value;if(md=="last")return[Math.max(0,tEnd-+$("xlast").value),tEnd];
 if(md=="range")return[+$("xmin").value,+$("xmax").value];return[0,tEnd]}
function plot(id,pts,opt){const [g,w,h]=sizeCanvas($(id));const L=46,R=10,T=8,B=24,pw=w-L-R,ph=h-T-B;
 g.fillStyle="#0b0d11";g.fillRect(0,0,w,h);const [x0,x1]=xRange();
 const vis=pts.filter(p=>p[0]>=x0&&p[0]<=x1);let y0=opt.ymin,y1=opt.ymax;
 if(y1==null||isNaN(y1))y1=Math.max(...vis.map(p=>p[1]),opt.thr||0,opt.yfloor||1e-3)*1.1;
 const log=opt.log;const ly=v=>Math.log10(Math.max(v,1e-3));
 const Y=v=>T+ph-(log?(ly(v)-ly(Math.max(y0,0.01)))/(ly(y1)-ly(Math.max(y0,0.01))):(v-y0)/(y1-y0))*ph;
 const X=t=>L+(t-x0)/(x1-x0||1)*pw;
 g.strokeStyle="#262b35";g.fillStyle=css("--muted");g.font="11px system-ui";g.lineWidth=1;
 const yt=log?[0.01,0.1,1,10,100,1000].filter(v=>v>=Math.max(y0,0.01)&&v<=y1):niceTicks(y0,y1,4);
 for(const v of yt){g.beginPath();g.moveTo(L,Y(v));g.lineTo(w-R,Y(v));g.stroke();g.fillText(opt.fmt?opt.fmt(v):v,4,Y(v)+4)}
 for(const v of niceTicks(x0,x1,6)){g.fillText(v+" s",X(v)-10,h-6)}
 if(opt.thr){g.strokeStyle=css("--bad");g.setLineDash([5,4]);g.beginPath();g.moveTo(L,Y(opt.thr));g.lineTo(w-R,Y(opt.thr));g.stroke();g.setLineDash([])}
 g.save();g.beginPath();g.rect(L,T,pw,ph);g.clip();
 if(opt.band){g.strokeStyle="rgba(255,255,255,.28)";g.lineWidth=1.5;g.beginPath();   // GPS error bars
  for(const p of vis)if(p[2]){const lo=Math.max(p[1]-opt.band*p[2],log?0.01:0);g.moveTo(X(p[0]),Y(lo));g.lineTo(X(p[0]),Y(p[1]+opt.band*p[2]))}g.stroke()}
 g.strokeStyle=opt.col;g.lineWidth=2;g.beginPath();let on=false;
 for(let i=0;i<vis.length;i++){const p=vis[i],gap=i>0&&p[0]-vis[i-1][0]>opt.gap;
  if(!on||gap){g.moveTo(X(p[0]),Y(p[1]));on=true}else g.lineTo(X(p[0]),Y(p[1]))}g.stroke();
 if(opt.thr){g.fillStyle=css("--bad");for(const p of vis)if(p[1]>opt.thr){g.fillRect(X(p[0])-1.5,Y(p[1])-1.5,3,3)}}
 g.restore();}
function stats(){const e=D.err.map(p=>p[1]);if(!e.length)return;const s=[...e].sort((a,b)=>a-b),thr=+$("thr").value;
 $("cur").textContent=e[e.length-1].toFixed(2)+" m";$("max").textContent=s[s.length-1].toFixed(2)+" m";
 $("med").textContent=s[Math.floor(s.length/2)].toFixed(2)+" m";
 const over=e.filter(v=>v>thr).length/e.length;$("over").textContent=(100*over).toFixed(1)+" %";
 $("cur").style.color=e[e.length-1]>thr?css("--bad"):css("--ok");}
function draw(){stats();extraTiles();const names={1:"localized",2:"warning",3:"error",5:"not localized"};
 $("st").textContent=status.code==null?"no status":(names[status.code]||("code "+status.code));
 $("st").style.color=status.code==1?css("--ok"):status.code==5?css("--warn"):css("--bad");
 const m=/(\d+) \/ (\d+) inliers/.exec(status.note||"");$("inl").textContent=m?`${m[1]} / ${m[2]}`:"–";
 $("refk").textContent=mode=="gt"?"GT age":"GPS age";$("refname").textContent=mode=="gt"?"ground truth":"GPS";
 $("age").textContent=refAge==null?"–":refAge.toFixed(1)+" s";$("age").style.color=refAge!=null&&refAge>3?css("--bad"):"";
 $("note").textContent=status.note||"";
 const ym=$("ymax").value;
 plot("err",D.err,{col:css("--est"),ymin:0,ymax:ym===""?null:+ym,thr:+$("thr").value,log:$("ylog").checked,yfloor:1,gap:5,band:+$("sig").value,fmt:v=>v+" m"});
 plot("rate",D.inl.map(p=>[p[0],p[2]?p[1]/p[2]:0]),{col:css("--inl"),ymin:0,ymax:null,yfloor:0.25,gap:30,fmt:v=>(100*v).toFixed(0)+"%"});
 drawMap();}
for(const id of["xmode","xlast","xmin","xmax","ymax","ylog","thr","trail","follow","sig"])$(id).addEventListener("input",draw);
$("priors").addEventListener("input",()=>{priorLegend();draw()});
addEventListener("resize",draw);
fetch("meta").then(r=>r.json()).then(m=>{META=m;if(m.has_image){IMG=new Image();IMG.onload=draw;IMG.src="aerial.jpg"}
 fetch("priors").then(r=>r.json()).then(p=>{PRIORS=p;priorLegend();draw()});poll()});
</script></body></html>"""


def make_handler(viewer, jpeg, meta, priors):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def send(self, code, body, ctype):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            u = urlparse(self.path)
            if u.path in ("/", "/index.html"):
                self.send(200, PAGE.encode(), "text/html; charset=utf-8")
            elif u.path == "/meta":
                self.send(200, json.dumps(meta).encode(), "application/json")
            elif u.path == "/priors":
                self.send(200, json.dumps(priors).encode(), "application/json")
            elif u.path == "/aerial.jpg" and jpeg:
                self.send(200, jpeg, "image/jpeg")
            elif u.path == "/data":
                q = {k: int(v[0]) for k, v in parse_qs(u.query).items()}
                self.send(
                    200, json.dumps(viewer.snapshot(q)).encode(), "application/json"
                )
            else:
                self.send(404, b"not found", "text/plain")

    return H


def main():
    env_origin = os.environ.get("ADT4_ORIGIN_UTM", "482937.0,4421340.0").split(",")
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawTextHelpFormatter
    )
    ap.add_argument("robot")
    ap.add_argument(
        "--origin",
        nargs=2,
        type=float,
        default=[float(v) for v in env_origin[:2]],
        help="UTM easting northing of the map frame origin (default $ADT4_ORIGIN_UTM)",
    )
    ap.add_argument(
        "--utm-zone",
        type=int,
        default=None,
        help="force a UTM zone (default: from lon)",
    )
    ap.add_argument("--map-frame", default="map")
    ap.add_argument("--base-frame", default=None, help="default <robot>/base_link")
    ap.add_argument(
        "--odom-frame",
        default=None,
        help="default <robot>/odom (for distance traveled)",
    )
    ap.add_argument("--gps-topic", default=None, help="default /<robot>/fix")
    ap.add_argument(
        "--min-fix-status",
        type=int,
        default=0,
        help="ignore fixes below this NavSatStatus (0 fix, 1 SBAS, 2 RTK/GBAS)",
    )
    ap.add_argument(
        "--gt-csv", default=None, help="score against a UTM GT CSV instead of GPS"
    )
    ap.add_argument("--status-node", default="meridian_rpgo")
    ap.add_argument(
        "--prior-map",
        default=os.environ.get("ADT4_PRIOR_MAP") or None,
        help="fused prior map dir, to draw the prior sessions' trajectories "
        "(default $ADT4_PRIOR_MAP)",
    )
    ap.add_argument(
        "--ortho",
        default=DEFAULT_ORTHO,
        help=f"north-up GeoTIFF in the same UTM zone (default {DEFAULT_ORTHO})",
    )
    ap.add_argument(
        "--ortho-px", type=int, default=4000, help="max side of the served image"
    )
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--sim-time", action="store_true", help="use /clock (bag replay)")
    ap.add_argument(
        "--log-csv", default=None, help="also append every sample to this CSV"
    )
    a = ap.parse_args(rclpy.utilities.remove_ros_args()[1:])

    jpeg, extent = None, None
    if a.ortho and os.path.exists(a.ortho):
        t = time.time()
        jpeg, extent = load_ortho(a.ortho, a.origin, a.ortho_px)
        print(
            f"ortho ready ({len(jpeg) / 1e6:.1f} MB, {time.time() - t:.0f} s)",
            flush=True,
        )
    else:
        print(
            f"no ortho at {a.ortho}: map will have no background (pass --ortho)",
            flush=True,
        )
    priors = load_priors(a.prior_map) if a.prior_map else []
    if a.prior_map:
        print(f"prior trajectories: {len(priors)} from {a.prior_map}", flush=True)
    meta = dict(
        has_image=jpeg is not None,
        extent=extent,
        origin=a.origin,
        robot=a.robot,
        num_priors=len(priors),
    )

    rclpy.init()
    viewer = Viewer(a)
    srv = ThreadingHTTPServer(
        ("0.0.0.0", a.port), make_handler(viewer, jpeg, meta, priors)
    )
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    print(f"open http://localhost:{a.port}", flush=True)
    try:
        rclpy.spin(viewer)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    srv.shutdown()
    viewer.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
