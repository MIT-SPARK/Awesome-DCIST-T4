#!/usr/bin/env python3
"""
Doodle Labs Mesh Rider — live mesh topology visualizer.

Runs a local web server that queries all radios from topology.yaml and
displays an interactive force-directed graph in the browser.

Usage:
    ./doodlelabs_mesh_viz.py                  # default port 8780
    ./doodlelabs_mesh_viz.py --port 9000
    ./doodlelabs_mesh_viz.py --no-open        # don't auto-open browser
    ./doodlelabs_mesh_viz.py --refresh 10     # poll interval in seconds
"""

import argparse
import json
import os
import ssl
import sys
import threading
import time
import urllib.request
import webbrowser
from http.server import HTTPServer, BaseHTTPRequestHandler
from pathlib import Path

import platform
import socket
import subprocess

import yaml

# ---------------------------------------------------------------------------
# Local machine info
# ---------------------------------------------------------------------------

def _get_local_machine_info():
    """Detect this machine's identity and Doodle Labs interface."""
    info = {
        "name": "_this_machine",
        "is_local": True,
        "reachable": True,
        "hostname": socket.gethostname(),
        "model": f"{platform.system()} {platform.machine()}",
        "ip": "",
    }
    # Find the interface with a 192.168.153.x address
    try:
        result = subprocess.run(
            ["ip", "-o", "addr", "show"], capture_output=True, text=True)
        for line in result.stdout.strip().split("\n"):
            if "192.168.153." in line:
                parts = line.split()
                info["dl_interface"] = parts[1]
                # Extract our IP from the addr field (e.g. 192.168.153.100/24)
                for p in parts:
                    if p.startswith("192.168.153."):
                        info["ip"] = p.split("/")[0]
                        break
                break
    except Exception:
        pass
    # Uptime
    try:
        with open("/proc/uptime") as f:
            info["uptime"] = int(float(f.read().split()[0]))
    except Exception:
        pass
    # Memory
    try:
        with open("/proc/meminfo") as f:
            meminfo = {}
            for line in f:
                k, v = line.split(":")
                meminfo[k.strip()] = int(v.strip().split()[0]) * 1024
            info["mem_total"] = meminfo.get("MemTotal", 0)
            info["mem_free"] = (meminfo.get("MemFree", 0)
                                + meminfo.get("Buffers", 0)
                                + meminfo.get("Cached", 0))
    except Exception:
        pass
    return info


# ---------------------------------------------------------------------------
# Radio API
# ---------------------------------------------------------------------------

_ssl_ctx = ssl.create_default_context()
_ssl_ctx.check_hostname = False
_ssl_ctx.verify_mode = ssl.CERT_NONE

NULL_TOKEN = "00000000000000000000000000000000"


def _rpc(ip, token, obj, method, args=None):
    payload = json.dumps({
        "jsonrpc": "2.0", "id": 1, "method": "call",
        "params": [token, obj, method, args or {}],
    }).encode()
    req = urllib.request.Request(
        f"https://{ip}/ubus", data=payload,
        headers={"Content-Type": "application/json"},
    )
    with urllib.request.urlopen(req, timeout=5, context=_ssl_ctx) as resp:
        data = json.loads(resp.read())
    result = data.get("result", [])
    if len(result) >= 2:
        return result[0], result[1]
    if len(result) == 1:
        return result[0], None
    return -1, None


def _login(ip, user, password):
    code, data = _rpc(ip, NULL_TOKEN, "session", "login",
                      {"username": user, "password": password})
    if code == 0 and data:
        return data.get("ubus_rpc_session", "")
    return ""


def query_radio(ip, user, password):
    """Query a single radio. Returns a dict with all available info."""
    info = {"ip": ip, "reachable": False}
    try:
        token = _login(ip, user, password)
        if not token:
            info["error"] = "auth failed"
            return info
        info["reachable"] = True

        code, board = _rpc(ip, token, "system", "board", {})
        if code == 0 and board:
            info["hostname"] = board.get("hostname", "")
            info["model"] = board.get("model", "")
            rel = board.get("release", {})
            info["firmware"] = rel.get("description", rel.get("version", ""))

        code, sysinfo = _rpc(ip, token, "system", "info", {})
        if code == 0 and sysinfo:
            info["uptime"] = sysinfo.get("uptime", 0)
            mem = sysinfo.get("memory", {})
            info["mem_total"] = mem.get("total", 0)
            info["mem_free"] = (mem.get("free", 0) + mem.get("buffered", 0)
                                + mem.get("cached", 0))

        code, iw = _rpc(ip, token, "iwinfo", "info", {"device": "wlan0"})
        if code == 0 and iw:
            info["bssid"] = iw.get("bssid", "")
            info["ssid"] = iw.get("ssid", "")
            info["mode"] = iw.get("mode", "")
            info["frequency"] = iw.get("frequency")
            info["channel"] = iw.get("channel")
            info["htmode"] = iw.get("htmode", "")
            info["txpower"] = iw.get("txpower")
            info["signal"] = iw.get("signal")
            info["noise"] = iw.get("noise")
            info["bitrate"] = iw.get("bitrate")

        code, data = _rpc(ip, token, "file", "read",
                          {"path": "/tmp/linkstate_current.json", "base64": False})
        if code == 0 and data and data.get("data"):
            try:
                ls = json.loads(data["data"])
                info["sta_stats"] = ls.get("sta_stats", [])
                info["mesh_stats"] = ls.get("mesh_stats", [])
                info["activity"] = ls.get("activity")
            except json.JSONDecodeError:
                pass

        code, data = _rpc(ip, token, "file", "exec",
                          {"command": "cat",
                           "params": ["/tmp/run/pancake.txt"]})
        if code == 0 and data and data.get("stdout"):
            try:
                pancake = json.loads(data["stdout"])
                temp = pancake.get("Temperature", pancake.get("temperature"))
                vin = pancake.get("VIN VOLTAGE", pancake.get("vin_voltage"))
                if temp is not None:
                    info["temperature"] = float(temp)
                if vin is not None:
                    info["battery_v"] = float(vin) / 20.2
            except (json.JSONDecodeError, ValueError, TypeError):
                pass

        for field in ("latitude", "longitude", "altitude"):
            code, data = _rpc(ip, token, "file", "read",
                              {"path": f"/var/run/gps/{field}"})
            if code == 0 and data and data.get("data"):
                val = data["data"].strip()
                if val and val != "0":
                    info.setdefault("gps", {})[field] = val

    except Exception as e:
        info["error"] = str(e)
    return info


# ---------------------------------------------------------------------------
# Topology poller
# ---------------------------------------------------------------------------

class TopologyState:
    def __init__(self, topology_path):
        self.topology_path = topology_path
        self.lock = threading.Lock()
        self.data = {"nodes": [], "edges": [], "ts": 0}

    def poll(self):
        with open(self.topology_path) as f:
            cfg = yaml.safe_load(f)
        radios_cfg = cfg.get("doodlelabs_radios", {}).get("radios", {})

        nodes = []
        edges = []
        bssid_to_name = {}  # bssid -> config name

        # Add local machine node
        local = _get_local_machine_info()
        nodes.append(local)
        local_connected_radio = None  # first radio at 192.168.153.x

        for name, rcfg in sorted(radios_cfg.items()):
            ip = rcfg.get("ip", "")
            if not ip:
                continue
            user = rcfg.get("user", "user")
            password = rcfg.get("password", "DoodleSmartRadio")

            info = query_radio(ip, user, password)
            info["name"] = name

            bssid = info.get("bssid", "").lower()
            if bssid:
                bssid_to_name[bssid] = name

            nodes.append(info)

            # The radio at 192.168.153.1 is the one physically connected
            # to this machine via USB-ethernet
            if ip.startswith("192.168.153.") and info["reachable"]:
                if local_connected_radio is None:
                    local_connected_radio = name

        # Ethernet link from this machine to the directly-connected radio
        if local_connected_radio:
            edges.append({
                "from": "_this_machine",
                "to": local_connected_radio,
                "type": "ethernet",
            })

        # Build edges from sta_stats and mesh_stats
        seen_edges = set()
        for node in nodes:
            if node.get("is_local"):
                continue
            src = node["name"]
            for peer in node.get("sta_stats", []):
                mac = peer.get("mac", "").lower()
                dst = bssid_to_name.get(mac, mac)
                key = tuple(sorted([src, dst]))
                if key not in seen_edges:
                    seen_edges.add(key)
                    edges.append({
                        "from": src,
                        "to": dst,
                        "rssi": peer.get("rssi"),
                        "rssi_ant": peer.get("rssi_ant", []),
                        "mcs": peer.get("mcs"),
                        "pl_ratio": peer.get("pl_ratio", 0),
                        "type": "direct",
                    })

            for mesh_node in node.get("mesh_stats", []):
                mac = mesh_node.get("orig_address", "").lower()
                dst = bssid_to_name.get(mac, mac)
                hop = mesh_node.get("hop_status", "")
                key = tuple(sorted([src, dst]))
                if key not in seen_edges:
                    seen_edges.add(key)
                    edges.append({
                        "from": src,
                        "to": dst,
                        "tq": mesh_node.get("tq"),
                        "hop_status": hop,
                        "last_seen_ms": mesh_node.get("last_seen_msecs"),
                        "type": "mesh",
                    })

        with self.lock:
            self.data = {
                "nodes": nodes,
                "edges": edges,
                "ts": time.time(),
            }

    def get(self):
        with self.lock:
            return self.data


def poller_loop(state, interval):
    while True:
        try:
            state.poll()
        except Exception as e:
            print(f"[poller] error: {e}", file=sys.stderr)
        time.sleep(interval)


# ---------------------------------------------------------------------------
# HTML page
# ---------------------------------------------------------------------------

HTML_PAGE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>Doodle Labs Mesh Visualizer</title>
<style>
  * { margin: 0; padding: 0; box-sizing: border-box; }
  body {
    font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, monospace;
    background: #0d1117; color: #c9d1d9;
  }
  #header {
    padding: 12px 20px; background: #161b22; border-bottom: 1px solid #30363d;
    display: flex; align-items: center; justify-content: space-between;
  }
  #header h1 { font-size: 16px; font-weight: 600; color: #58a6ff; }
  #header .status { font-size: 12px; color: #8b949e; }
  #header .status .dot {
    display: inline-block; width: 8px; height: 8px; border-radius: 50%;
    margin-right: 4px; vertical-align: middle;
  }
  #header .controls { display: flex; align-items: center; gap: 10px; }
  #header .controls button {
    background: #21262d; border: 1px solid #30363d; color: #c9d1d9;
    padding: 4px 10px; border-radius: 4px; cursor: pointer; font-size: 12px;
  }
  #header .controls button:hover { background: #30363d; }
  #header .controls button.active { background: #238636; border-color: #238636; }
  #header .controls select {
    background: #21262d; border: 1px solid #30363d; color: #c9d1d9;
    padding: 3px 6px; border-radius: 4px; font-size: 12px;
  }
  #header .status .dot.live { background: #3fb950; }
  #header .status .dot.stale { background: #d29922; }
  #main { display: flex; height: calc(100vh - 49px); }
  #graph { flex: 1; position: relative; }
  #graph canvas { display: block; }
  #sidebar {
    width: 320px; background: #161b22; border-left: 1px solid #30363d;
    overflow-y: auto; padding: 16px; font-size: 13px;
  }
  #sidebar h2 { font-size: 14px; color: #58a6ff; margin-bottom: 12px; }
  #sidebar .empty { color: #484f58; font-style: italic; }
  .node-card {
    background: #0d1117; border: 1px solid #30363d; border-radius: 6px;
    padding: 10px 12px; margin-bottom: 10px;
  }
  .node-card.selected { border-color: #58a6ff; }
  .node-card .name { font-weight: 600; color: #f0f6fc; margin-bottom: 4px; }
  .node-card .field { color: #8b949e; margin-bottom: 2px; }
  .node-card .field span { color: #c9d1d9; }
  .badge {
    display: inline-block; padding: 1px 6px; border-radius: 10px;
    font-size: 11px; font-weight: 600;
  }
  .badge.green { background: #238636; color: #fff; }
  .badge.yellow { background: #9e6a03; color: #fff; }
  .badge.red { background: #da3633; color: #fff; }
  .badge.gray { background: #30363d; color: #8b949e; }
  .edge-info {
    background: #0d1117; border: 1px solid #30363d; border-radius: 6px;
    padding: 10px 12px; margin-bottom: 10px;
  }
  .edge-info .label { font-weight: 600; color: #f0f6fc; margin-bottom: 4px; }
  #legend {
    position: absolute; bottom: 12px; left: 12px; background: #161b22e0;
    border: 1px solid #30363d; border-radius: 6px; padding: 10px 14px;
    font-size: 11px; color: #8b949e;
  }
  #legend .row { display: flex; align-items: center; margin-bottom: 3px; }
  #legend .swatch {
    width: 24px; height: 4px; border-radius: 2px; margin-right: 8px;
  }
</style>
</head>
<body>
<div id="header">
  <h1>Doodle Labs Mesh Topology</h1>
  <div class="controls">
    <button id="btnRefresh" onclick="manualRefresh()" title="Refresh now">Refresh</button>
    <button id="btnPause" onclick="togglePause()" title="Pause/resume auto-refresh">Auto</button>
    <select id="selInterval" onchange="changeInterval(this.value)" title="Auto-refresh interval">
      <option value="3">3s</option>
      <option value="5" selected>5s</option>
      <option value="10">10s</option>
      <option value="30">30s</option>
      <option value="60">60s</option>
    </select>
    <div class="status">
      <span class="dot live" id="statusDot"></span>
      <span id="statusText">connecting...</span>
    </div>
  </div>
</div>
<div id="main">
  <div id="graph"><canvas id="canvas"></canvas>
    <div id="legend">
      <div class="row"><div class="swatch" style="background:#3fb950"></div>RSSI &gt; -60 dBm (excellent)</div>
      <div class="row"><div class="swatch" style="background:#d29922"></div>RSSI -60 to -75 dBm (good)</div>
      <div class="row"><div class="swatch" style="background:#da3633"></div>RSSI &lt; -75 dBm (weak)</div>
      <div class="row"><div class="swatch" style="background:#30363d; border: 1px dashed #484f58"></div>Mesh-only (no direct link)</div>
      <div class="row"><div class="swatch" style="background:#8b949e"></div>Ethernet (USB-to-radio)</div>
      <div style="margin-top:6px; font-size:10px">
        <span style="display:inline-block;width:12px;height:12px;border-radius:50%;background:#0d1117;border:1.5px solid #58a6ff;vertical-align:middle;margin-right:4px"></span>Radio
        <span style="display:inline-block;width:12px;height:12px;border-radius:2px;background:#0d1117;border:1.5px solid #da7;vertical-align:middle;margin-left:8px;margin-right:4px"></span>This machine
      </div>
    </div>
  </div>
  <div id="sidebar">
    <h2>Nodes</h2>
    <div id="nodeList"><div class="empty">waiting for data...</div></div>
    <h2 style="margin-top:16px">Links</h2>
    <div id="edgeList"><div class="empty">waiting for data...</div></div>
  </div>
</div>

<script>
// ---------------------------------------------------------------------------
// Graph engine — simple force-directed layout on canvas
// ---------------------------------------------------------------------------
const canvas = document.getElementById('canvas');
const ctx = canvas.getContext('2d');

let W, H;
function resize() {
  const r = canvas.parentElement.getBoundingClientRect();
  W = canvas.width = r.width * devicePixelRatio;
  H = canvas.height = r.height * devicePixelRatio;
  canvas.style.width = r.width + 'px';
  canvas.style.height = r.height + 'px';
  ctx.setTransform(devicePixelRatio, 0, 0, devicePixelRatio, 0, 0);
}
resize();
window.addEventListener('resize', resize);

let nodes = [];  // {id, x, y, vx, vy, info}
let edges = [];  // {source, target, data}
let selected = null;  // node id
let dragging = null;
let mouse = {x: 0, y: 0};

const NODE_RADIUS = 28;
const SPRING_LEN = 180;
const SPRING_K = 0.004;
const REPULSION = 8000;
const DAMPING = 0.85;
const CENTER_PULL = 0.001;

function rssiColor(rssi) {
  if (rssi == null) return '#30363d';
  if (rssi > -60) return '#3fb950';
  if (rssi > -75) return '#d29922';
  return '#da3633';
}

function tqColor(tq) {
  if (tq == null) return '#30363d';
  if (tq > 200) return '#3fb950';
  if (tq > 128) return '#d29922';
  return '#da3633';
}

function batteryBadge(v) {
  if (v == null) return '';
  if (v > 7.84) return 'green';
  if (v > 7.39) return 'yellow';
  return 'red';
}

function nodeColor(info) {
  if (info.is_local) return '#ddaa77';
  if (!info.reachable) return '#484f58';
  if (info.battery_v != null && info.battery_v < 6.5) return '#da3633';
  return '#58a6ff';
}

function tick() {
  const cw = W / devicePixelRatio, ch = H / devicePixelRatio;
  // Repulsion
  for (let i = 0; i < nodes.length; i++) {
    for (let j = i + 1; j < nodes.length; j++) {
      let dx = nodes[j].x - nodes[i].x;
      let dy = nodes[j].y - nodes[i].y;
      let d2 = dx * dx + dy * dy;
      if (d2 < 1) d2 = 1;
      let f = REPULSION / d2;
      let fx = f * dx / Math.sqrt(d2);
      let fy = f * dy / Math.sqrt(d2);
      nodes[i].vx -= fx; nodes[i].vy -= fy;
      nodes[j].vx += fx; nodes[j].vy += fy;
    }
  }
  // Springs
  for (const e of edges) {
    const a = nodes.find(n => n.id === e.source);
    const b = nodes.find(n => n.id === e.target);
    if (!a || !b) continue;
    let dx = b.x - a.x, dy = b.y - a.y;
    let dist = Math.sqrt(dx * dx + dy * dy) || 1;
    let f = SPRING_K * (dist - SPRING_LEN);
    let fx = f * dx / dist, fy = f * dy / dist;
    a.vx += fx; a.vy += fy;
    b.vx -= fx; b.vy -= fy;
  }
  // Center pull + integrate
  for (const n of nodes) {
    if (n.id === dragging) { n.vx = 0; n.vy = 0; continue; }
    n.vx += (cw / 2 - n.x) * CENTER_PULL;
    n.vy += (ch / 2 - n.y) * CENTER_PULL;
    n.vx *= DAMPING; n.vy *= DAMPING;
    n.x += n.vx; n.y += n.vy;
    n.x = Math.max(NODE_RADIUS, Math.min(cw - NODE_RADIUS, n.x));
    n.y = Math.max(NODE_RADIUS, Math.min(ch - NODE_RADIUS, n.y));
  }
}

function draw() {
  const cw = W / devicePixelRatio, ch = H / devicePixelRatio;
  ctx.clearRect(0, 0, cw, ch);

  // Edges
  for (const e of edges) {
    const a = nodes.find(n => n.id === e.source);
    const b = nodes.find(n => n.id === e.target);
    if (!a || !b) continue;

    let color, width, dash;
    if (e.data.type === 'ethernet') {
      color = '#8b949e';
      width = 2;
      dash = [];
    } else if (e.data.type === 'direct') {
      color = rssiColor(e.data.rssi);
      width = 2.5;
      dash = [];
    } else {
      color = tqColor(e.data.tq);
      width = 1.5;
      dash = [6, 4];
    }

    ctx.beginPath();
    ctx.setLineDash(dash);
    ctx.strokeStyle = color;
    ctx.lineWidth = width;
    ctx.moveTo(a.x, a.y);
    ctx.lineTo(b.x, b.y);
    ctx.stroke();
    ctx.setLineDash([]);

    // Edge label
    let label = '';
    if (e.data.type === 'ethernet') {
      label = 'USB-ethernet';
    } else if (e.data.type === 'direct' && e.data.rssi != null) {
      label = e.data.rssi + ' dBm';
      if (e.data.mcs != null) label += ' / MCS ' + e.data.mcs;
    } else if (e.data.tq != null) {
      label = 'TQ ' + Math.round(e.data.tq * 100 / 255) + '%';
      if (e.data.hop_status) label += ' / ' + e.data.hop_status;
    }
    if (label) {
      const mx = (a.x + b.x) / 2, my = (a.y + b.y) / 2;
      ctx.font = '10px monospace';
      ctx.fillStyle = '#8b949e';
      ctx.textAlign = 'center';
      ctx.fillText(label, mx, my - 6);
    }
  }

  // Nodes
  for (const n of nodes) {
    const isSel = n.id === selected;
    const color = nodeColor(n.info);
    const isLocal = n.info.is_local;

    if (isLocal) {
      // Rounded rectangle for this machine
      const rw = 72, rh = 44, cr = 8;
      const rx = n.x - rw/2, ry = n.y - rh/2;

      if (isSel) {
        ctx.beginPath();
        ctx.roundRect(rx - 4, ry - 4, rw + 8, rh + 8, cr + 2);
        ctx.fillStyle = color + '30';
        ctx.fill();
      }

      ctx.beginPath();
      ctx.roundRect(rx, ry, rw, rh, cr);
      ctx.fillStyle = '#0d1117';
      ctx.fill();
      ctx.strokeStyle = isSel ? color : (color + '90');
      ctx.lineWidth = isSel ? 2.5 : 1.5;
      ctx.stroke();

      ctx.font = 'bold 11px monospace';
      ctx.fillStyle = '#f0f6fc';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(n.info.hostname || 'this machine', n.x, n.y - 5);
      ctx.font = '9px monospace';
      ctx.fillStyle = '#8b949e';
      ctx.fillText(n.info.ip || 'local', n.x, n.y + 8);
    } else {
      // Circle for radios
      if (isSel) {
        ctx.beginPath();
        ctx.arc(n.x, n.y, NODE_RADIUS + 6, 0, Math.PI * 2);
        ctx.fillStyle = color + '30';
        ctx.fill();
      }

      ctx.beginPath();
      ctx.arc(n.x, n.y, NODE_RADIUS, 0, Math.PI * 2);
      ctx.fillStyle = '#0d1117';
      ctx.fill();
      ctx.strokeStyle = isSel ? color : (color + '90');
      ctx.lineWidth = isSel ? 2.5 : 1.5;
      ctx.stroke();

      ctx.font = 'bold 11px monospace';
      ctx.fillStyle = '#f0f6fc';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      const label = n.info.hostname
        ? n.info.hostname.replace('smartradio-', 'sr-')
        : n.id;
      ctx.fillText(label, n.x, n.y - 4);

      ctx.font = '9px monospace';
      ctx.fillStyle = '#8b949e';
      let sub = '';
      if (n.info.battery_v != null) sub = n.info.battery_v.toFixed(1) + 'V';
      else if (n.info.signal != null) sub = n.info.signal + ' dBm';
      else if (!n.info.reachable) sub = 'offline';
      ctx.fillText(sub, n.x, n.y + 8);
    }
  }

  tick();
  requestAnimationFrame(draw);
}

// ---------------------------------------------------------------------------
// Interaction
// ---------------------------------------------------------------------------

function nodeAt(x, y) {
  for (const n of nodes) {
    if (n.info.is_local) {
      // Rectangle hit-test
      if (Math.abs(x - n.x) < 40 && Math.abs(y - n.y) < 26) return n;
    } else {
      const dx = n.x - x, dy = n.y - y;
      if (dx * dx + dy * dy < NODE_RADIUS * NODE_RADIUS) return n;
    }
  }
  return null;
}

canvas.addEventListener('mousedown', e => {
  const rect = canvas.getBoundingClientRect();
  const x = e.clientX - rect.left, y = e.clientY - rect.top;
  const hit = nodeAt(x, y);
  if (hit) { dragging = hit.id; selected = hit.id; }
  else { selected = null; }
  updateSidebar();
});

canvas.addEventListener('mousemove', e => {
  const rect = canvas.getBoundingClientRect();
  mouse.x = e.clientX - rect.left;
  mouse.y = e.clientY - rect.top;
  if (dragging) {
    const n = nodes.find(n => n.id === dragging);
    if (n) { n.x = mouse.x; n.y = mouse.y; }
  }
  canvas.style.cursor = nodeAt(mouse.x, mouse.y) ? 'grab' : 'default';
});

canvas.addEventListener('mouseup', () => { dragging = null; });

// ---------------------------------------------------------------------------
// Sidebar
// ---------------------------------------------------------------------------

function uptimeStr(s) {
  if (!s) return '?';
  const d = Math.floor(s / 86400), h = Math.floor((s % 86400) / 3600),
        m = Math.floor((s % 3600) / 60);
  let parts = [];
  if (d) parts.push(d + 'd');
  if (h) parts.push(h + 'h');
  parts.push(m + 'm');
  return parts.join(' ');
}

function updateSidebar() {
  // Node cards
  let nodeHtml = '';
  for (const n of nodes) {
    const info = n.info;
    const sel = n.id === selected ? ' selected' : '';
    const hn = info.hostname || n.id;
    nodeHtml += `<div class="node-card${sel}" onclick="selected='${n.id}';updateSidebar()">`;
    nodeHtml += `<div class="name">${hn}`;
    if (info.is_local) nodeHtml += ` <span class="badge" style="background:#6e5c2e;color:#fff">this machine</span>`;
    else if (!info.reachable) nodeHtml += ` <span class="badge gray">offline</span>`;
    nodeHtml += `</div>`;
    if (info.ip) nodeHtml += `<div class="field">IP: <span>${info.ip}</span></div>`;
    if (info.dl_interface) nodeHtml += `<div class="field">Interface: <span>${info.dl_interface}</span></div>`;
    if (info.model) nodeHtml += `<div class="field">Model: <span>${info.model}</span></div>`;
    if (info.firmware) nodeHtml += `<div class="field">FW: <span>${info.firmware}</span></div>`;
    if (info.uptime) nodeHtml += `<div class="field">Uptime: <span>${uptimeStr(info.uptime)}</span></div>`;
    if (info.frequency) {
      let rf = info.frequency + 'MHz';
      if (info.htmode) rf += ' / ' + info.htmode;
      if (info.channel) rf += ' / ch' + info.channel;
      nodeHtml += `<div class="field">Radio: <span>${rf}</span></div>`;
    }
    if (info.signal != null) {
      let link = 'Signal: ' + info.signal + 'dBm';
      if (info.noise != null) link += ' / Noise: ' + info.noise + 'dBm';
      if (info.signal != null && info.noise != null && info.noise !== 0)
        link += ' / SNR: ' + (info.signal - info.noise) + 'dB';
      nodeHtml += `<div class="field">Link: <span>${link}</span></div>`;
    }
    if (info.bitrate) nodeHtml += `<div class="field">Rate: <span>${(info.bitrate/1000).toFixed(1)} Mbps</span></div>`;
    if (info.battery_v != null) {
      const bc = batteryBadge(info.battery_v);
      nodeHtml += `<div class="field">Battery: <span class="badge ${bc}">${info.battery_v.toFixed(2)}V</span></div>`;
    }
    if (info.temperature != null)
      nodeHtml += `<div class="field">Temp: <span>${info.temperature}°C</span></div>`;
    if (info.mem_total) {
      const pct = Math.round(100 * (1 - info.mem_free / info.mem_total));
      nodeHtml += `<div class="field">Memory: <span>${pct}% used</span></div>`;
    }
    if (info.gps) {
      const g = info.gps;
      let gps = (g.latitude || '?') + ', ' + (g.longitude || '?');
      if (g.altitude) gps += ' alt ' + g.altitude + 'm';
      nodeHtml += `<div class="field">GPS: <span>${gps}</span></div>`;
    }
    if (info.error) nodeHtml += `<div class="field" style="color:#da3633">${info.error}</div>`;
    nodeHtml += '</div>';
  }
  document.getElementById('nodeList').innerHTML = nodeHtml || '<div class="empty">no nodes</div>';

  // Edge cards
  let edgeHtml = '';
  for (const e of edges) {
    edgeHtml += '<div class="edge-info">';
    edgeHtml += `<div class="label">${e.source} ↔ ${e.target}</div>`;
    if (e.data.type === 'ethernet') {
      edgeHtml += `<div class="field">Type: <span>USB-ethernet</span></div>`;
    } else if (e.data.type === 'direct') {
      if (e.data.rssi != null) {
        const c = e.data.rssi > -60 ? 'green' : e.data.rssi > -75 ? 'yellow' : 'red';
        edgeHtml += `<div class="field">RSSI: <span class="badge ${c}">${e.data.rssi} dBm</span></div>`;
      }
      if (e.data.rssi_ant && e.data.rssi_ant.length)
        edgeHtml += `<div class="field">Antennas: <span>${e.data.rssi_ant.join(' / ')} dBm</span></div>`;
      if (e.data.mcs != null) edgeHtml += `<div class="field">MCS: <span>${e.data.mcs}</span></div>`;
      if (e.data.pl_ratio) edgeHtml += `<div class="field">Packet loss: <span>${e.data.pl_ratio}%</span></div>`;
    } else {
      if (e.data.tq != null) {
        const pct = Math.round(e.data.tq * 100 / 255);
        const c = e.data.tq > 200 ? 'green' : e.data.tq > 128 ? 'yellow' : 'red';
        edgeHtml += `<div class="field">TQ: <span class="badge ${c}">${pct}% (${e.data.tq}/255)</span></div>`;
      }
      if (e.data.hop_status) edgeHtml += `<div class="field">Hop: <span>${e.data.hop_status}</span></div>`;
      if (e.data.last_seen_ms != null) edgeHtml += `<div class="field">Last seen: <span>${(e.data.last_seen_ms/1000).toFixed(1)}s ago</span></div>`;
    }
    edgeHtml += '</div>';
  }
  document.getElementById('edgeList').innerHTML = edgeHtml || '<div class="empty">no links</div>';
}

// ---------------------------------------------------------------------------
// Refresh controls
// ---------------------------------------------------------------------------

let refreshInterval = REFRESH_INTERVAL;
let autoRefresh = true;
let refreshTimer = null;
let fetching = false;

function togglePause() {
  autoRefresh = !autoRefresh;
  const btn = document.getElementById('btnPause');
  btn.textContent = autoRefresh ? 'Auto' : 'Paused';
  btn.classList.toggle('active', autoRefresh);
  scheduleRefresh();
}

function changeInterval(val) {
  refreshInterval = parseInt(val);
  scheduleRefresh();
}

function manualRefresh() {
  fetchData();
}

function scheduleRefresh() {
  if (refreshTimer) clearInterval(refreshTimer);
  if (autoRefresh) {
    refreshTimer = setInterval(fetchData, refreshInterval * 1000);
  }
}

// Set initial dropdown to match server-side default
document.getElementById('selInterval').value = String(refreshInterval);
document.getElementById('btnPause').classList.add('active');

// ---------------------------------------------------------------------------
// Data fetcher
// ---------------------------------------------------------------------------

async function fetchData() {
  if (fetching) return;
  fetching = true;
  document.getElementById('btnRefresh').textContent = '...';
  try {
    const resp = await fetch('/api/topology');
    const data = await resp.json();

    const cw = W / devicePixelRatio, ch = H / devicePixelRatio;

    // Merge nodes — keep positions for existing ones
    const oldMap = {};
    for (const n of nodes) oldMap[n.id] = n;

    const newNodes = [];
    for (const info of data.nodes) {
      const id = info.name;
      if (oldMap[id]) {
        oldMap[id].info = info;
        newNodes.push(oldMap[id]);
      } else {
        const angle = newNodes.length * 2.4;
        const r = 60 + newNodes.length * 30;
        newNodes.push({
          id, info,
          x: cw / 2 + r * Math.cos(angle),
          y: ch / 2 + r * Math.sin(angle),
          vx: 0, vy: 0,
        });
      }
    }
    nodes = newNodes;

    edges = data.edges.map(e => ({
      source: e.from, target: e.to, data: e,
    }));

    const age = (Date.now() / 1000 - data.ts);
    const dot = document.getElementById('statusDot');
    const txt = document.getElementById('statusText');
    if (age < refreshInterval * 2) {
      dot.className = 'dot live';
      txt.textContent = `live · ${nodes.length} node(s) · ${edges.length} link(s)`;
    } else {
      dot.className = 'dot stale';
      txt.textContent = `stale (${Math.round(age)}s ago)`;
    }

    updateSidebar();
  } catch (e) {
    document.getElementById('statusDot').className = 'dot stale';
    document.getElementById('statusText').textContent = 'fetch error: ' + e;
  } finally {
    fetching = false;
    document.getElementById('btnRefresh').textContent = 'Refresh';
  }
}

fetchData();
scheduleRefresh();
requestAnimationFrame(draw);
</script>
</body>
</html>"""


# ---------------------------------------------------------------------------
# HTTP server
# ---------------------------------------------------------------------------

class Handler(BaseHTTPRequestHandler):
    state = None  # set in main

    def log_message(self, fmt, *args):
        pass  # quiet

    def do_GET(self):
        if self.path == '/api/topology':
            data = self.state.get()
            body = json.dumps(data).encode()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', len(body))
            self.end_headers()
            self.wfile.write(body)
        elif self.path in ('/', '/index.html'):
            body = HTML_PAGE.replace(
                'REFRESH_INTERVAL', str(self.refresh_interval),
            ).encode()
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.send_header('Content-Length', len(body))
            self.end_headers()
            self.wfile.write(body)
        else:
            self.send_error(404)


def main():
    parser = argparse.ArgumentParser(
        description='Doodle Labs mesh topology visualizer')
    parser.add_argument('--port', type=int, default=8780)
    parser.add_argument('--refresh', type=int, default=5,
                        help='poll interval in seconds')
    parser.add_argument('--no-open', action='store_true',
                        help="don't auto-open browser")
    args = parser.parse_args()

    script_dir = Path(__file__).resolve().parent
    topology = script_dir.parent / 'config' / 'topology.yaml'
    if not topology.exists():
        print(f"topology.yaml not found at {topology}", file=sys.stderr)
        sys.exit(1)

    state = TopologyState(str(topology))

    # Initial poll before serving
    print(f"Querying radios from {topology.name}...")
    state.poll()
    data = state.get()
    print(f"  Found {len(data['nodes'])} node(s), {len(data['edges'])} link(s)")

    # Start background poller
    t = threading.Thread(target=poller_loop, args=(state, args.refresh),
                         daemon=True)
    t.start()

    # Start HTTP server
    Handler.state = state
    Handler.refresh_interval = args.refresh
    server = HTTPServer(('127.0.0.1', args.port), Handler)

    url = f'http://localhost:{args.port}'
    print(f"Serving at {url}  (refresh every {args.refresh}s)")
    print("Press Ctrl+C to stop.")

    if not args.no_open:
        webbrowser.open(url)

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == '__main__':
    main()
