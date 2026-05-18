#!/usr/bin/env bash
# doodlelabs_status.sh — Doodle Labs Mesh Rider radio status via JSON-RPC (ubus) API
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TOPOLOGY="${SCRIPT_DIR}/../config/topology.yaml"

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; CYAN='\033[0;36m'; NC='\033[0m'; BOLD='\033[1m'; DIM='\033[2m'

# ---------------------------------------------------------------------------
# Topology helpers
# ---------------------------------------------------------------------------

get_doodlelabs_interface() {
    python3 -c "
import subprocess
result = subprocess.run(['ip', '-o', 'addr', 'show'], capture_output=True, text=True)
for line in result.stdout.strip().split('\n'):
    if '192.168.153.' in line:
        print(line.split()[1])
        break
"
}

get_radio_entries() {
    python3 -c "
import yaml
with open('${TOPOLOGY}') as f:
    data = yaml.safe_load(f)
radios = data.get('doodlelabs_radios', {}).get('radios', {})
for name, info in sorted(radios.items()):
    ip = info.get('ip', '') or ''
    if ip:
        user = info.get('user', 'user')
        password = info.get('password', 'DoodleSmartRadio')
        print(f'{name}|{ip}|{user}|{password}')
"
}

# ---------------------------------------------------------------------------
# Commands
# ---------------------------------------------------------------------------

cmd_status() {
    echo -e "${BOLD}=== Doodle Labs Radio Status ===${NC}"
    echo ""

    # Check for local interface
    local iface
    iface=$(get_doodlelabs_interface) || true
    if [[ -n "$iface" ]]; then
        echo -e "Doodle Labs interface: ${CYAN}${iface}${NC}"
    else
        echo -e "${YELLOW}No local Doodle Labs interface detected (no 192.168.153.x address).${NC}"
    fi
    echo ""

    echo -e "${BOLD}Radio status (JSON-RPC API):${NC}"
    echo ""

    while IFS='|' read -r name ip user password; do
        [[ -z "$name" ]] && continue
        [[ -z "$ip" ]] && continue

        echo -ne "  Querying ${name} (${ip})... "

        local api_result
        api_result=$(DL_IP="$ip" DL_USER="$user" DL_PASS="$password" python3 << 'PYEOF'
import urllib.request, json, ssl, time, sys, os

ctx = ssl.create_default_context()
ctx.check_hostname = False
ctx.verify_mode = ssl.CERT_NONE

ip = os.environ["DL_IP"]
user = os.environ["DL_USER"]
password = os.environ["DL_PASS"]

base = f"https://{ip}/ubus"
null_token = "00000000000000000000000000000000"

def rpc(token, obj, method, args=None):
    if args is None:
        args = {}
    payload = json.dumps({
        "jsonrpc": "2.0", "id": 1, "method": "call",
        "params": [token, obj, method, args]
    }).encode()
    req = urllib.request.Request(base, data=payload,
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=5, context=ctx) as resp:
        data = json.loads(resp.read())
    result = data.get("result", [])
    if len(result) >= 2:
        return result[0], result[1]
    elif len(result) == 1:
        return result[0], None
    return -1, None

try:
    start = time.monotonic()

    code, login_data = rpc(null_token, "session", "login",
                           {"username": user, "password": password})
    if code != 0 or not login_data:
        print(json.dumps({"error": f"login failed (code={code})"}))
        sys.exit()
    token = login_data.get("ubus_rpc_session", "")
    if not token:
        print(json.dumps({"error": "no session token returned"}))
        sys.exit()

    out = {}

    code, data = rpc(token, "system", "board", {})
    if code == 0 and data:
        out["board"] = data

    code, data = rpc(token, "file", "read",
                     {"path": "/tmp/linkstate_current.json", "base64": False})
    if code == 0 and data and data.get("data"):
        try:
            out["linkstate"] = json.loads(data["data"])
        except json.JSONDecodeError:
            pass

    code, data = rpc(token, "file", "exec",
                     {"command": "cat", "params": ["/tmp/run/pancake.txt"]})
    if code == 0 and data and data.get("stdout"):
        try:
            out["pancake"] = json.loads(data["stdout"])
        except json.JSONDecodeError:
            pass

    gps = {}
    for field in ["latitude", "longitude", "altitude"]:
        code, data = rpc(token, "file", "read",
                         {"path": f"/var/run/gps/{field}"})
        if code == 0 and data and data.get("data"):
            gps[field] = data["data"].strip()
    if gps:
        out["gps"] = gps

    code, data = rpc(token, "system", "info", {})
    if code == 0 and data:
        out["sysinfo"] = data

    code, data = rpc(token, "iwinfo", "info", {"device": "wlan0"})
    if code == 0 and data:
        out["iwinfo"] = data

    elapsed_ms = (time.monotonic() - start) * 1000
    out["_response_ms"] = elapsed_ms
    print(json.dumps(out))

except Exception as e:
    print(json.dumps({"error": str(e)}))
PYEOF
)

        # Display results — pass JSON via env var to avoid stdin/heredoc conflict
        DL_JSON="$api_result" python3 << 'PYEOF'
import json, sys, os

raw = os.environ.get("DL_JSON", "").strip()
if not raw:
    print('\033[0;31mno response\033[0m\n')
    sys.exit()

try:
    data = json.loads(raw)
except json.JSONDecodeError:
    print(f'\033[0;31mparse error\033[0m\n')
    sys.exit()

if 'error' in data:
    print(f'\033[0;31mFAILED\033[0m: {data["error"]}\n')
    sys.exit()

resp_ms = data.get('_response_ms', 0)
print(f'\033[0;32mOK\033[0m ({resp_ms:.0f}ms)')

# Board info
board = data.get('board', {})
hostname = board.get('hostname', '?')
model = board.get('model', '?')
release = board.get('release', {})
firmware = release.get('description', release.get('version', '?'))
print(f'    Hostname: {hostname}')
print(f'    Model:    {model}')
print(f'    Firmware: {firmware}')

# System info (uptime, memory)
sysinfo = data.get('sysinfo', {})
if sysinfo:
    uptime_s = sysinfo.get('uptime', 0)
    days = uptime_s // 86400
    hours = (uptime_s % 86400) // 3600
    mins = (uptime_s % 3600) // 60
    parts = []
    if days: parts.append(f'{days}d')
    if hours: parts.append(f'{hours}h')
    parts.append(f'{mins}m')
    print(f'    Uptime:   {" ".join(parts)}')

    mem = sysinfo.get('memory', {})
    total = mem.get('total', 0)
    free = mem.get('free', 0) + mem.get('buffered', 0) + mem.get('cached', 0)
    if total > 0:
        used_pct = 100 * (1 - free / total)
        print(f'    Memory:   {used_pct:.0f}% used ({free // 1024 // 1024}MB free / {total // 1024 // 1024}MB)')

# Pancake (wearable: battery voltage + temperature)
pancake = data.get('pancake')
if pancake:
    temp = pancake.get('Temperature', pancake.get('temperature'))
    vin = pancake.get('VIN VOLTAGE', pancake.get('vin_voltage'))
    parts = []
    if temp is not None:
        temp_val = float(temp)
        if temp_val > 70:
            tc = '\033[0;31m'
        elif temp_val > 55:
            tc = '\033[1;33m'
        else:
            tc = '\033[0;32m'
        parts.append(f'Temp: {tc}{temp_val:.0f}C\033[0m')
    if vin is not None:
        volts = float(vin) / 20.2
        if volts > 7.84:
            vc = '\033[0;32m'
        elif volts > 7.39:
            vc = '\033[1;33m'
        elif volts > 6.5:
            vc = '\033[0;31m'
        else:
            vc = '\033[0;31m\033[5m'
        parts.append(f'Battery: {vc}{volts:.2f}V\033[0m')
    if parts:
        print(f'    Wearable: {" | ".join(parts)}')

# Wireless info (prefer iwinfo for RF params — linkstate chan_width/noise can be 0)
iwinfo = data.get('iwinfo', {})
ls = data.get('linkstate', {})
if iwinfo:
    freq = iwinfo.get('frequency')
    channel = iwinfo.get('channel')
    txpower = iwinfo.get('txpower')
    htmode = iwinfo.get('htmode', '')
    noise = iwinfo.get('noise')
    signal = iwinfo.get('signal')
    bitrate = iwinfo.get('bitrate')
    mode = iwinfo.get('mode')

    parts = []
    if freq: parts.append(f'{freq}MHz')
    if channel: parts.append(f'ch{channel}')
    if htmode: parts.append(htmode)
    if txpower: parts.append(f'{txpower}dBm TX')
    if mode: parts.append(mode)
    if parts:
        print(f'    Radio:    {" / ".join(parts)}')

    rf_parts = []
    if signal is not None: rf_parts.append(f'Signal: {signal}dBm')
    if noise is not None: rf_parts.append(f'Noise: {noise}dBm')
    if signal is not None and noise is not None and noise != 0:
        snr = signal - noise
        rf_parts.append(f'SNR: {snr}dB')
    if bitrate: rf_parts.append(f'Rate: {bitrate/1000:.1f}Mbps')
    activity = ls.get('activity') if ls else None
    if activity is not None: rf_parts.append(f'Activity: {activity}%')
    if rf_parts:
        print(f'    Link:     {" / ".join(rf_parts)}')

    # Connected peers (from linkstate)
    sta_stats = ls.get('sta_stats', [])
    if sta_stats:
        print(f'    Peers ({len(sta_stats)}):')
        for peer in sta_stats:
            mac = peer.get('mac', '?')
            rssi = peer.get('rssi', '?')
            rssi_ant = peer.get('rssi_ant', [])
            pl = peer.get('pl_ratio', 0)
            mcs = peer.get('mcs', '?')
            rssi_val = float(rssi) if rssi != '?' else -999
            if rssi_val > -60:
                rc = '\033[0;32m'
            elif rssi_val > -75:
                rc = '\033[1;33m'
            else:
                rc = '\033[0;31m'
            ant_str = f' (ant: {"/".join(str(a) for a in rssi_ant)})' if rssi_ant else ''
            pl_str = f'  PL: {pl}%' if pl and float(pl) > 0 else ''
            print(f'      {mac}  RSSI: {rc}{rssi}dBm{ant_str}\033[0m  MCS: {mcs}{pl_str}')

    # Mesh nodes
    mesh_stats = ls.get('mesh_stats', [])
    if mesh_stats:
        print(f'    Mesh ({len(mesh_stats)} nodes):')
        for node in mesh_stats:
            orig = node.get('orig_address', '?')
            tq = node.get('tq', '?')
            hop = node.get('hop_status', '?')
            seen = node.get('last_seen_msecs', '?')
            tq_val = int(tq) if tq != '?' else 0
            tq_pct = tq_val * 100 // 255 if tq_val else 0
            if tq_val > 200:
                tc = '\033[0;32m'
            elif tq_val > 128:
                tc = '\033[1;33m'
            else:
                tc = '\033[0;31m'
            print(f'      {orig}  TQ: {tc}{tq_pct}%\033[0m ({tq}/255)  {hop}  seen {seen}ms ago')

# GPS
gps = data.get('gps', {})
if gps:
    lat = gps.get('latitude', '')
    lon = gps.get('longitude', '')
    alt = gps.get('altitude', '')
    if lat and lon and lat != '0' and lon != '0':
        parts = [f'{lat}, {lon}']
        if alt:
            parts.append(f'alt {alt}m')
        print(f'    GPS:      {" / ".join(parts)}')

print()
PYEOF

    done < <(get_radio_entries)
}

cmd_topology() {
    echo -e "${BOLD}=== Doodle Labs Mesh Topology ===${NC}"
    echo ""

    DL_TOPOLOGY="$TOPOLOGY" python3 << 'PYEOF'
import yaml, urllib.request, json, ssl, sys, os

ctx = ssl.create_default_context()
ctx.check_hostname = False
ctx.verify_mode = ssl.CERT_NONE

with open(os.environ["DL_TOPOLOGY"]) as f:
    data = yaml.safe_load(f)

radios = data.get('doodlelabs_radios', {}).get('radios', {})
null_token = "00000000000000000000000000000000"

def rpc(ip, token, obj, method, args=None):
    if args is None:
        args = {}
    payload = json.dumps({
        "jsonrpc": "2.0", "id": 1, "method": "call",
        "params": [token, obj, method, args]
    }).encode()
    req = urllib.request.Request(f"https://{ip}/ubus", data=payload,
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=5, context=ctx) as resp:
        result = json.loads(resp.read())
    r = result.get("result", [])
    if len(r) >= 2:
        return r[0], r[1]
    return -1, None

def login(ip, user, password):
    code, data = rpc(ip, null_token, "session", "login",
                     {"username": user, "password": password})
    if code == 0 and data:
        return data.get("ubus_rpc_session", "")
    return ""

all_nodes = {}

for name, info in sorted(radios.items()):
    ip = info.get('ip', '')
    if not ip:
        continue
    user = info.get('user', 'user')
    password = info.get('password', 'DoodleSmartRadio')

    try:
        token = login(ip, user, password)
        if not token:
            print(f'  \033[1;33m{name}: auth failed\033[0m')
            continue

        code, data = rpc(ip, token, "file", "read",
                         {"path": "/tmp/linkstate_current.json", "base64": False})
        if code != 0 or not data or not data.get("data"):
            print(f'  \033[1;33m{name}: no linkstate\033[0m')
            continue

        ls = json.loads(data["data"])

        code, board = rpc(ip, token, "system", "board", {})
        hostname = board.get("hostname", name) if code == 0 and board else name

        mesh_stats = ls.get("mesh_stats", [])
        sta_stats = ls.get("sta_stats", [])

        peers = []
        for peer in sta_stats:
            peers.append({
                "mac": peer.get("mac", "?"),
                "rssi": peer.get("rssi", "?"),
            })

        all_nodes[name] = {
            "hostname": hostname,
            "ip": ip,
            "mesh": mesh_stats,
            "peers": peers,
            "noise": ls.get("noise"),
            "freq": ls.get("oper_freq"),
        }

    except Exception as e:
        print(f'  \033[1;33m{name}: {e}\033[0m')

if not all_nodes:
    print("  No radios reachable.")
    sys.exit()

print("Reachable radios:")
for name, info in sorted(all_nodes.items()):
    freq = info.get("freq", "?")
    noise = info.get("noise", "?")
    print(f'  {name} ({info["hostname"]}) at {info["ip"]}  freq={freq}MHz noise={noise}dBm')
print()

print("Direct peers (L1 links):")
for name, info in sorted(all_nodes.items()):
    peers = info.get("peers", [])
    if peers:
        for p in peers:
            print(f'  {name} <-> {p["mac"]}  RSSI: {p["rssi"]}dBm')
    else:
        print(f'  {name}: no direct peers')
print()

print("Mesh topology (batman-adv):")
for name, info in sorted(all_nodes.items()):
    mesh = info.get("mesh", [])
    if mesh:
        print(f'  From {name}:')
        for node in mesh:
            orig = node.get("orig_address", "?")
            tq = node.get("tq", "?")
            hop = node.get("hop_status", "?")
            tq_val = int(tq) if tq != '?' else 0
            tq_pct = tq_val * 100 // 255 if tq_val else 0
            print(f'    {orig}  TQ={tq_pct}% ({tq}/255)  {hop}')
    else:
        print(f'  From {name}: no mesh nodes')
PYEOF
}

cmd_linkstate() {
    echo -e "${BOLD}=== Doodle Labs Live Link State ===${NC}"
    echo ""

    local interval="${1:-5}"
    echo -e "Polling every ${interval}s. Press Ctrl+C to stop."
    echo ""

    trap 'echo -e "\n${NC}Stopped."; exit 0' INT

    while true; do
        local ts
        ts=$(date '+%H:%M:%S')

        while IFS='|' read -r name ip user password; do
            [[ -z "$name" ]] && continue
            [[ -z "$ip" ]] && continue

            DL_IP="$ip" DL_USER="$user" DL_PASS="$password" DL_NAME="$name" DL_TS="$ts" python3 << 'PYEOF'
import urllib.request, json, ssl, sys, os

ctx = ssl.create_default_context()
ctx.check_hostname = False
ctx.verify_mode = ssl.CERT_NONE

ip = os.environ["DL_IP"]
user = os.environ["DL_USER"]
password = os.environ["DL_PASS"]
name = os.environ["DL_NAME"]
ts = os.environ["DL_TS"]

base = f"https://{ip}/ubus"
null_token = "00000000000000000000000000000000"

def rpc(token, obj, method, args=None):
    if args is None:
        args = {}
    payload = json.dumps({
        "jsonrpc": "2.0", "id": 1, "method": "call",
        "params": [token, obj, method, args]
    }).encode()
    req = urllib.request.Request(base, data=payload,
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=5, context=ctx) as resp:
        data = json.loads(resp.read())
    result = data.get("result", [])
    if len(result) >= 2:
        return result[0], result[1]
    return -1, None

try:
    code, login_data = rpc(null_token, "session", "login",
                           {"username": user, "password": password})
    if code != 0 or not login_data:
        print(f'[{ts}] {name}: \033[0;31mauth failed\033[0m')
        sys.exit()
    token = login_data["ubus_rpc_session"]

    code, data = rpc(token, "file", "read",
                     {"path": "/tmp/linkstate_current.json", "base64": False})
    if code != 0 or not data or not data.get("data"):
        print(f'[{ts}] {name}: \033[0;31mno linkstate\033[0m')
        sys.exit()

    ls = json.loads(data["data"])
    noise = ls.get("noise", "?")
    activity = ls.get("activity", "?")

    peers = ls.get("sta_stats", [])
    peer_parts = []
    for p in peers:
        mac = p.get("mac", "?")[-8:]
        rssi = p.get("rssi", "?")
        mcs = p.get("mcs", "?")
        peer_parts.append(f'{mac}:{rssi}dBm/MCS{mcs}')

    mesh = ls.get("mesh_stats", [])

    print(f'[{ts}] {name}: noise={noise}dBm activity={activity}% '
          f'peers=[{", ".join(peer_parts)}] mesh_nodes={len(mesh)}')

except Exception as e:
    print(f'[{ts}] {name}: \033[0;31m{e}\033[0m')
PYEOF

        done < <(get_radio_entries)

        sleep "$interval"
    done
}

cmd_battery_watch() {
    local threshold="${1:-7.0}"
    local interval="${2:-30}"
    echo -e "${BOLD}=== Doodle Labs Battery Watch ===${NC} (threshold: ${threshold}V, interval: ${interval}s)"
    echo -e "Voltage thresholds: >7.84V ${GREEN}green${NC}, 7.39-7.84V ${YELLOW}yellow${NC}, 6.5-7.39V ${RED}red${NC}"
    echo -e "Press Ctrl+C to stop."
    echo ""

    trap 'echo -e "\n${NC}Stopped."; exit 0' INT

    while true; do
        local ts
        ts=$(date '+%H:%M:%S')

        while IFS='|' read -r name ip user password; do
            [[ -z "$name" ]] && continue
            [[ -z "$ip" ]] && continue

            DL_IP="$ip" DL_USER="$user" DL_PASS="$password" DL_NAME="$name" DL_TS="$ts" DL_THRESHOLD="$threshold" python3 << 'PYEOF'
import urllib.request, json, ssl, sys, os

ctx = ssl.create_default_context()
ctx.check_hostname = False
ctx.verify_mode = ssl.CERT_NONE

ip = os.environ["DL_IP"]
user = os.environ["DL_USER"]
password = os.environ["DL_PASS"]
name = os.environ["DL_NAME"]
ts = os.environ["DL_TS"]
threshold = float(os.environ["DL_THRESHOLD"])

base = f"https://{ip}/ubus"
null_token = "00000000000000000000000000000000"

def rpc(token, obj, method, args=None):
    if args is None:
        args = {}
    payload = json.dumps({
        "jsonrpc": "2.0", "id": 1, "method": "call",
        "params": [token, obj, method, args]
    }).encode()
    req = urllib.request.Request(base, data=payload,
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=5, context=ctx) as resp:
        data = json.loads(resp.read())
    result = data.get("result", [])
    if len(result) >= 2:
        return result[0], result[1]
    return -1, None

try:
    code, login_data = rpc(null_token, "session", "login",
                           {"username": user, "password": password})
    if code != 0 or not login_data:
        print(f'[{ts}] {name}: \033[0;31munreachable\033[0m')
        sys.exit()
    token = login_data["ubus_rpc_session"]

    code, data = rpc(token, "file", "exec",
                     {"command": "cat", "params": ["/tmp/run/pancake.txt"]})
    if code != 0 or not data or not data.get("stdout"):
        print(f'[{ts}] {name}: \033[1;33mno pancake data (not a wearable?)\033[0m')
        sys.exit()

    pancake = json.loads(data["stdout"])
    vin = pancake.get("VIN VOLTAGE", pancake.get("vin_voltage"))
    temp = pancake.get("Temperature", pancake.get("temperature"))

    if vin is not None:
        volts = float(vin) / 20.2
        temp_str = f'  Temp: {temp}C' if temp is not None else ''

        if volts <= threshold:
            print(f'[{ts}] \033[0;31m\033[1mALERT\033[0m {name}: battery \033[0;31m{volts:.2f}V\033[0m{temp_str} \a')
        elif volts <= 7.39:
            print(f'[{ts}] {name}: battery \033[0;31m{volts:.2f}V\033[0m{temp_str}')
        elif volts <= 7.84:
            print(f'[{ts}] {name}: battery \033[1;33m{volts:.2f}V\033[0m{temp_str}')
        else:
            print(f'[{ts}] {name}: battery \033[0;32m{volts:.2f}V\033[0m{temp_str}')
    else:
        print(f'[{ts}] {name}: \033[1;33mno voltage data\033[0m')

except Exception as e:
    print(f'[{ts}] {name}: \033[0;31m{e}\033[0m')
PYEOF

        done < <(get_radio_entries)

        sleep "$interval"
    done
}

cmd_radio_config() {
    echo -e "${BOLD}=== Doodle Labs Radio Configuration ===${NC}"
    echo ""

    while IFS='|' read -r name ip user password; do
        [[ -z "$name" ]] && continue
        [[ -z "$ip" ]] && continue

        echo -ne "  ${name} (${ip}): "

        DL_IP="$ip" DL_USER="$user" DL_PASS="$password" python3 << 'PYEOF'
import urllib.request, json, ssl, sys, os

ctx = ssl.create_default_context()
ctx.check_hostname = False
ctx.verify_mode = ssl.CERT_NONE

ip = os.environ["DL_IP"]
user = os.environ["DL_USER"]
password = os.environ["DL_PASS"]

base = f"https://{ip}/ubus"
null_token = "00000000000000000000000000000000"

def rpc(token, obj, method, args=None):
    if args is None:
        args = {}
    payload = json.dumps({
        "jsonrpc": "2.0", "id": 1, "method": "call",
        "params": [token, obj, method, args]
    }).encode()
    req = urllib.request.Request(base, data=payload,
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=5, context=ctx) as resp:
        data = json.loads(resp.read())
    result = data.get("result", [])
    if len(result) >= 2:
        return result[0], result[1]
    return -1, None

try:
    code, login_data = rpc(null_token, "session", "login",
                           {"username": user, "password": password})
    if code != 0 or not login_data:
        print('\033[0;31mauth failed\033[0m')
        sys.exit()
    token = login_data["ubus_rpc_session"]

    print()

    code, board = rpc(token, "system", "board", {})
    if code == 0 and board:
        print(f'    Hostname:   {board.get("hostname", "?")}')
        print(f'    Model:      {board.get("model", "?")}')
        release = board.get("release", {})
        print(f'    Firmware:   {release.get("description", release.get("version", "?"))}')
        print(f'    Kernel:     {board.get("kernel", "?")}')

    code, info = rpc(token, "system", "info", {})
    if code == 0 and info:
        uptime = info.get("uptime", 0)
        d, h, m = uptime // 86400, (uptime % 86400) // 3600, (uptime % 3600) // 60
        parts = []
        if d: parts.append(f'{d}d')
        if h: parts.append(f'{h}h')
        parts.append(f'{m}m')
        print(f'    Uptime:     {" ".join(parts)}')

    code, iw = rpc(token, "iwinfo", "info", {"device": "wlan0"})
    if code == 0 and iw:
        print(f'    SSID:       {iw.get("ssid", "?")}')
        print(f'    Mode:       {iw.get("mode", "?")}')
        print(f'    Frequency:  {iw.get("frequency", "?")} MHz')
        print(f'    Channel:    {iw.get("channel", "?")}')
        print(f'    TX Power:   {iw.get("txpower", "?")} dBm')
        print(f'    Bitrate:    {iw.get("bitrate", "?")}')
        enc = iw.get("encryption", {})
        if enc:
            print(f'    Encryption: {enc.get("description", enc.get("enabled", "?"))}')
        hwmodes = iw.get("hwmodes", [])
        if hwmodes:
            print(f'    HW Modes:   {", ".join(hwmodes)}')

    code, data = rpc(token, "file", "read",
                     {"path": "/tmp/linkstate_current.json", "base64": False})
    if code == 0 and data and data.get("data"):
        ls = json.loads(data["data"])
        print(f'    Chan Width: {ls.get("chan_width", "?")} MHz')
        print(f'    Noise:      {ls.get("noise", "?")} dBm')
        print(f'    LNA:        {ls.get("lna_status", "?")}')

    code, data = rpc(token, "file", "exec",
                     {"command": "cat", "params": ["/tmp/run/pancake.txt"]})
    if code == 0 and data and data.get("stdout"):
        try:
            pancake = json.loads(data["stdout"])
            if pancake:
                print(f'    Wearable HW:')
                for k, v in pancake.items():
                    print(f'      {k}: {v}')
        except json.JSONDecodeError:
            pass

    print()

except Exception as e:
    print(f'\033[0;31mFAILED: {e}\033[0m')
    print()
PYEOF

    done < <(get_radio_entries)
}

cmd_ssh() {
    local target="${1:-}"
    if [[ -z "$target" ]]; then
        local first
        first=$(get_radio_entries | head -1)
        if [[ -z "$first" ]]; then
            echo -e "${RED}No radios configured in topology.yaml${NC}"
            return 1
        fi
        target=$(echo "$first" | cut -d'|' -f2)
    fi

    echo -e "${BOLD}Connecting to ${target}...${NC}"
    echo -e "${DIM}(default creds: user / DoodleSmartRadio)${NC}"
    ssh -o StrictHostKeyChecking=no -o HostKeyAlgorithms=+ssh-rsa -o PubkeyAcceptedAlgorithms=+ssh-rsa "root@${target}"
}

cmd_discover() {
    local iface
    iface=$(get_doodlelabs_interface)
    if [[ -z "$iface" ]]; then
        echo -e "${RED}Could not find Doodle Labs interface (no 192.168.153.x address).${NC}"
        return 1
    fi

    echo -e "${BOLD}=== Doodle Labs Radio Discovery ===${NC}"
    echo -e "Interface: ${CYAN}${iface}${NC}"
    echo ""

    echo -ne "  192.168.153.1 (config IP): "
    if ping -c 1 -W 2 192.168.153.1 &>/dev/null; then
        local result
        result=$(python3 << 'PYEOF' 2>/dev/null
import urllib.request, json, ssl

ctx = ssl.create_default_context()
ctx.check_hostname = False
ctx.verify_mode = ssl.CERT_NONE

null_token = "00000000000000000000000000000000"
base = "https://192.168.153.1/ubus"

for username in ["user", "root"]:
    try:
        payload = json.dumps({
            "jsonrpc": "2.0", "id": 1, "method": "call",
            "params": [null_token, "session", "login",
                       {"username": username, "password": "DoodleSmartRadio"}]
        }).encode()
        req = urllib.request.Request(base, data=payload,
                                     headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=5, context=ctx) as resp:
            data = json.loads(resp.read())
        result = data.get("result", [])
        if len(result) >= 2 and result[0] == 0:
            token = result[1]["ubus_rpc_session"]
            payload = json.dumps({
                "jsonrpc": "2.0", "id": 2, "method": "call",
                "params": [token, "system", "board", {}]
            }).encode()
            req = urllib.request.Request(base, data=payload,
                                         headers={"Content-Type": "application/json"})
            with urllib.request.urlopen(req, timeout=5, context=ctx) as resp:
                data = json.loads(resp.read())
            result = data.get("result", [])
            if len(result) >= 2 and result[1]:
                board = result[1]
                hostname = board.get("hostname", "?")
                model = board.get("model", "?")
                rel = board.get("release", {}).get("version", "?")
                print(f"FOUND  {hostname} / {model} / fw {rel} (login: {username})")
                break
    except:
        continue
else:
    print("reachable but login failed")
PYEOF
)
        if [[ -n "$result" ]]; then
            echo -e "${GREEN}${result}${NC}"
        else
            echo -e "${YELLOW}reachable but no API response${NC}"
        fi
    else
        echo -e "${RED}unreachable${NC}"
    fi

    echo ""
    echo "Checking ARP table for 10.223.x.x mesh entries..."
    local found=0
    while IFS= read -r line; do
        local mesh_ip
        mesh_ip=$(echo "$line" | awk '{print $1}')
        [[ -z "$mesh_ip" ]] && continue
        echo -ne "  ${mesh_ip}: "
        if ping -c 1 -W 1 "$mesh_ip" &>/dev/null; then
            echo -e "${GREEN}reachable${NC}"
            ((found++))
        else
            echo -e "${DIM}stale ARP entry${NC}"
        fi
    done < <(ip neigh show 2>/dev/null | grep '10.223' | grep -v FAILED)

    if [[ $found -eq 0 ]]; then
        echo -e "${DIM}No mesh IPs found in ARP table.${NC}"
        echo -e "Try: ${CYAN}ping 10.223.255.255${NC} to trigger mesh discovery."
    fi
}

usage() {
    echo "Usage: $(basename "$0") <command>"
    echo ""
    echo "Commands:"
    echo "  status                    Query all radios (firmware, signal, mesh, battery)"
    echo "  topology                  Show mesh routing topology (batman-adv)"
    echo "  linkstate [interval]      Live link state monitor (default: 5s)"
    echo "  discover                  Detect radios on the network"
    echo "  battery-watch [thr] [int] Monitor battery voltage (threshold V, interval sec)"
    echo "  radio-config              Show detailed radio configuration"
    echo "  ssh [ip]                  SSH to a radio (default: first in topology)"
    echo ""
    echo "JSON-RPC API endpoint: https://<ip>/ubus (OpenWrt ubus)"
    echo "Default credentials: user / DoodleSmartRadio"
    echo ""
    echo "Setup:"
    echo "  1. Connect via USB-ethernet (radio appears at 192.168.153.1)"
    echo "  2. Add radio to topology.yaml under doodlelabs_radios.radios"
    echo "  3. Run 'status' to query the radio"
}

case "${1:-}" in
    status)         cmd_status ;;
    topology)       cmd_topology ;;
    linkstate)      cmd_linkstate "${2:-5}" ;;
    discover)       cmd_discover ;;
    battery-watch)  cmd_battery_watch "${2:-7.0}" "${3:-30}" ;;
    radio-config)   cmd_radio_config ;;
    ssh)            cmd_ssh "${2:-}" ;;
    -h|--help)      usage ;;
    "")             cmd_status ;;
    *)              echo "Unknown command: $1"; usage; exit 1 ;;
esac
