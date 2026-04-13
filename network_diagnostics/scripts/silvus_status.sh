#!/usr/bin/env bash
# silvus_status.sh — Silvus StreamCaster radio status via JSON-RPC API + UDP telemetry
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TOPOLOGY="${SCRIPT_DIR}/../config/topology.yaml"

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; CYAN='\033[0;36m'; NC='\033[0m'; BOLD='\033[1m'; DIM='\033[2m'

# ---------------------------------------------------------------------------
# Topology helpers
# ---------------------------------------------------------------------------

get_silvus_interface() {
    python3 -c "
import subprocess
result = subprocess.run(['ip', '-o', 'addr', 'show'], capture_output=True, text=True)
for line in result.stdout.strip().split('\n'):
    if '192.168.100.' in line:
        print(line.split()[1])
        break
"
}

get_radio_mgmt_ips() {
    python3 -c "
import yaml
with open('${TOPOLOGY}') as f:
    data = yaml.safe_load(f)
radios = data.get('silvus_radios', {}).get('radios', {})
for name, info in sorted(radios.items()):
    mgmt = info.get('mgmt_ip', '') or ''
    if mgmt:
        node_id = info.get('node_id', '') or ''
        print(f'{name}|{mgmt}|{node_id}')
"
}

get_mgmt_subnet() {
    python3 -c "
import yaml
with open('${TOPOLOGY}') as f:
    data = yaml.safe_load(f)
print(data.get('silvus_radios', {}).get('mgmt_subnet', '172.20.0.0/16'))
"
}

# ---------------------------------------------------------------------------
# Auto-detection: find which radio is directly connected to this machine
# ---------------------------------------------------------------------------

CACHE_DIR="${HOME}/.cache/silvus_diagnostics"

detect_local_radio() {
    # The Silvus mesh is L2-bridged, so all radios appear as direct neighbors.
    # Network-based detection (TTL, ARP, timing) cannot reliably distinguish
    # the physically-connected radio from remote ones.
    #
    # Strategy: use a cache file keyed by the USB-ethernet adapter MAC address.
    # The adapter MAC is stable and unique per machine, so once we know which
    # radio is connected to which adapter, the mapping persists even if the
    # radio is moved to a different machine (different adapter MAC = re-detect).
    #
    # Cache: ~/.cache/silvus_diagnostics/adapter_<MAC>.radio

    local iface
    iface=$(get_silvus_interface)
    if [[ -z "$iface" ]]; then
        return 1
    fi

    # Get adapter MAC
    local adapter_mac
    adapter_mac=$(ip link show "$iface" 2>/dev/null | grep 'link/ether' | awk '{print $2}' | tr ':' '-')
    if [[ -z "$adapter_mac" ]]; then
        return 1
    fi

    # Check cache
    mkdir -p "$CACHE_DIR" 2>/dev/null || true
    local cache_file="${CACHE_DIR}/adapter_${adapter_mac}.radio"
    if [[ -f "$cache_file" ]]; then
        cat "$cache_file"
        return 0
    fi

    return 1
}

save_local_radio() {
    local node_id="$1"
    local mgmt_ip="$2"

    local iface
    iface=$(get_silvus_interface)
    [[ -z "$iface" ]] && return 1

    local adapter_mac
    adapter_mac=$(ip link show "$iface" 2>/dev/null | grep 'link/ether' | awk '{print $2}' | tr ':' '-')
    [[ -z "$adapter_mac" ]] && return 1

    mkdir -p "$CACHE_DIR" 2>/dev/null || true
    echo "${node_id}|${mgmt_ip}" > "${CACHE_DIR}/adapter_${adapter_mac}.radio"
}

# ---------------------------------------------------------------------------
# JSON-RPC query
# ---------------------------------------------------------------------------

query_radio_api() {
    local mgmt_ip="$1"
    python3 -c "
import urllib.request, json, time

url = 'http://${mgmt_ip}/cgi-bin/streamscape_api'
batch = [
    {'jsonrpc': '2.0', 'method': 'battery_percent', 'params': [], 'id': 1},
    {'jsonrpc': '2.0', 'method': 'input_voltage_monitoring', 'params': [], 'id': 2},
    {'jsonrpc': '2.0', 'method': 'read_current_temperature', 'params': [], 'id': 3},
    {'jsonrpc': '2.0', 'method': 'nodeid', 'params': [], 'id': 4},
    {'jsonrpc': '2.0', 'method': 'uptime', 'params': [], 'id': 5},
    {'jsonrpc': '2.0', 'method': 'model', 'params': [], 'id': 6},
    {'jsonrpc': '2.0', 'method': 'build_tag', 'params': [], 'id': 7},
    {'jsonrpc': '2.0', 'method': 'routing_tree', 'params': [], 'id': 8},
    {'jsonrpc': '2.0', 'method': 'gps_mode', 'params': [], 'id': 9},
    {'jsonrpc': '2.0', 'method': 'gps_coordinates', 'params': [], 'id': 10},
    {'jsonrpc': '2.0', 'method': 'local_address', 'params': [], 'id': 11},
]

method_names = {r['id']: r['method'] for r in batch}

try:
    start = time.monotonic()
    payload = json.dumps(batch).encode()
    req = urllib.request.Request(url, data=payload, headers={'Content-Type': 'application/json'})
    with urllib.request.urlopen(req, timeout=5) as resp:
        results = json.loads(resp.read())
    elapsed_ms = (time.monotonic() - start) * 1000
    out = {'_response_ms': elapsed_ms}
    for r in results:
        rid = r.get('id', 0)
        method = method_names.get(rid, '?')
        if 'error' not in r:
            out[method] = r['result']
        else:
            out[method] = None
    print(json.dumps(out))
except Exception as e:
    print(json.dumps({'error': str(e)}))
" 2>/dev/null
}

# ---------------------------------------------------------------------------
# Commands
# ---------------------------------------------------------------------------

cmd_status() {
    echo -e "${BOLD}=== Silvus Radio Status ===${NC}"
    echo ""

    # Detect local radio
    echo -ne "Detecting local radio... "
    local local_info
    local_info=$(detect_local_radio 2>/dev/null) || true
    local local_nodeid=""
    local local_mgmt=""
    if [[ -n "$local_info" ]]; then
        local_nodeid=$(echo "$local_info" | cut -d'|' -f1)
        local_mgmt=$(echo "$local_info" | cut -d'|' -f2)
        echo -e "${GREEN}found${NC} node ${local_nodeid} at ${local_mgmt}"
    else
        echo -e "${YELLOW}not detected (no Silvus interface or route to 172.20.x.x)${NC}"
    fi
    echo ""

    # Query all known radios
    echo -e "${BOLD}Radio status (JSON-RPC API):${NC}"
    echo ""

    while IFS='|' read -r name mgmt_ip node_id; do
        [[ -z "$name" ]] && continue
        [[ -z "$mgmt_ip" ]] && continue

        echo -ne "  Querying ${name} (${mgmt_ip})... "
        local api_result
        api_result=$(query_radio_api "$mgmt_ip")

        python3 -c "
import json, sys, re

data = json.loads('''${api_result}''')
if 'error' in data:
    print('\033[0;31mFAILED\033[0m: ' + data['error'])
    print()
    sys.exit()

resp_ms = data.get('_response_ms', 0)
local_nodeid = '${local_nodeid}'

battery = data.get('battery_percent')
voltage = data.get('input_voltage_monitoring')
temp = data.get('read_current_temperature')
nodeid = data.get('nodeid')
model = data.get('model')
build = data.get('build_tag')
uptime_raw = data.get('uptime')
routing = data.get('routing_tree')
gps_mode = data.get('gps_mode')
gps_coords = data.get('gps_coordinates')

nid = nodeid[0] if nodeid else '?'

# Check if this is the locally-connected radio
is_local = (nid == local_nodeid) if local_nodeid else (resp_ms < 5)
local_tag = ' \033[0;36m(LOCAL)\033[0m' if is_local else ''

print(f'\033[0;32mOK\033[0m ({resp_ms:.0f}ms){local_tag}')

bat_str = f'{float(battery[0]):.0f}%' if battery else '?'
volt_str = f'{float(voltage[0])/1000:.2f}V' if voltage else '?'
temp_str = f'{temp[0]}°C' if temp else '?'
mdl = model[0] if model else '?'
bld = (build[0].replace('streamscape_', '') if build else '?')

bat_val = float(battery[0]) if battery else -1
if bat_val >= 50:
    bat_color = '\033[0;32m'
elif bat_val >= 20:
    bat_color = '\033[1;33m'
else:
    bat_color = '\033[0;31m'

print(f'    Model:    {mdl} (firmware {bld})')
print(f'    Node ID:  {nid}')
print(f'    Battery:  {bat_color}{bat_str}\033[0m  Voltage: {volt_str}  Temp: {temp_str}')

if uptime_raw:
    up = uptime_raw[0]
    up = re.sub(r'^.*up\s*', '', up)
    up = re.sub(r',\s*\d+ users.*', '', up)
    print(f'    Uptime:   {up.strip()}')

if routing:
    print(f'    Mesh:     {len(routing)} node(s) in routing tree: {routing}')

if gps_mode:
    mode = gps_mode[0]
    if gps_coords and gps_coords != ['0', '0', '0']:
        print(f'    GPS:      {mode} ({gps_coords[0]}, {gps_coords[1]}, {gps_coords[2]})')
    else:
        print(f'    GPS:      {mode}')

print()
" 2>/dev/null || echo -e "${RED}parse error${NC}"

    done < <(get_radio_mgmt_ips)
}

cmd_topology() {
    echo -e "${BOLD}=== Silvus Mesh Topology ===${NC}"
    echo ""

    python3 -c "
import yaml, urllib.request, json

with open('${TOPOLOGY}') as f:
    data = yaml.safe_load(f)

radios = data.get('silvus_radios', {}).get('radios', {})
node_map = {}    # node_id -> radio name
node_mgmt = {}   # node_id -> mgmt_ip

for name, info in radios.items():
    nid = info.get('node_id')
    mgmt = info.get('mgmt_ip', '')
    if nid:
        node_map[int(nid)] = name
        if mgmt:
            node_mgmt[int(nid)] = mgmt

trees = {}
radio_details = {}  # name -> {nodeid, mgmt_ip, local_address}
for name, info in sorted(radios.items()):
    mgmt_ip = info.get('mgmt_ip', '')
    if not mgmt_ip:
        continue
    try:
        url = f'http://{mgmt_ip}/cgi-bin/streamscape_api'
        batch = [
            {'jsonrpc': '2.0', 'method': 'routing_tree', 'params': [], 'id': 1},
            {'jsonrpc': '2.0', 'method': 'nodeid', 'params': [], 'id': 2},
            {'jsonrpc': '2.0', 'method': 'local_address', 'params': [], 'id': 3},
        ]
        payload = json.dumps(batch).encode()
        req = urllib.request.Request(url, data=payload, headers={'Content-Type': 'application/json'})
        with urllib.request.urlopen(req, timeout=5) as resp:
            results = json.loads(resp.read())
        for r in results:
            if r['id'] == 1 and 'error' not in r:
                trees[name] = r['result']
            if r['id'] == 2 and 'error' not in r:
                nid = int(r['result'][0])
                node_map[nid] = name
                radio_details.setdefault(name, {})['nodeid'] = nid
            if r['id'] == 3 and 'error' not in r:
                radio_details.setdefault(name, {})['mgmt_ip'] = r['result'][0]
    except:
        print(f'  \033[1;33m{name}: unreachable\033[0m')

all_nodes = set()
for tree in trees.values():
    all_nodes.update(tree)

print('Nodes in mesh:')
for nid in sorted(all_nodes):
    label = node_map.get(nid, 'unknown')
    mgmt = node_mgmt.get(nid, '?')
    print(f'  Node {nid}  mgmt={mgmt:<18s}  ({label})')

print()
print('Routing trees:')
for rname, tree in sorted(trees.items()):
    parts = []
    for n in tree:
        label = node_map.get(n, str(n))
        mgmt = node_mgmt.get(n, '?')
        parts.append(f'{label} ({mgmt})')
    print(f'  {rname}: {\" -> \".join(parts)}')
" 2>/dev/null
}

cmd_detect() {
    echo -e "${BOLD}=== Detect Local Radio ===${NC}"
    echo ""

    local iface
    iface=$(get_silvus_interface)
    if [[ -z "$iface" ]]; then
        echo -e "${RED}No Silvus interface found (no 192.168.100.x address).${NC}"
        return 1
    fi

    local adapter_mac
    adapter_mac=$(ip link show "$iface" 2>/dev/null | grep 'link/ether' | awk '{print $2}')
    echo -e "Silvus interface: ${CYAN}${iface}${NC} (MAC: ${adapter_mac})"

    # Check cache
    local cached
    cached=$(detect_local_radio 2>/dev/null) || true
    if [[ -n "$cached" ]]; then
        local cached_nid cached_mgmt
        cached_nid=$(echo "$cached" | cut -d'|' -f1)
        cached_mgmt=$(echo "$cached" | cut -d'|' -f2)
        echo -e "Cached mapping: adapter ${adapter_mac} -> node ${GREEN}${cached_nid}${NC} (${cached_mgmt})"
        echo ""
        echo -n "Use cached mapping? [Y/n] "
        read -r answer
        if [[ "$answer" != "n" && "$answer" != "N" ]]; then
            echo -e "${BOLD}Local radio: node ${cached_nid} (${cached_mgmt})${NC}"
            return 0
        fi
    fi

    # Check for mgmt route
    local mgmt_subnet
    mgmt_subnet=$(get_mgmt_subnet)
    if ! ip route show | grep -q "$mgmt_subnet"; then
        echo -e "${YELLOW}No route to ${mgmt_subnet}. Adding...${NC}"
        sudo ip route add "$mgmt_subnet" dev "$iface" 2>/dev/null || {
            echo -e "${RED}Failed. Run: sudo ip route add ${mgmt_subnet} dev ${iface}${NC}"
            return 1
        }
    fi

    # List all reachable radios
    echo ""
    echo -e "The Silvus mesh is L2-bridged — all radios appear as direct neighbors."
    echo -e "Listing reachable radios for manual selection:"
    echo ""

    local radio_list=()
    local idx=0
    while IFS='|' read -r name mgmt_ip node_id; do
        [[ -z "$name" ]] && continue
        [[ -z "$mgmt_ip" ]] && continue

        if ping -c 1 -W 2 "$mgmt_ip" &>/dev/null; then
            ((idx++))
            radio_list+=("${node_id}|${mgmt_ip}|${name}")
            printf "  ${GREEN}%d)${NC} %-20s node=%-10s mgmt=%s\n" "$idx" "$name" "$node_id" "$mgmt_ip"
        else
            printf "  ${RED}-${NC} %-20s node=%-10s mgmt=%s ${RED}(unreachable)${NC}\n" "$name" "$node_id" "$mgmt_ip"
        fi
    done < <(get_radio_mgmt_ips)

    if [[ ${#radio_list[@]} -eq 0 ]]; then
        echo -e "${RED}No radios reachable.${NC}"
        return 1
    fi

    echo ""
    echo -n "Which radio is physically connected to this machine? [1-${#radio_list[@]}] "
    read -r choice

    if [[ "$choice" -ge 1 && "$choice" -le ${#radio_list[@]} ]] 2>/dev/null; then
        local selected="${radio_list[$((choice-1))]}"
        local sel_nid sel_mgmt sel_name
        sel_nid=$(echo "$selected" | cut -d'|' -f1)
        sel_mgmt=$(echo "$selected" | cut -d'|' -f2)
        sel_name=$(echo "$selected" | cut -d'|' -f3)

        save_local_radio "$sel_nid" "$sel_mgmt"
        echo ""
        echo -e "${GREEN}Saved:${NC} adapter ${adapter_mac} -> node ${sel_nid} (${sel_mgmt})"
        echo -e "This mapping is cached at ${DIM}~/.cache/silvus_diagnostics/${NC}"
        echo -e "Run '$(basename "$0") detect' again to re-detect if the radio is swapped."
    else
        echo -e "${RED}Invalid selection.${NC}"
        return 1
    fi
}

cmd_discover() {
    local iface
    iface=$(get_silvus_interface)
    if [[ -z "$iface" ]]; then
        echo -e "${RED}Could not find Silvus network interface.${NC}"
        return 1
    fi

    local mgmt_subnet
    mgmt_subnet=$(get_mgmt_subnet)

    echo -e "${BOLD}=== Silvus Radio Discovery ===${NC}"
    echo -e "Silvus interface: ${CYAN}${iface}${NC}"
    echo -e "Management subnet: ${CYAN}${mgmt_subnet}${NC}"
    echo ""

    if ! ip route show | grep -q "$mgmt_subnet"; then
        echo -e "${YELLOW}Adding route for ${mgmt_subnet} via ${iface}...${NC}"
        sudo ip route add "$mgmt_subnet" dev "$iface" 2>/dev/null || {
            echo -e "${RED}Failed to add route (need sudo). Run manually:${NC}"
            echo "  sudo ip route add ${mgmt_subnet} dev ${iface}"
            return 1
        }
    fi

    echo "Sending broadcast ping to trigger ARP responses..."
    ping -b -c 3 -W 1 172.20.255.255 2>/dev/null || true
    sleep 1

    echo ""
    echo "Checking ARP table for 172.20.x.x entries..."
    local found=0
    while IFS= read -r line; do
        local ip
        ip=$(echo "$line" | awk '{print $1}')
        [[ -z "$ip" ]] && continue
        echo -ne "  ${ip}: "
        local result
        result=$(python3 -c "
import urllib.request, json
url = 'http://${ip}/cgi-bin/streamscape_api'
batch = [
    {'jsonrpc': '2.0', 'method': 'nodeid', 'params': [], 'id': 1},
    {'jsonrpc': '2.0', 'method': 'model', 'params': [], 'id': 2},
]
try:
    payload = json.dumps(batch).encode()
    req = urllib.request.Request(url, data=payload, headers={'Content-Type': 'application/json'})
    with urllib.request.urlopen(req, timeout=3) as resp:
        results = json.loads(resp.read())
    parts = []
    for r in results:
        if 'error' not in r:
            if r['id'] == 1: parts.append(f'NodeID={r[\"result\"][0]}')
            if r['id'] == 2: parts.append(f'Model={r[\"result\"][0]}')
    print(' '.join(parts) if parts else 'HTTP OK')
except:
    print('no API')
" 2>/dev/null)
        echo -e "${GREEN}FOUND${NC} ${result}"
        ((found++))
    done < <(ip neigh show dev "$iface" 2>/dev/null | grep '172.20' | grep -v FAILED)

    if [[ $found -eq 0 ]]; then
        echo -e "${YELLOW}No radios found via ARP. Try scanning manually:${NC}"
        echo "  nmap -sn ${mgmt_subnet} --min-parallelism 100"
    else
        echo ""
        echo -e "${GREEN}Found ${found} radio(s).${NC}"
        echo "Update topology.yaml silvus_radios.radios.*.mgmt_ip with these IPs."
    fi
}

cmd_listen() {
    local port="${1:-5100}"
    echo -e "${BOLD}=== Silvus UDP Telemetry Listener (port ${port}) ===${NC}"
    echo -e "Listening for RSSI, temperature, and voltage reports..."
    echo -e "Configure UDP streaming on each radio's Node Diagnostics page to send to this machine."
    echo -e "Press Ctrl+C to stop."
    echo ""

    python3 -c "
import socket
import struct
import sys
import math
from datetime import datetime

PORT = ${port}

TYPE_NAMES = {
    1:    'END_REPORT',
    2:    'TEMPERATURE',
    3:    'MAX_TEMPERATURE',
    4:    'OVERHEAT_COUNT',
    8:    'START_TEMP_REPORT',
    9:    'TEMP_REVISION',
    4001: 'START_VOLTAGE_REPORT',
    4003: 'VOLTAGE_REVISION',
    4004: 'CURRENT_VOLTAGE',
    4005: 'MIN_VOLTAGE',
    4006: 'MAX_VOLTAGE',
    4007: 'UNDERVOLTAGE_COUNT',
    4008: 'OVERVOLTAGE_COUNT',
    5000: 'RSSI_ANT1',
    5001: 'RSSI_ANT2',
    5002: 'RSSI_ANT3',
    5003: 'RSSI_ANT4',
    5004: 'NOISE_FLOOR',
    5005: 'SYNC_SIGNAL',
    5006: 'SYNC_NOISE',
    5007: 'NODE_ID',
    5008: 'REPORT_SEQ',
    5009: 'START_RSSI_REPORT',
    5010: 'RSSI_REVISION',
}

def parse_tlv(data):
    reports = []
    offset = 0
    current_report = {}
    while offset + 4 <= len(data):
        tlv_type = struct.unpack('!H', data[offset:offset+2])[0]
        tlv_len = struct.unpack('!H', data[offset+2:offset+4])[0]
        offset += 4
        if offset + tlv_len > len(data):
            break
        value_bytes = data[offset:offset+tlv_len]
        value_str = value_bytes.rstrip(b'\x00').decode('ascii', errors='replace')
        offset += tlv_len
        type_name = TYPE_NAMES.get(tlv_type, f'UNKNOWN_{tlv_type}')
        current_report[type_name] = value_str
        if tlv_type == 1:
            reports.append(current_report)
            current_report = {}
    if current_report:
        reports.append(current_report)
    return reports

def format_rssi(report):
    parts = []
    node_id = report.get('NODE_ID', '?')
    parts.append(f'Node {node_id}')
    for ant in ['RSSI_ANT1', 'RSSI_ANT2', 'RSSI_ANT3', 'RSSI_ANT4']:
        if ant in report:
            val = int(report[ant]) / 2.0
            parts.append(f'{ant[-4:]}: {val:.1f} dBm')
    if 'NOISE_FLOOR' in report:
        noise = int(report['NOISE_FLOOR']) / 2.0
        parts.append(f'Noise: {noise:.1f} dBm')
    if 'SYNC_SIGNAL' in report and 'SYNC_NOISE' in report:
        try:
            x = int(report['SYNC_SIGNAL'])
            y = int(report['SYNC_NOISE'])
            z = (y - x) / 51.0
            if z > 0:
                snr_mw = (x - 12 * z) / (64 * z)
                if snr_mw > 0:
                    snr_db = 10 * math.log10(snr_mw)
                    parts.append(f'SNR: {snr_db:.1f} dB')
        except (ValueError, ZeroDivisionError):
            pass
    return ' | '.join(parts)

def format_voltage(report):
    parts = []
    if 'CURRENT_VOLTAGE' in report:
        parts.append(f'Voltage: {report[\"CURRENT_VOLTAGE\"]}V')
    if 'MIN_VOLTAGE' in report:
        parts.append(f'Min: {report[\"MIN_VOLTAGE\"]}V')
    if 'MAX_VOLTAGE' in report:
        parts.append(f'Max: {report[\"MAX_VOLTAGE\"]}V')
    if 'UNDERVOLTAGE_COUNT' in report:
        cnt = report['UNDERVOLTAGE_COUNT']
        if cnt != '0':
            parts.append(f'Undervoltage events: {cnt}')
    return ' | '.join(parts) if parts else str(report)

def format_temp(report):
    parts = []
    if 'TEMPERATURE' in report:
        parts.append(f'Temp: {report[\"TEMPERATURE\"]}C')
    if 'MAX_TEMPERATURE' in report:
        parts.append(f'Max: {report[\"MAX_TEMPERATURE\"]}C')
    if 'OVERHEAT_COUNT' in report:
        cnt = report['OVERHEAT_COUNT']
        if cnt != '0':
            parts.append(f'Overheat events: {cnt}')
    return ' | '.join(parts) if parts else str(report)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(('0.0.0.0', PORT))
print(f'Listening on UDP port {PORT}...')
print()

while True:
    data, addr = sock.recvfrom(1500)
    ts = datetime.now().strftime('%H:%M:%S')
    reports = parse_tlv(data)
    for report in reports:
        if 'START_RSSI_REPORT' in report or 'RSSI_ANT1' in report:
            print(f'[{ts}] RSSI  from {addr[0]:>15s}: {format_rssi(report)}')
        elif 'START_VOLTAGE_REPORT' in report or 'CURRENT_VOLTAGE' in report:
            print(f'[{ts}] VOLT  from {addr[0]:>15s}: {format_voltage(report)}')
        elif 'START_TEMP_REPORT' in report or 'TEMPERATURE' in report:
            print(f'[{ts}] TEMP  from {addr[0]:>15s}: {format_temp(report)}')
        else:
            print(f'[{ts}] DATA  from {addr[0]:>15s}: {report}')
    sys.stdout.flush()
"
}

cmd_rssi_matrix() {
    echo -e "${BOLD}=== RSSI Link Quality Matrix ===${NC}"
    echo ""

    python3 -c "
import yaml, urllib.request, json, sys

with open('${TOPOLOGY}') as f:
    data = yaml.safe_load(f)

radios = data.get('silvus_radios', {}).get('radios', {})
radio_list = []  # [(name, mgmt_ip, node_id)]
for name, info in sorted(radios.items()):
    mgmt = info.get('mgmt_ip', '')
    nid = info.get('node_id', '')
    if mgmt:
        radio_list.append((name, mgmt, str(nid)))

if len(radio_list) < 2:
    print('  Need at least 2 radios in topology.yaml for a matrix.')
    sys.exit()

# Query each radio for per-neighbor RSSI via routing_tree + rssi data
# The StreamScape API gives RSSI per antenna; we collect from each radio's perspective
results = {}  # name -> {nodeid: ..., rssi_per_neighbor: {nid: val}}
for name, mgmt_ip, node_id in radio_list:
    try:
        url = f'http://{mgmt_ip}/cgi-bin/streamscape_api'
        batch = [
            {'jsonrpc': '2.0', 'method': 'nodeid', 'params': [], 'id': 1},
            {'jsonrpc': '2.0', 'method': 'routing_tree', 'params': [], 'id': 2},
            {'jsonrpc': '2.0', 'method': 'neighbor_list', 'params': [], 'id': 3},
            {'jsonrpc': '2.0', 'method': 'rssi', 'params': [], 'id': 4},
            {'jsonrpc': '2.0', 'method': 'link_stats', 'params': [], 'id': 5},
            {'jsonrpc': '2.0', 'method': 'snr_stats', 'params': [], 'id': 6},
        ]
        payload = json.dumps(batch).encode()
        req = urllib.request.Request(url, data=payload, headers={'Content-Type': 'application/json'})
        with urllib.request.urlopen(req, timeout=5) as resp:
            resp_data = json.loads(resp.read())
        parsed = {}
        for r in resp_data:
            rid = r.get('id', 0)
            if 'error' not in r:
                parsed[rid] = r['result']
            else:
                parsed[rid] = None
        results[name] = {
            'nodeid': parsed.get(1, [node_id])[0] if parsed.get(1) else node_id,
            'routing_tree': parsed.get(2, []),
            'neighbor_list': parsed.get(3),
            'rssi': parsed.get(4),
            'link_stats': parsed.get(5),
            'snr_stats': parsed.get(6),
        }
    except Exception as e:
        results[name] = {'error': str(e)}

# Build node_id -> name map
nid_to_name = {}
for name, mgmt_ip, node_id in radio_list:
    nid_to_name[node_id] = name
    if name in results and 'nodeid' in results[name]:
        nid_to_name[str(results[name]['nodeid'])] = name

# Display what we got
names = [r[0] for r in radio_list]
max_name = max(len(n) for n in names)

# Try to build matrix from available data
has_link_stats = any(results.get(n, {}).get('link_stats') for n in names)
has_rssi = any(results.get(n, {}).get('rssi') for n in names)

# Print header
header = ' ' * (max_name + 2)
for n in names:
    header += f'{n:>14s}'
print(header)
print(' ' * (max_name + 2) + '-' * (14 * len(names)))

for src_name in names:
    row = f'{src_name:<{max_name}s}  '
    info = results.get(src_name, {})
    if 'error' in info:
        row += f'\033[0;31m(unreachable)\033[0m'
        print(row)
        continue

    for dst_name in names:
        if src_name == dst_name:
            row += f'         \033[2m ---\033[0m'
            continue

        # Try to extract RSSI from link_stats or other data
        val = None
        link = info.get('link_stats')
        rssi = info.get('rssi')
        snr = info.get('snr_stats')

        # link_stats and rssi may be lists or dicts keyed by neighbor
        # Try multiple formats since the API varies
        dst_nid = None
        for nid, nm in nid_to_name.items():
            if nm == dst_name:
                dst_nid = nid
                break

        if rssi and isinstance(rssi, list) and len(rssi) > 0:
            # rssi might be a flat value (single neighbor case)
            try:
                val = float(rssi[0]) / 2.0 if len(names) == 2 else None
            except (ValueError, TypeError):
                pass

        if link and isinstance(link, (list, dict)):
            # Try to extract per-neighbor from link_stats
            if isinstance(link, dict) and dst_nid and dst_nid in link:
                try:
                    val = float(link[dst_nid])
                except (ValueError, TypeError):
                    pass

        # Check routing tree membership as fallback
        tree = info.get('routing_tree', [])
        in_mesh = dst_nid and int(dst_nid) in [int(x) for x in tree] if dst_nid and tree else False

        if val is not None:
            if val > -60:
                color = '\033[0;32m'  # green
            elif val > -80:
                color = '\033[1;33m'  # yellow
            else:
                color = '\033[0;31m'  # red
            row += f'{color}{val:>12.1f}dB\033[0m'
        elif in_mesh:
            row += f'  \033[0;32m    linked\033[0m'
        else:
            row += f'          \033[2m?\033[0m'

    print(row)

print()

# Print raw data summary for debugging
print('Raw API data per radio:')
for name in names:
    info = results.get(name, {})
    if 'error' in info:
        print(f'  {name}: \033[0;31m{info[\"error\"]}\033[0m')
        continue
    parts = []
    if info.get('routing_tree'):
        tree_names = [nid_to_name.get(str(n), str(n)) for n in info['routing_tree']]
        parts.append(f'mesh=[{\", \".join(tree_names)}]')
    if info.get('rssi') is not None:
        parts.append(f'rssi={info[\"rssi\"]}')
    if info.get('link_stats') is not None:
        parts.append(f'link_stats={info[\"link_stats\"]}')
    if info.get('snr_stats') is not None:
        parts.append(f'snr_stats={info[\"snr_stats\"]}')
    if info.get('neighbor_list') is not None:
        parts.append(f'neighbors={info[\"neighbor_list\"]}')
    print(f'  {name}: {\"  \".join(parts)}')
" 2>/dev/null
}

cmd_battery_watch() {
    local threshold="${1:-20}"
    local interval="${2:-30}"
    echo -e "${BOLD}=== Battery Watch ===${NC} (threshold: ${threshold}%, interval: ${interval}s)"
    echo -e "Press Ctrl+C to stop."
    echo ""

    trap 'echo -e "\n${NC}Stopped."; exit 0' INT

    while true; do
        local ts
        ts=$(date '+%H:%M:%S')

        while IFS='|' read -r name mgmt_ip node_id; do
            [[ -z "$name" ]] && continue
            [[ -z "$mgmt_ip" ]] && continue

            local result
            result=$(python3 -c "
import urllib.request, json
url = 'http://${mgmt_ip}/cgi-bin/streamscape_api'
batch = [
    {'jsonrpc': '2.0', 'method': 'battery_percent', 'params': [], 'id': 1},
    {'jsonrpc': '2.0', 'method': 'input_voltage_monitoring', 'params': [], 'id': 2},
]
try:
    payload = json.dumps(batch).encode()
    req = urllib.request.Request(url, data=payload, headers={'Content-Type': 'application/json'})
    with urllib.request.urlopen(req, timeout=5) as resp:
        results = json.loads(resp.read())
    bat = None; volt = None
    for r in results:
        if r['id'] == 1 and 'error' not in r: bat = r['result'][0]
        if r['id'] == 2 and 'error' not in r: volt = r['result'][0]
    print(f'{bat}|{volt}')
except:
    print('ERR|ERR')
" 2>/dev/null)

            local bat volt
            bat=$(echo "$result" | cut -d'|' -f1)
            volt=$(echo "$result" | cut -d'|' -f2)

            if [[ "$bat" == "ERR" ]]; then
                echo -e "[${ts}] ${name}: ${RED}unreachable${NC}"
                continue
            fi

            local bat_val
            bat_val=$(echo "$bat" | cut -d'.' -f1)
            local volt_str=""
            if [[ "$volt" != "None" && -n "$volt" ]]; then
                volt_str=$(python3 -c "print(f'{float(${volt})/1000:.2f}V')" 2>/dev/null || echo "")
            fi

            if [[ -n "$bat_val" ]] && [[ "$bat_val" -le "$threshold" ]] 2>/dev/null; then
                echo -e "[${ts}] ${RED}${BOLD}ALERT${NC} ${name}: battery ${RED}${bat_val}%${NC} ${volt_str} \a"
            else
                echo -e "[${ts}] ${name}: battery ${GREEN}${bat_val}%${NC} ${volt_str}"
            fi
        done < <(get_radio_mgmt_ips)

        sleep "$interval"
    done
}

cmd_radio_config() {
    echo -e "${BOLD}=== Radio Configuration ===${NC}"
    echo ""

    while IFS='|' read -r name mgmt_ip node_id; do
        [[ -z "$name" ]] && continue
        [[ -z "$mgmt_ip" ]] && continue

        echo -ne "  ${name} (${mgmt_ip}): "
        python3 -c "
import urllib.request, json

url = 'http://${mgmt_ip}/cgi-bin/streamscape_api'
batch = [
    {'jsonrpc': '2.0', 'method': 'model', 'params': [], 'id': 1},
    {'jsonrpc': '2.0', 'method': 'build_tag', 'params': [], 'id': 2},
    {'jsonrpc': '2.0', 'method': 'nodeid', 'params': [], 'id': 3},
    {'jsonrpc': '2.0', 'method': 'radio_mode', 'params': [], 'id': 4},
    {'jsonrpc': '2.0', 'method': 'frequency', 'params': [], 'id': 5},
    {'jsonrpc': '2.0', 'method': 'bandwidth', 'params': [], 'id': 6},
    {'jsonrpc': '2.0', 'method': 'channel', 'params': [], 'id': 7},
    {'jsonrpc': '2.0', 'method': 'tx_power', 'params': [], 'id': 8},
    {'jsonrpc': '2.0', 'method': 'capabilities', 'params': [], 'id': 9},
    {'jsonrpc': '2.0', 'method': 'board_type', 'params': [], 'id': 10},
    {'jsonrpc': '2.0', 'method': 'encryption_mode', 'params': [], 'id': 11},
    {'jsonrpc': '2.0', 'method': 'mimo_mode', 'params': [], 'id': 12},
    {'jsonrpc': '2.0', 'method': 'version', 'params': [], 'id': 13},
    {'jsonrpc': '2.0', 'method': 'antenna_config', 'params': [], 'id': 14},
]

method_names = {r['id']: r['method'] for r in batch}

try:
    payload = json.dumps(batch).encode()
    req = urllib.request.Request(url, data=payload, headers={'Content-Type': 'application/json'})
    with urllib.request.urlopen(req, timeout=5) as resp:
        results = json.loads(resp.read())
    out = {}
    for r in results:
        rid = r.get('id', 0)
        method = method_names.get(rid, '?')
        if 'error' not in r:
            out[method] = r['result']
        else:
            out[method] = None

    print()

    def val(key, idx=0):
        v = out.get(key)
        if v is None: return '(not supported)'
        if isinstance(v, list) and len(v) > idx: return v[idx]
        return str(v)

    mdl = val('model')
    bld = val('build_tag').replace('streamscape_', '') if out.get('build_tag') else '?'
    ver = val('version')
    nid = val('nodeid')
    board = val('board_type')
    print(f'    Model:       {mdl} ({board})')
    print(f'    Firmware:    {bld} (version {ver})')
    print(f'    Node ID:     {nid}')

    rm = val('radio_mode')
    freq = val('frequency')
    bw = val('bandwidth')
    ch = val('channel')
    txp = val('tx_power')
    print(f'    Radio mode:  {rm}')
    print(f'    Frequency:   {freq} MHz')
    print(f'    Bandwidth:   {bw} MHz')
    print(f'    Channel:     {ch}')
    print(f'    TX power:    {txp}')

    mimo = val('mimo_mode')
    ant = val('antenna_config')
    enc = val('encryption_mode')
    print(f'    MIMO mode:   {mimo}')
    print(f'    Antenna:     {ant}')
    print(f'    Encryption:  {enc}')

    caps = out.get('capabilities')
    if caps and isinstance(caps, list):
        print(f'    Capabilities: {caps}')

    print()
except Exception as e:
    print(f'\033[0;31mFAILED: {e}\033[0m')
    print()
" 2>/dev/null || echo -e "${RED}unreachable${NC}"

    done < <(get_radio_mgmt_ips)
}

cmd_config_diff() {
    echo -e "${BOLD}=== Radio Configuration Diff ===${NC}"
    echo ""
    echo "Comparing settings across all radios..."
    echo ""

    python3 -c "
import urllib.request, json, yaml

with open('${TOPOLOGY}') as f:
    data = yaml.safe_load(f)

radios = data.get('silvus_radios', {}).get('radios', {})

methods = [
    'radio_mode', 'frequency', 'bandwidth', 'channel', 'tx_power',
    'encryption_mode', 'mimo_mode', 'antenna_config', 'build_tag',
    'model', 'board_type',
]

# Query all radios
all_configs = {}  # name -> {method: value}
radio_names = []
for name, info in sorted(radios.items()):
    mgmt_ip = info.get('mgmt_ip', '')
    if not mgmt_ip:
        continue
    radio_names.append(name)
    try:
        url = f'http://{mgmt_ip}/cgi-bin/streamscape_api'
        batch = [{'jsonrpc': '2.0', 'method': m, 'params': [], 'id': i+1}
                 for i, m in enumerate(methods)]
        payload = json.dumps(batch).encode()
        req = urllib.request.Request(url, data=payload, headers={'Content-Type': 'application/json'})
        with urllib.request.urlopen(req, timeout=5) as resp:
            results = json.loads(resp.read())
        cfg = {}
        for r in results:
            idx = r.get('id', 0) - 1
            if 0 <= idx < len(methods):
                method = methods[idx]
                if 'error' not in r:
                    val = r['result']
                    if isinstance(val, list) and len(val) == 1:
                        val = val[0]
                    cfg[method] = val
                else:
                    cfg[method] = None
        all_configs[name] = cfg
    except Exception as e:
        all_configs[name] = {'_error': str(e)}

if len(all_configs) < 2:
    print('  Need at least 2 reachable radios to diff.')
    import sys; sys.exit()

# Find differences
max_name = max(len(n) for n in radio_names)
max_method = max(len(m) for m in methods)

# Header
header = f'  {\"Parameter\":<{max_method}s}'
for n in radio_names:
    header += f'  {n:>{max_name + 2}s}'
print(header)
print('  ' + '-' * (max_method + (max_name + 4) * len(radio_names)))

has_diff = False
for method in methods:
    values = []
    for name in radio_names:
        cfg = all_configs.get(name, {})
        if '_error' in cfg:
            values.append('(error)')
        else:
            v = cfg.get(method)
            if v is None:
                values.append('N/A')
            elif isinstance(v, str):
                values.append(v.replace('streamscape_', ''))
            else:
                values.append(str(v))

    # Check if all values are the same
    unique = set(values)
    is_diff = len(unique) > 1

    if is_diff:
        has_diff = True
        row = f'  \033[1;33m{method:<{max_method}s}\033[0m'
    else:
        row = f'  {method:<{max_method}s}'

    for v in values:
        if is_diff:
            row += f'  \033[1;33m{v:>{max_name + 2}s}\033[0m'
        else:
            row += f'  {v:>{max_name + 2}s}'

    print(row)

print()
if has_diff:
    print('\033[1;33mDifferences found (highlighted above)\033[0m')
else:
    print('\033[0;32mAll radios have matching configuration.\033[0m')
" 2>/dev/null
}

usage() {
    echo "Usage: $(basename "$0") <command>"
    echo ""
    echo "Commands:"
    echo "  status                    Query all radios (battery, voltage, temp, mesh)"
    echo "  detect                    Identify which radio is connected to this machine"
    echo "  topology                  Show mesh routing topology"
    echo "  discover                  Scan for radio management IPs on 172.20.x.x"
    echo "  listen [port]             Listen for UDP telemetry (RSSI/voltage/temp)"
    echo "  rssi                      Show RSSI link quality matrix between radios"
    echo "  battery-watch [thr] [int] Monitor battery levels (threshold%, interval sec)"
    echo "  radio-config              Show detailed radio configuration"
    echo "  config-diff               Compare settings across all radios"
    echo ""
    echo "JSON-RPC API endpoint: /cgi-bin/streamscape_api (StreamScape 5)"
    echo ""
    echo "Setup:"
    echo "  1. Ensure route exists: sudo ip route add 172.20.0.0/16 dev <silvus_iface>"
    echo "  2. Run 'discover' or set mgmt_ip in topology.yaml"
    echo "  3. Run 'status' to query all radios"
}

case "${1:-}" in
    status)         cmd_status ;;
    detect)         cmd_detect ;;
    topology)       cmd_topology ;;
    discover)       cmd_discover ;;
    listen)         cmd_listen "${2:-5100}" ;;
    rssi)           cmd_rssi_matrix ;;
    battery-watch)  cmd_battery_watch "${2:-20}" "${3:-30}" ;;
    radio-config)   cmd_radio_config ;;
    config-diff)    cmd_config_diff ;;
    -h|--help)      usage ;;
    "")             cmd_status ;;
    *)              echo "Unknown command: $1"; usage; exit 1 ;;
esac
