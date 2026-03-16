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

detect_local_radio() {
    # Strategy:
    #   1. Find the Silvus ethernet interface on this machine
    #   2. Check ARP table on that interface for 172.20.x.x entries
    #   3. Query each ARP-reachable 172.20.x.x via JSON-RPC to get its nodeid
    #   4. The one that responds is our directly-connected radio
    #
    # If no ARP entries, fall back to querying all known mgmt_ips and picking
    # the fastest responder (local radio responds in <1ms, remote via mesh ~10ms+)

    local iface
    iface=$(get_silvus_interface)
    if [[ -z "$iface" ]]; then
        return 1
    fi

    # Check ARP table for direct 172.20.x.x neighbors
    local arp_ips
    arp_ips=$(ip neigh show dev "$iface" 2>/dev/null | grep '172.20' | grep -v FAILED | awk '{print $1}')

    if [[ -n "$arp_ips" ]]; then
        for ip in $arp_ips; do
            local result
            result=$(python3 -c "
import urllib.request, json
url = 'http://${ip}/cgi-bin/streamscape_api'
batch = [{'jsonrpc': '2.0', 'method': 'nodeid', 'params': [], 'id': 1},
         {'jsonrpc': '2.0', 'method': 'local_address', 'params': [], 'id': 2}]
try:
    req = urllib.request.Request(url, data=json.dumps(batch).encode(),
                                 headers={'Content-Type': 'application/json'})
    with urllib.request.urlopen(req, timeout=2) as resp:
        results = json.loads(resp.read())
    nid = ''; addr = ''
    for r in results:
        if r['id'] == 1 and 'error' not in r: nid = r['result'][0]
        if r['id'] == 2 and 'error' not in r: addr = r['result'][0]
    print(f'{nid}|{addr}')
except:
    pass
" 2>/dev/null)
            if [[ -n "$result" ]]; then
                echo "$result"
                return 0
            fi
        done
    fi

    # Fallback: try each known mgmt_ip with TTL=1 ping — only the directly-connected
    # radio will respond (single L2 hop). Remote radios need mesh hops and won't reply.
    while IFS='|' read -r rname mgmt_ip rnode_id; do
        [[ -z "$mgmt_ip" ]] && continue
        if ping -c 1 -W 1 -t 1 "$mgmt_ip" &>/dev/null; then
            # TTL=1 succeeded — this radio is directly connected
            local result
            result=$(python3 -c "
import urllib.request, json
url = 'http://${mgmt_ip}/cgi-bin/streamscape_api'
batch = [{'jsonrpc': '2.0', 'method': 'nodeid', 'params': [], 'id': 1},
         {'jsonrpc': '2.0', 'method': 'local_address', 'params': [], 'id': 2}]
try:
    req = urllib.request.Request(url, data=json.dumps(batch).encode(),
                                 headers={'Content-Type': 'application/json'})
    with urllib.request.urlopen(req, timeout=3) as resp:
        results = json.loads(resp.read())
    nid = ''; addr = ''
    for r in results:
        if r['id'] == 1 and 'error' not in r: nid = r['result'][0]
        if r['id'] == 2 and 'error' not in r: addr = r['result'][0]
    print(f'{nid}|{addr}')
except:
    pass
" 2>/dev/null)
            if [[ -n "$result" ]]; then
                echo "$result"
                return 0
            fi
        fi
    done < <(get_radio_mgmt_ips)
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
    echo -e "Silvus interface: ${CYAN}${iface}${NC}"

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

    # ARP-based detection
    echo ""
    echo "Checking ARP table for directly-connected radio..."
    local arp_ips
    arp_ips=$(ip neigh show dev "$iface" 2>/dev/null | grep '172.20' | grep -v FAILED | awk '{print $1}')

    if [[ -z "$arp_ips" ]]; then
        echo "No ARP entries. Pinging known radio management IPs to populate ARP..."
        while IFS='|' read -r name mgmt_ip node_id; do
            [[ -z "$mgmt_ip" ]] && continue
            ping -c 1 -W 1 "$mgmt_ip" &>/dev/null &
        done < <(get_radio_mgmt_ips)
        wait
        sleep 1
        arp_ips=$(ip neigh show dev "$iface" 2>/dev/null | grep '172.20' | grep -v FAILED | awk '{print $1}')
    fi

    if [[ -z "$arp_ips" ]]; then
        echo -e "${YELLOW}No radio management IPs reachable on ${iface}.${NC}"
        echo ""
        echo "Falling back to timing-based detection..."
    fi

    # TTL=1 ping test — only the directly-connected radio will respond
    echo ""
    echo -e "${BOLD}Testing radios with TTL=1 ping (only direct L2 neighbor responds):${NC}"
    printf "  ${BOLD}%-20s %-18s %-10s %s${NC}\n" "RADIO" "MGMT_IP" "NODE_ID" "RESULT"
    echo "  ----------------------------------------------------------"

    local detected_name=""
    local detected_nid=""
    while IFS='|' read -r name mgmt_ip node_id; do
        [[ -z "$name" ]] && continue
        [[ -z "$mgmt_ip" ]] && continue

        if ping -c 1 -W 1 -t 1 "$mgmt_ip" &>/dev/null; then
            printf "  %-20s %-18s %-10s ${CYAN}%s${NC}\n" "$name" "$mgmt_ip" "$node_id" "<-- LOCAL (TTL=1 OK)"
            detected_name="$name"
            detected_nid="$node_id"
        else
            # Check if reachable at all (multi-hop)
            if ping -c 1 -W 2 "$mgmt_ip" &>/dev/null; then
                printf "  %-20s %-18s %-10s %s\n" "$name" "$mgmt_ip" "$node_id" "reachable (multi-hop)"
            else
                printf "  %-20s %-18s %-10s ${RED}%s${NC}\n" "$name" "$mgmt_ip" "$node_id" "unreachable"
            fi
        fi
    done < <(get_radio_mgmt_ips)

    echo ""
    if [[ -n "$detected_name" ]]; then
        echo -e "${BOLD}Detected local radio: ${detected_name} (node ${detected_nid})${NC}"
    else
        echo -e "${YELLOW}Could not detect local radio via TTL=1.${NC}"
        echo "This may happen if the radio is on the same L2 broadcast domain."
        echo "Try: silvus_status.sh status (uses API response timing as fallback)"
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

usage() {
    echo "Usage: $(basename "$0") <command>"
    echo ""
    echo "Commands:"
    echo "  status              Query all radios (battery, voltage, temp, mesh) + detect local"
    echo "  detect              Identify which radio is directly connected to this machine"
    echo "  topology            Show mesh routing topology"
    echo "  discover            Scan for radio management IPs on 172.20.x.x"
    echo "  listen [port]       Listen for UDP telemetry (RSSI/voltage/temp) [default: 5100]"
    echo ""
    echo "JSON-RPC API endpoint: /cgi-bin/streamscape_api (StreamScape 5)"
    echo ""
    echo "Setup:"
    echo "  1. Ensure route exists: sudo ip route add 172.20.0.0/16 dev <silvus_iface>"
    echo "  2. Run 'discover' or set mgmt_ip in topology.yaml"
    echo "  3. Run 'status' to query all radios"
}

case "${1:-}" in
    status)     cmd_status ;;
    detect)     cmd_detect ;;
    topology)   cmd_topology ;;
    discover)   cmd_discover ;;
    listen)     cmd_listen "${2:-5100}" ;;
    -h|--help)  usage ;;
    "")         cmd_status ;;
    *)          echo "Unknown command: $1"; usage; exit 1 ;;
esac
