#!/usr/bin/env bash
# silvus_status.sh — Silvus StreamCaster radio status: web probe, UDP telemetry listener, discovery
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TOPOLOGY="${SCRIPT_DIR}/../config/topology.yaml"

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; CYAN='\033[0;36m'; NC='\033[0m'; BOLD='\033[1m'

get_silvus_interface() {
    python3 -c "
import yaml, subprocess, socket
with open('${TOPOLOGY}') as f:
    data = yaml.safe_load(f)
hostname = socket.gethostname()
machines = data.get('machines', {})
if hostname in machines:
    silvus_ip = machines[hostname].get('addresses', {}).get('silvus', '')
    if silvus_ip:
        result = subprocess.run(['ip', '-o', 'addr', 'show'], capture_output=True, text=True)
        for line in result.stdout.strip().split('\n'):
            if silvus_ip in line:
                print(line.split()[1])
                break
"
}

get_radio_info() {
    python3 -c "
import yaml
with open('${TOPOLOGY}') as f:
    data = yaml.safe_load(f)
radios = data.get('silvus_radios', {}).get('radios', {})
for name, info in sorted(radios.items()):
    mgmt = info.get('mgmt_ip', '')
    data_ip = info.get('data_ip', '')
    attached = info.get('attached_to', '')
    print(f'{name} {mgmt} {data_ip} {attached}')
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

cmd_discover() {
    local iface
    iface=$(get_silvus_interface)
    if [[ -z "$iface" ]]; then
        echo -e "${RED}Could not find Silvus network interface.${NC}"
        echo "This machine may not have a Silvus radio connected."
        return 1
    fi

    local mgmt_subnet
    mgmt_subnet=$(get_mgmt_subnet)

    echo -e "${BOLD}=== Silvus Radio Discovery ===${NC}"
    echo -e "Silvus interface: ${CYAN}${iface}${NC}"
    echo -e "Management subnet: ${CYAN}${mgmt_subnet}${NC}"
    echo ""

    # Ensure route exists for mgmt subnet
    local subnet_prefix
    subnet_prefix=$(echo "$mgmt_subnet" | cut -d'/' -f1 | sed 's/\.[0-9]*$//')
    if ! ip route show | grep -q "$mgmt_subnet"; then
        echo -e "${YELLOW}Adding route for ${mgmt_subnet} via ${iface}...${NC}"
        echo -e "${YELLOW}  sudo ip route add ${mgmt_subnet} dev ${iface}${NC}"
        sudo ip route add "$mgmt_subnet" dev "$iface" 2>/dev/null || {
            echo -e "${RED}Failed to add route (need sudo). Run manually:${NC}"
            echo "  sudo ip route add ${mgmt_subnet} dev ${iface}"
            return 1
        }
    fi

    echo "Scanning ${mgmt_subnet} for radio management interfaces..."
    echo "(This may take a while)"
    echo ""

    # Use the subnet prefix (172.20) and scan common patterns
    # Silvus radios typically use 172.20.{high}.{low} based on node ID
    local found=0
    for subnet in "${subnet_prefix}.0" "${subnet_prefix}.1" "${subnet_prefix}.2" "${subnet_prefix}.3" "${subnet_prefix}.4" "${subnet_prefix}.5"; do
        for host in $(seq 1 254); do
            local ip="${subnet}.${host}"
            if ping -c 1 -W 1 "$ip" &>/dev/null; then
                local http_status
                http_status=$(curl -s -m 2 -o /dev/null -w '%{http_code}' "http://${ip}/" 2>/dev/null || echo "000")
                if [[ "$http_status" != "000" ]]; then
                    echo -e "  ${GREEN}FOUND${NC} ${ip} (HTTP ${http_status})"
                    ((found++))
                else
                    echo -e "  ${CYAN}PING OK${NC} ${ip} (no HTTP)"
                fi
            fi
        done
    done

    echo ""
    if [[ $found -gt 0 ]]; then
        echo -e "${GREEN}Found ${found} radio(s) with web interface.${NC}"
        echo "Update topology.yaml silvus_radios.radios.*.mgmt_ip with discovered IPs."
    else
        echo -e "${YELLOW}No radio web interfaces found.${NC}"
        echo "Try scanning additional subnets or check radio documentation for management IP."
    fi
}

cmd_probe() {
    echo -e "${BOLD}=== Silvus Radio Web Probe ===${NC}"
    echo ""
    printf "${BOLD}%-18s %-18s %-12s %-10s %s${NC}\n" "RADIO" "MGMT_IP" "ATTACHED" "HTTP" "INFO"
    echo "------------------------------------------------------------------------"

    while IFS=' ' read -r name mgmt_ip data_ip attached; do
        [[ -z "$name" ]] && continue

        if [[ -z "$mgmt_ip" ]]; then
            printf "%-18s %-18s %-12s ${YELLOW}%-10s${NC} %s\n" "$name" "(not set)" "$attached" "SKIP" "Set mgmt_ip in topology.yaml"
            continue
        fi

        local http_status
        http_status=$(curl -s -m 3 -o /dev/null -w '%{http_code}' "http://${mgmt_ip}/" 2>/dev/null || echo "000")

        if [[ "$http_status" == "000" ]]; then
            printf "%-18s %-18s %-12s ${RED}%-10s${NC}\n" "$name" "$mgmt_ip" "$attached" "DOWN"
            continue
        fi

        # Try to scrape info from the web page
        local page_content
        page_content=$(curl -s -m 5 "http://${mgmt_ip}/" 2>/dev/null || echo "")

        local info=""
        if [[ -n "$page_content" ]]; then
            # Try to extract voltage/temp from the page header
            local voltage temp
            voltage=$(echo "$page_content" | grep -oP 'Voltage[:\s]*[\d.]+\s*V' | head -1 || echo "")
            temp=$(echo "$page_content" | grep -oP 'Temp[erature]*[:\s]*[\d.]+\s*[°CcF]' | head -1 || echo "")
            [[ -n "$voltage" ]] && info+="$voltage "
            [[ -n "$temp" ]] && info+="$temp "
        fi

        printf "%-18s %-18s %-12s ${GREEN}%-10s${NC} %s\n" "$name" "$mgmt_ip" "$attached" "HTTP $http_status" "$info"

        # Try known API patterns
        for endpoint in "/api/status" "/streamscape/api/status" "/cgi-bin/status" "/api/v1/status" "/status.json"; do
            local resp
            resp=$(curl -s -m 2 "http://${mgmt_ip}${endpoint}" 2>/dev/null)
            if [[ -n "$resp" ]] && [[ "$resp" != *"404"* ]] && [[ "$resp" != *"Not Found"* ]]; then
                echo -e "    ${CYAN}Found endpoint: ${endpoint}${NC}"
                echo "$resp" | head -5 | sed 's/^/    /'
            fi
        done
    done < <(get_radio_info)
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
import time
from datetime import datetime

PORT = ${port}

# TLV type names from Silvus StreamCaster manual
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
    \"\"\"Parse TLV-encoded Silvus UDP telemetry packet.\"\"\"
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
        # Strip null terminator
        value_str = value_bytes.rstrip(b'\x00').decode('ascii', errors='replace')
        offset += tlv_len

        type_name = TYPE_NAMES.get(tlv_type, f'UNKNOWN_{tlv_type}')
        current_report[type_name] = value_str

        if tlv_type == 1:  # END_REPORT
            reports.append(current_report)
            current_report = {}

    if current_report:
        reports.append(current_report)
    return reports

def format_rssi(report):
    \"\"\"Format RSSI report with SNR calculation.\"\"\"
    parts = []
    node_id = report.get('NODE_ID', '?')
    parts.append(f'Node {node_id}')

    for ant in ['RSSI_ANT1', 'RSSI_ANT2', 'RSSI_ANT3', 'RSSI_ANT4']:
        if ant in report:
            # Values in half-dBm steps
            val = int(report[ant]) / 2.0
            parts.append(f'{ant[-4:]}: {val:.1f} dBm')

    if 'NOISE_FLOOR' in report:
        noise = int(report['NOISE_FLOOR']) / 2.0
        parts.append(f'Noise: {noise:.1f} dBm')

    # SNR from sync signal/noise
    if 'SYNC_SIGNAL' in report and 'SYNC_NOISE' in report:
        try:
            x = int(report['SYNC_SIGNAL'])
            y = int(report['SYNC_NOISE'])
            z = (y - x) / 51.0
            if z > 0:
                import math
                snr_mw = (x - 12 * z) / (64 * z)
                if snr_mw > 0:
                    snr_db = 10 * math.log10(snr_mw)
                    parts.append(f'SNR: {snr_db:.1f} dB')
        except (ValueError, ZeroDivisionError):
            pass

    return ' | '.join(parts)

def format_voltage(report):
    \"\"\"Format voltage report.\"\"\"
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
    \"\"\"Format temperature report.\"\"\"
    parts = []
    if 'TEMPERATURE' in report:
        parts.append(f'Temp: {report[\"TEMPERATURE\"]}°C')
    if 'MAX_TEMPERATURE' in report:
        parts.append(f'Max: {report[\"MAX_TEMPERATURE\"]}°C')
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

cmd_status() {
    echo -e "${BOLD}=== Silvus Radio Status ===${NC}"
    echo ""

    # Check Silvus data plane connectivity
    echo -e "${BOLD}Data plane (ping):${NC}"
    printf "  ${BOLD}%-18s %-18s %-12s %-10s %s${NC}\n" "RADIO" "DATA_IP" "ATTACHED" "STATUS" "RTT"
    echo "  ----------------------------------------------------------------------"

    while IFS=' ' read -r name mgmt_ip data_ip attached; do
        [[ -z "$name" ]] && continue
        if [[ -z "$data_ip" ]]; then
            continue
        fi
        local result
        if result=$(ping -c 1 -W 2 "$data_ip" 2>/dev/null | grep 'time='); then
            local rtt
            rtt=$(echo "$result" | sed -n 's/.*time=\([0-9.]*\).*/\1/p')
            printf "  %-18s %-18s %-12s ${GREEN}%-10s${NC} %s ms\n" "$name" "$data_ip" "$attached" "UP" "$rtt"
        else
            printf "  %-18s %-18s %-12s ${RED}%-10s${NC}\n" "$name" "$data_ip" "$attached" "DOWN"
        fi
    done < <(get_radio_info)

    echo ""

    # Try web probe for radios with mgmt IPs configured
    local has_mgmt=false
    while IFS=' ' read -r name mgmt_ip data_ip attached; do
        [[ -n "$mgmt_ip" ]] && has_mgmt=true
    done < <(get_radio_info)

    if $has_mgmt; then
        echo -e "${BOLD}Management interfaces:${NC}"
        cmd_probe
    else
        echo -e "${YELLOW}No management IPs configured in topology.yaml.${NC}"
        echo "Run '$(basename "$0") discover' to find radio management IPs."
    fi
}

usage() {
    echo "Usage: $(basename "$0") <command>"
    echo ""
    echo "Commands:"
    echo "  status              Show radio data plane connectivity + web probe"
    echo "  discover            Scan for radio management IPs on 172.20.x.x"
    echo "  probe               Probe known radio web interfaces for status/API"
    echo "  listen [port]       Listen for UDP telemetry (RSSI/voltage/temp) [default: 5100]"
    echo ""
    echo "Setup:"
    echo "  1. Run 'discover' to find radio management IPs"
    echo "  2. Update topology.yaml silvus_radios.radios.*.mgmt_ip"
    echo "  3. Configure UDP streaming on each radio's Node Diagnostics page"
    echo "  4. Run 'listen' to receive telemetry"
}

case "${1:-}" in
    status)     cmd_status ;;
    discover)   cmd_discover ;;
    probe)      cmd_probe ;;
    listen)     cmd_listen "${2:-5100}" ;;
    -h|--help)  usage ;;
    "")         cmd_status ;;
    *)          echo "Unknown command: $1"; usage; exit 1 ;;
esac
