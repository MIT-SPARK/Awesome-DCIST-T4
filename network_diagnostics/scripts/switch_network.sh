#!/usr/bin/env bash
# switch_network.sh — Switch between WiFi and Silvus networks
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TOPOLOGY="${SCRIPT_DIR}/../config/topology.yaml"

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; CYAN='\033[0;36m'; NC='\033[0m'; BOLD='\033[1m'

get_subnets() {
    python3 -c "
import yaml
with open('${TOPOLOGY}') as f:
    data = yaml.safe_load(f)
nets = data.get('networks', {})
for name, info in nets.items():
    print(f\"{name} {info['subnet']}\")
"
}

get_peer_ip() {
    local network="$1"
    local hostname
    hostname=$(hostname)
    python3 -c "
import yaml
with open('${TOPOLOGY}') as f:
    data = yaml.safe_load(f)
machines = data.get('machines', {})
for name, info in sorted(machines.items()):
    if name == '${hostname}':
        continue
    addrs = info.get('addresses', {})
    if '${network}' in addrs:
        print(addrs['${network}'])
        break
"
}

get_zenoh_endpoints() {
    local network="$1"
    local hostname
    hostname=$(hostname)
    python3 -c "
import yaml
with open('${TOPOLOGY}') as f:
    data = yaml.safe_load(f)
port = data.get('zenoh', {}).get('port', 7447)
machines = data.get('machines', {})
for name, info in sorted(machines.items()):
    if name == '${hostname}':
        continue
    addrs = info.get('addresses', {})
    if '${network}' in addrs:
        print(f\"tcp/{addrs['${network}']}:{port}\")
"
}

detect_current_network() {
    while IFS=' ' read -r netname subnet; do
        local prefix
        prefix=$(echo "$subnet" | cut -d'/' -f1 | sed 's/\.[0-9]*$//')
        if ip addr show 2>/dev/null | grep -q "inet ${prefix}"; then
            echo "$netname"
            return
        fi
    done < <(get_subnets)
    echo "unknown"
}

switch_to_silvus() {
    echo -e "${BOLD}Switching to Silvus network...${NC}"

    # Find Silvus-connected ethernet interface (typically a USB-Ethernet adapter)
    local silvus_iface=""
    for iface in $(ls /sys/class/net/ 2>/dev/null); do
        [[ "$iface" == "lo" || "$iface" == "docker0" || "$iface" == "br0" ]] && continue
        # Check if this is an ethernet interface (not wireless)
        if [[ ! -d "/sys/class/net/${iface}/wireless" ]]; then
            # Check if it could be Silvus (has 192.168.100.x address or is an eth adapter)
            if ip addr show "$iface" 2>/dev/null | grep -q "192.168.100"; then
                silvus_iface="$iface"
                break
            fi
        fi
    done

    if [[ -z "$silvus_iface" ]]; then
        # Try to find an ethernet interface that isn't the primary one
        for iface in $(ls /sys/class/net/ 2>/dev/null); do
            [[ "$iface" == "lo" || "$iface" == "docker0" || "$iface" == "br0" ]] && continue
            if [[ ! -d "/sys/class/net/${iface}/wireless" ]]; then
                silvus_iface="$iface"
                echo -e "${YELLOW}No active Silvus interface detected. Trying ${silvus_iface}...${NC}"
                break
            fi
        done
    fi

    if [[ -z "$silvus_iface" ]]; then
        echo -e "${RED}No ethernet interface found for Silvus.${NC}"
        return 1
    fi

    echo -e "Using interface: ${CYAN}${silvus_iface}${NC}"

    # Bring up the Silvus interface
    sudo ip link set "$silvus_iface" up

    # Disable WiFi to avoid routing conflicts
    if command -v nmcli &>/dev/null; then
        echo "Disabling WiFi via nmcli..."
        nmcli radio wifi off 2>/dev/null || true
    fi

    # Verify
    sleep 1
    local peer_ip
    peer_ip=$(get_peer_ip "silvus")
    if [[ -n "$peer_ip" ]]; then
        echo -n "Verifying connectivity to peer ${peer_ip}... "
        if ping -c 1 -W 2 "$peer_ip" &>/dev/null; then
            echo -e "${GREEN}OK${NC}"
        else
            echo -e "${RED}FAILED${NC}"
            echo -e "${YELLOW}Interface is up but peer not reachable. Check Silvus radio.${NC}"
        fi
    fi
}

switch_to_wifi() {
    echo -e "${BOLD}Switching to WiFi network...${NC}"

    if ! command -v nmcli &>/dev/null; then
        echo -e "${RED}nmcli not found — cannot manage WiFi.${NC}"
        return 1
    fi

    # Enable WiFi
    nmcli radio wifi on

    # Wait for connection
    echo "Waiting for WiFi connection..."
    sleep 3

    local wifi_conn
    wifi_conn=$(nmcli -t -f NAME,TYPE connection show --active 2>/dev/null | grep ':wifi' | head -1 | cut -d: -f1)

    if [[ -n "$wifi_conn" ]]; then
        echo -e "Connected to WiFi: ${GREEN}${wifi_conn}${NC}"
    else
        echo -e "${YELLOW}WiFi enabled but no active connection. Connect manually:${NC}"
        echo "  nmcli device wifi list"
        echo "  nmcli device wifi connect <SSID>"
    fi

    # Detect which WiFi network we're on
    local detected
    detected=$(detect_current_network)
    echo -e "Detected network: ${CYAN}${detected}${NC}"

    if [[ "$detected" != "unknown" ]]; then
        local peer_ip
        peer_ip=$(get_peer_ip "$detected")
        if [[ -n "$peer_ip" ]]; then
            echo -n "Verifying connectivity to peer ${peer_ip}... "
            if ping -c 1 -W 2 "$peer_ip" &>/dev/null; then
                echo -e "${GREEN}OK${NC}"
            else
                echo -e "${RED}FAILED${NC}"
            fi
        fi
    fi
}

show_zenoh_hints() {
    local network="$1"
    echo ""
    echo -e "${BOLD}Zenoh connect endpoints for ${network}:${NC}"
    get_zenoh_endpoints "$network" | while read -r endpoint; do
        echo "  $endpoint"
    done
    echo ""
    echo "To generate a full Zenoh config:"
    echo "  $(dirname "$0")/gen_zenoh_config.sh --network ${network}"
}

usage() {
    echo "Usage: $(basename "$0") --to <silvus|wifi> [--show-zenoh]"
    echo ""
    echo "Options:"
    echo "  --to <target>    Switch to 'silvus' or 'wifi'"
    echo "  --show-zenoh     Show Zenoh config hints after switching"
    echo "  --status         Show current network status"
}

show_status() {
    echo -e "${BOLD}=== Network Status ===${NC}"
    local current
    current=$(detect_current_network)
    echo -e "Detected active network: ${CYAN}${current}${NC}"
    echo ""
    echo "Active interfaces:"
    ip -brief addr show | grep -v "^lo " | while read -r line; do
        echo "  $line"
    done
}

target=""
show_zenoh=false
while [[ $# -gt 0 ]]; do
    case "$1" in
        --to) target="$2"; shift 2 ;;
        --show-zenoh) show_zenoh=true; shift ;;
        --status) show_status; exit 0 ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Unknown option: $1"; usage; exit 1 ;;
    esac
done

if [[ -z "$target" ]]; then
    usage
    exit 1
fi

case "$target" in
    silvus)
        switch_to_silvus
        $show_zenoh && show_zenoh_hints "silvus"
        ;;
    wifi)
        switch_to_wifi
        detected=$(detect_current_network)
        $show_zenoh && [[ "$detected" != "unknown" ]] && show_zenoh_hints "$detected"
        ;;
    *)
        echo -e "${RED}Unknown target: ${target}. Use 'silvus' or 'wifi'.${NC}"
        exit 1
        ;;
esac
