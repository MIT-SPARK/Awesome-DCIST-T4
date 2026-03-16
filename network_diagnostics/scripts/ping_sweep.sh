#!/usr/bin/env bash
# ping_sweep.sh — Non-ROS network diagnostics: ping sweep, bandwidth, traceroute, MTU
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TOPOLOGY="${SCRIPT_DIR}/../config/topology.yaml"

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; CYAN='\033[0;36m'; NC='\033[0m'; BOLD='\033[1m'

parse_topology() {
    python3 -c "
import yaml, json, sys
with open('${TOPOLOGY}') as f:
    data = yaml.safe_load(f)
json.dump(data, sys.stdout)
"
}

get_machines_on_network() {
    local network="$1"
    python3 -c "
import yaml, json
with open('${TOPOLOGY}') as f:
    data = yaml.safe_load(f)
machines = data.get('machines', {})
for name, info in sorted(machines.items()):
    addrs = info.get('addresses', {})
    if '${network}' in addrs:
        print(f\"{name} {addrs['${network}']} {info.get('role','unknown')} {info.get('desc','')}\")
"
}

get_all_networks() {
    python3 -c "
import yaml
with open('${TOPOLOGY}') as f:
    data = yaml.safe_load(f)
for net in data.get('networks', {}):
    print(net)
"
}

cmd_sweep() {
    local network="${1:-}"

    if [[ -z "$network" ]]; then
        # Sweep all networks
        local networks
        networks=$(get_all_networks)
        for net in $networks; do
            cmd_sweep "$net"
            echo ""
        done
        return
    fi

    echo -e "${BOLD}=== Ping Sweep: ${network} ===${NC}"
    printf "${BOLD}%-12s %-18s %-14s %-8s %s${NC}\n" "HOST" "IP" "ROLE" "STATUS" "RTT"
    echo "--------------------------------------------------------------"

    while IFS=' ' read -r name ip role desc; do
        [[ -z "$name" ]] && continue
        local result
        if result=$(ping -c 1 -W 2 "$ip" 2>/dev/null | grep 'time='); then
            local rtt
            rtt=$(echo "$result" | sed -n 's/.*time=\([0-9.]*\).*/\1/p')
            printf "%-12s %-18s %-14s ${GREEN}%-8s${NC} %s ms\n" "$name" "$ip" "$role" "UP" "$rtt"
        else
            printf "%-12s %-18s %-14s ${RED}%-8s${NC} %s\n" "$name" "$ip" "$role" "DOWN" "-"
        fi
    done < <(get_machines_on_network "$network")
}

cmd_bandwidth() {
    local target_ip="$1"
    if ! command -v iperf3 &>/dev/null; then
        echo -e "${YELLOW}iperf3 not installed — skipping bandwidth test${NC}"
        echo "Install with: sudo apt install iperf3"
        echo "Note: iperf3 server must be running on target (iperf3 -s)"
        return 1
    fi
    echo -e "${BOLD}=== Bandwidth test to ${target_ip} ===${NC}"
    echo "Ensure iperf3 -s is running on the target."
    iperf3 -c "$target_ip" -t 10
}

cmd_traceroute() {
    local target_ip="$1"
    echo -e "${BOLD}=== Traceroute to ${target_ip} ===${NC}"
    if command -v traceroute &>/dev/null; then
        traceroute -w 2 "$target_ip"
    else
        echo -e "${YELLOW}traceroute not installed, using ping-based trace${NC}"
        for ttl in 1 2 3 4 5 10 15 20; do
            local result
            result=$(ping -c 1 -W 2 -t "$ttl" "$target_ip" 2>/dev/null | head -2) || true
            echo "TTL=$ttl: $result"
        done
    fi
}

cmd_mtu() {
    local target_ip="$1"
    echo -e "${BOLD}=== MTU Path Discovery to ${target_ip} ===${NC}"
    local mtu_found=0
    for size in 1500 1472 1400 1300 1200 1100 1000 500; do
        if ping -c 1 -W 2 -M do -s "$size" "$target_ip" &>/dev/null; then
            echo -e "  ${GREEN}Size $size: OK${NC}"
            mtu_found=$((size + 28))  # 20 IP + 8 ICMP header
        else
            echo -e "  ${RED}Size $size: blocked${NC}"
        fi
    done
    if [[ $mtu_found -gt 0 ]]; then
        echo -e "${BOLD}Estimated path MTU: ${mtu_found} bytes${NC}"
    else
        echo -e "${RED}Could not determine MTU — host may be unreachable${NC}"
    fi
}

usage() {
    echo "Usage: $(basename "$0") <command> [options]"
    echo ""
    echo "Commands:"
    echo "  sweep [--network <name>]   Ping all machines (optionally filter by network)"
    echo "  bandwidth <target_ip>      Run iperf3 bandwidth test"
    echo "  traceroute <target_ip>     Trace route to target"
    echo "  mtu <target_ip>            Test MTU path to target"
    echo ""
    echo "Networks: silvus, mit_wifi, penn_wifi"
}

case "${1:-}" in
    sweep)
        shift
        network=""
        while [[ $# -gt 0 ]]; do
            case "$1" in
                --network) network="$2"; shift 2 ;;
                *) echo "Unknown option: $1"; usage; exit 1 ;;
            esac
        done
        cmd_sweep "$network"
        ;;
    bandwidth)
        [[ $# -lt 2 ]] && { echo "Usage: $0 bandwidth <target_ip>"; exit 1; }
        cmd_bandwidth "$2"
        ;;
    traceroute)
        [[ $# -lt 2 ]] && { echo "Usage: $0 traceroute <target_ip>"; exit 1; }
        cmd_traceroute "$2"
        ;;
    mtu)
        [[ $# -lt 2 ]] && { echo "Usage: $0 mtu <target_ip>"; exit 1; }
        cmd_mtu "$2"
        ;;
    -h|--help|"")
        usage
        ;;
    *)
        echo "Unknown command: $1"
        usage
        exit 1
        ;;
esac
