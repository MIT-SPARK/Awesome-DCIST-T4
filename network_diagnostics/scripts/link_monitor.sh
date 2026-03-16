#!/usr/bin/env bash
# link_monitor.sh — Continuous live link status dashboard
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TOPOLOGY="${SCRIPT_DIR}/../config/topology.yaml"

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; CYAN='\033[0;36m'; NC='\033[0m'; BOLD='\033[1m'; DIM='\033[2m'

INTERVAL="${1:-2}"

get_machines_on_network() {
    local network="$1"
    python3 -c "
import yaml
with open('${TOPOLOGY}') as f:
    data = yaml.safe_load(f)
machines = data.get('machines', {})
for name, info in sorted(machines.items()):
    addrs = info.get('addresses', {})
    if '${network}' in addrs:
        print(f\"{name} {addrs['${network}']} {info.get('role','unknown')}\")
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

detect_active_network() {
    python3 -c "
import yaml, subprocess
with open('${TOPOLOGY}') as f:
    data = yaml.safe_load(f)
ip_out = subprocess.check_output(['ip', 'addr', 'show'], text=True)
for netname, info in data.get('networks', {}).items():
    prefix = info['subnet'].rsplit('.', 1)[0].rsplit('.', 1)[0]
    if f'inet {prefix}.' in ip_out or f'inet {prefix}/' in ip_out:
        print(netname)
"
}

echo -e "${BOLD}Link Monitor${NC} — refreshing every ${INTERVAL}s (Ctrl+C to stop)"
echo ""

# Determine which networks to monitor
active_networks=$(detect_active_network 2>/dev/null || echo "silvus")
if [[ -z "$active_networks" ]]; then
    active_networks="silvus"
fi

trap 'echo -e "\n${NC}Stopped."; exit 0' INT

while true; do
    # Clear screen and move cursor to top
    clear

    echo -e "${BOLD}=== Link Monitor ===${NC}  $(date '+%H:%M:%S')  (every ${INTERVAL}s)"
    echo ""

    for network in $active_networks; do
        echo -e "${CYAN}Network: ${network}${NC}"
        printf "  ${BOLD}%-12s %-18s %-8s %s${NC}\n" "HOST" "IP" "STATUS" "RTT"
        echo "  ------------------------------------------------"

        while IFS=' ' read -r name ip role; do
            [[ -z "$name" ]] && continue
            local_result=$(ping -c 1 -W 1 "$ip" 2>/dev/null) || true
            if echo "$local_result" | grep -q 'time='; then
                rtt=$(echo "$local_result" | grep 'time=' | sed -n 's/.*time=\([0-9.]*\).*/\1/p')
                if (( $(echo "$rtt > 100" | bc -l 2>/dev/null || echo 0) )); then
                    printf "  %-12s %-18s ${YELLOW}%-8s${NC} %s ms\n" "$name" "$ip" "SLOW" "$rtt"
                else
                    printf "  %-12s %-18s ${GREEN}%-8s${NC} %s ms\n" "$name" "$ip" "UP" "$rtt"
                fi
            else
                printf "  %-12s %-18s ${RED}%-8s${NC}\n" "$name" "$ip" "DOWN"
            fi
        done < <(get_machines_on_network "$network")
        echo ""
    done

    echo -e "${DIM}Press Ctrl+C to stop${NC}"
    sleep "$INTERVAL"
done
