#!/usr/bin/env bash
# ssh_check.sh — Verify SSH access to all machines
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TOPOLOGY="${SCRIPT_DIR}/../config/topology.yaml"

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; CYAN='\033[0;36m'; NC='\033[0m'; BOLD='\033[1m'

get_ssh_targets() {
    local network="${1:-}"
    python3 -c "
import yaml
with open('${TOPOLOGY}') as f:
    data = yaml.safe_load(f)

ssh_conf = data.get('ssh', {})
robot_user = ssh_conf.get('robot_user', 'swarm')
bs_user = ssh_conf.get('base_station_user', 'rrg')

machines = data.get('machines', {})
for name, info in sorted(machines.items()):
    role = info.get('role', 'robot')
    user = bs_user if role == 'base_station' else robot_user
    addrs = info.get('addresses', {})

    if '${network}':
        if '${network}' in addrs:
            print(f\"{name} {user} {addrs['${network}']} {role}\")
    else:
        # Use first available address
        for net, ip in addrs.items():
            print(f\"{name} {user} {ip} {role} ({net})\")
            break
"
}

network=""
timeout=3
while [[ $# -gt 0 ]]; do
    case "$1" in
        --network) network="$2"; shift 2 ;;
        --timeout) timeout="$2"; shift 2 ;;
        -h|--help)
            echo "Usage: $(basename "$0") [--network <name>] [--timeout <secs>]"
            exit 0
            ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

echo -e "${BOLD}=== SSH Access Check ===${NC}"
[[ -n "$network" ]] && echo -e "Network: ${CYAN}${network}${NC}"
echo ""
printf "${BOLD}%-12s %-8s %-18s %-14s %-10s${NC}\n" "HOST" "USER" "IP" "ROLE" "STATUS"
echo "--------------------------------------------------------------"

ok=0
fail=0

while IFS=' ' read -r name user ip role extra; do
    [[ -z "$name" ]] && continue
    if ssh -o ConnectTimeout="$timeout" -o BatchMode=yes -o StrictHostKeyChecking=no "$user@$ip" hostname &>/dev/null; then
        printf "%-12s %-8s %-18s %-14s ${GREEN}%-10s${NC}\n" "$name" "$user" "$ip" "$role" "OK"
        ((ok++))
    else
        printf "%-12s %-8s %-18s %-14s ${RED}%-10s${NC}\n" "$name" "$user" "$ip" "$role" "FAILED"
        ((fail++))
    fi
done < <(get_ssh_targets "$network")

echo ""
echo -e "Results: ${GREEN}${ok} OK${NC}, ${RED}${fail} FAILED${NC}"
