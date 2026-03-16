#!/usr/bin/env bash
# zenoh_check.sh — Zenoh connectivity diagnostics (no ROS2 required)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TOPOLOGY="${SCRIPT_DIR}/../config/topology.yaml"

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; CYAN='\033[0;36m'; NC='\033[0m'; BOLD='\033[1m'

get_zenoh_port() {
    python3 -c "
import yaml
with open('${TOPOLOGY}') as f:
    data = yaml.safe_load(f)
print(data.get('zenoh', {}).get('port', 7447))
"
}

get_connect_network() {
    python3 -c "
import yaml
with open('${TOPOLOGY}') as f:
    data = yaml.safe_load(f)
print(data.get('zenoh', {}).get('connect_network', 'silvus'))
"
}

get_peers() {
    local network="$1"
    local port="$2"
    python3 -c "
import yaml
with open('${TOPOLOGY}') as f:
    data = yaml.safe_load(f)
machines = data.get('machines', {})
for name, info in sorted(machines.items()):
    addrs = info.get('addresses', {})
    if '${network}' in addrs:
        print(f\"{name} {addrs['${network}']} ${port}\")
"
}

check_local_zenohd() {
    echo -e "${BOLD}[1/3] Local rmw_zenohd process${NC}"
    if pgrep -f "rmw_zenohd" &>/dev/null; then
        local pid
        pid=$(pgrep -f "rmw_zenohd" | head -1)
        echo -e "  ${GREEN}RUNNING${NC} (PID: ${pid})"
        return 0
    else
        echo -e "  ${YELLOW}NOT RUNNING — starting zenohd...${NC}"
        if command -v ros2 &>/dev/null; then
            ros2 run rmw_zenoh_cpp rmw_zenohd &>/dev/null &
            local new_pid=$!
            sleep 2
            if kill -0 "$new_pid" 2>/dev/null; then
                echo -e "  ${GREEN}STARTED${NC} (PID: ${new_pid})"
                return 0
            else
                echo -e "  ${RED}FAILED to start zenohd${NC}"
                return 1
            fi
        else
            echo -e "  ${RED}ros2 not found — source your workspace first${NC}"
            echo -e "  ${YELLOW}source \${ADT4_WS}/install/setup.zsh${NC}"
            return 1
        fi
    fi
}

check_local_port() {
    local port="$1"
    echo -e "\n${BOLD}[2/3] Local port ${port} listener${NC}"
    if ss -tlnp 2>/dev/null | grep -q ":${port} "; then
        echo -e "  ${GREEN}LISTENING${NC} on port ${port}"
        return 0
    else
        echo -e "  ${RED}NOT LISTENING${NC} on port ${port}"
        return 1
    fi
}

check_remote_peers() {
    local network="$1"
    local port="$2"
    local hostname
    hostname=$(hostname)

    echo -e "\n${BOLD}[3/3] Remote Zenoh port reachability (${network}, port ${port})${NC}"
    printf "  ${BOLD}%-12s %-18s %-12s${NC}\n" "HOST" "IP" "STATUS"
    echo "  ----------------------------------------"

    while IFS=' ' read -r name ip peer_port; do
        [[ -z "$name" ]] && continue
        [[ "$name" == "$hostname" ]] && continue

        if nc -z -w 2 "$ip" "$peer_port" 2>/dev/null; then
            printf "  %-12s %-18s ${GREEN}%-12s${NC}\n" "$name" "$ip" "REACHABLE"
        else
            printf "  %-12s %-18s ${RED}%-12s${NC}\n" "$name" "$ip" "UNREACHABLE"
        fi
    done < <(get_peers "$network" "$port")
}

main() {
    local port
    port=$(get_zenoh_port)
    local network
    network=$(get_connect_network)

    # Allow overriding network
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --network) network="$2"; shift 2 ;;
            --port) port="$2"; shift 2 ;;
            -h|--help) echo "Usage: $(basename "$0") [--network <name>] [--port <port>]"; exit 0 ;;
            *) echo "Unknown option: $1"; exit 1 ;;
        esac
    done

    echo -e "${BOLD}=== Zenoh Connectivity Check ===${NC}"
    echo -e "Network: ${CYAN}${network}${NC}  Port: ${CYAN}${port}${NC}"
    echo ""

    local errors=0
    check_local_zenohd || ((errors++)) || true
    check_local_port "$port" || ((errors++)) || true
    check_remote_peers "$network" "$port"

    echo ""
    if [[ $errors -eq 0 ]]; then
        echo -e "${GREEN}Local Zenoh setup looks good.${NC}"
    else
        echo -e "${YELLOW}${errors} local issue(s) detected. See above.${NC}"
    fi
}

main "$@"
