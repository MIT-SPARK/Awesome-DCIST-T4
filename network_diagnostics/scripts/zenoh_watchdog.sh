#!/usr/bin/env bash
# zenoh_watchdog.sh — Monitor Zenoh peer connections, alert on changes
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TOPOLOGY="${SCRIPT_DIR}/../config/topology.yaml"

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; CYAN='\033[0;36m'; NC='\033[0m'; BOLD='\033[1m'; DIM='\033[2m'

INTERVAL=5
ADMIN_PORT=8000

while [[ $# -gt 0 ]]; do
    case "$1" in
        --interval) INTERVAL="$2"; shift 2 ;;
        --admin-port) ADMIN_PORT="$2"; shift 2 ;;
        -h|--help)
            echo "Usage: $(basename "$0") [--interval <secs>] [--admin-port <port>]"
            echo ""
            echo "Monitors Zenoh peer connections and alerts on connect/disconnect."
            echo ""
            echo "Options:"
            echo "  --interval <secs>    Poll interval (default: 5)"
            echo "  --admin-port <port>  Zenoh admin REST port (default: 8000)"
            exit 0
            ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

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

echo -e "${BOLD}=== Zenoh Session Watchdog ===${NC}"
echo -e "Admin port: ${CYAN}${ADMIN_PORT}${NC}  Interval: ${CYAN}${INTERVAL}s${NC}"
echo -e "Press Ctrl+C to stop."
echo ""

# Check admin space is available
if ! curl -s -m 2 "http://localhost:${ADMIN_PORT}/@/router/local" &>/dev/null; then
    echo -e "${RED}Zenoh admin REST API not available on port ${ADMIN_PORT}${NC}"
    echo -e "Start zenohd with admin space enabled, or generate config:"
    echo -e "  ${SCRIPT_DIR}/gen_zenoh_config.sh --network <net> > /tmp/zenoh_config.json5"
    exit 1
fi

trap 'echo -e "\n${NC}Stopped."; exit 0' INT

PREV_PEERS=""

while true; do
    ts=$(date '+%H:%M:%S')

    # Get current sessions from admin space
    CURRENT_PEERS=$(curl -s -m 3 "http://localhost:${ADMIN_PORT}/@/router/local/link/**" 2>/dev/null | python3 -c "
import sys, json
try:
    data = json.load(sys.stdin)
    if not data:
        pass
    else:
        items = data if isinstance(data, list) else [data]
        for item in items:
            value = item.get('value', item)
            dst = value.get('dst', '')
            whatami = value.get('whatami', '')
            if dst:
                print(f'{dst}|{whatami}')
except:
    pass
" 2>/dev/null || true)

    if [[ -n "$PREV_PEERS" ]]; then
        # Compare: find new peers
        while IFS='|' read -r zid whatami; do
            [[ -z "$zid" ]] && continue
            if ! echo "$PREV_PEERS" | grep -q "^${zid}|"; then
                echo -e "[${ts}] ${GREEN}+ CONNECTED${NC}  ${zid} (${whatami})"
            fi
        done <<< "$CURRENT_PEERS"

        # Find disconnected peers
        while IFS='|' read -r zid whatami; do
            [[ -z "$zid" ]] && continue
            if [[ -n "$CURRENT_PEERS" ]]; then
                if ! echo "$CURRENT_PEERS" | grep -q "^${zid}|"; then
                    echo -e "[${ts}] ${RED}- DISCONNECTED${NC}  ${zid} (${whatami})"
                fi
            else
                echo -e "[${ts}] ${RED}- DISCONNECTED${NC}  ${zid} (${whatami})"
            fi
        done <<< "$PREV_PEERS"
    else
        # First run: show current state
        if [[ -n "$CURRENT_PEERS" ]]; then
            echo -e "[${ts}] Current sessions:"
            while IFS='|' read -r zid whatami; do
                [[ -z "$zid" ]] && continue
                echo -e "  ${GREEN}*${NC} ${zid} (${whatami})"
            done <<< "$CURRENT_PEERS"
        else
            echo -e "[${ts}] ${YELLOW}No active sessions${NC}"
        fi
    fi

    # Also check remote Zenoh port reachability periodically
    ZENOH_PORT=$(get_zenoh_port 2>/dev/null || echo "7447")
    NETWORK=$(get_connect_network 2>/dev/null || echo "silvus")
    HOSTNAME=$(hostname)

    while IFS=' ' read -r name ip port; do
        [[ -z "$name" ]] && continue
        [[ "$name" == "$HOSTNAME" ]] && continue
        if ! nc -z -w 2 "$ip" "$port" 2>/dev/null; then
            echo -e "[${ts}] ${YELLOW}! PORT UNREACHABLE${NC}  ${name} (${ip}:${port})"
        fi
    done < <(get_peers "$NETWORK" "$ZENOH_PORT" 2>/dev/null || true)

    PREV_PEERS="$CURRENT_PEERS"
    sleep "$INTERVAL"
done
