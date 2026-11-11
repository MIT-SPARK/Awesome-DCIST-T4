#!/usr/bin/env bash
# preflight.sh — Pre-flight checklist for ADT4 multi-robot system
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TOPOLOGY="${SCRIPT_DIR}/../config/topology.yaml"

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; CYAN='\033[0;36m'; NC='\033[0m'; BOLD='\033[1m'

NETWORK=""
SKIP_SSH=false
SKIP_SILVUS=false
BATTERY_THRESHOLD=20

while [[ $# -gt 0 ]]; do
    case "$1" in
        --network) NETWORK="$2"; shift 2 ;;
        --skip-ssh) SKIP_SSH=true; shift ;;
        --skip-silvus) SKIP_SILVUS=true; shift ;;
        --battery-threshold) BATTERY_THRESHOLD="$2"; shift 2 ;;
        -h|--help)
            echo "Usage: $(basename "$0") [--network <name>] [--skip-ssh] [--skip-silvus] [--battery-threshold <pct>]"
            echo ""
            echo "Pre-flight checklist for ADT4. Returns exit code 0 if all checks pass."
            exit 0
            ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

PASS=0
FAIL=0
WARN=0

check_pass() {
    echo -e "  ${GREEN}[PASS]${NC} $1"
    ((PASS++))
}

check_fail() {
    echo -e "  ${RED}[FAIL]${NC} $1"
    ((FAIL++))
}

check_warn() {
    echo -e "  ${YELLOW}[WARN]${NC} $1"
    ((WARN++))
}

echo -e "${BOLD}╔══════════════════════════════════════╗${NC}"
echo -e "${BOLD}║     ADT4 Pre-Flight Checklist        ║${NC}"
echo -e "${BOLD}╚══════════════════════════════════════╝${NC}"
echo ""
echo -e "Hostname: ${CYAN}$(hostname)${NC}"
echo -e "Date:     ${CYAN}$(date)${NC}"
[[ -n "$NETWORK" ]] && echo -e "Network:  ${CYAN}${NETWORK}${NC}"
echo ""

# --- Check 1: Network interfaces ---
echo -e "${BOLD}1. Network Interfaces${NC}"

detect_active_network() {
    python3 -c "
import yaml, subprocess
with open('${TOPOLOGY}') as f:
    data = yaml.safe_load(f)
ip_out = subprocess.check_output(['ip', 'addr', 'show'], text=True)
found = []
for netname, info in data.get('networks', {}).items():
    prefix = info['subnet'].rsplit('.', 1)[0].rsplit('.', 1)[0]
    if f'inet {prefix}.' in ip_out or f'inet {prefix}/' in ip_out:
        found.append(netname)
print(' '.join(found))
"
}

ACTIVE_NETS=$(detect_active_network 2>/dev/null || echo "")
if [[ -z "$ACTIVE_NETS" ]]; then
    check_fail "No known networks detected"
else
    check_pass "Active networks: ${ACTIVE_NETS}"
fi

# Use specified network or first active
if [[ -z "$NETWORK" ]]; then
    NETWORK=$(echo "$ACTIVE_NETS" | awk '{print $1}')
fi

# --- Check 2: Peer reachability ---
echo -e "${BOLD}2. Peer Reachability (${NETWORK:-none})${NC}"

if [[ -n "$NETWORK" ]]; then
    HOSTNAME=$(hostname)
    python3 -c "
import yaml
with open('${TOPOLOGY}') as f:
    data = yaml.safe_load(f)
machines = data.get('machines', {})
for name, info in sorted(machines.items()):
    addrs = info.get('addresses', {})
    if '${NETWORK}' in addrs:
        print(f\"{name} {addrs['${NETWORK}']} {info.get('role','unknown')}\")
" 2>/dev/null | while IFS=' ' read -r name ip role; do
        [[ -z "$name" ]] && continue
        [[ "$name" == "$HOSTNAME" ]] && continue
        if ping -c 1 -W 2 "$ip" &>/dev/null; then
            check_pass "${name} (${ip}) reachable"
        else
            check_fail "${name} (${ip}) unreachable"
        fi
    done
else
    check_warn "No network specified or detected"
fi

# --- Check 3: Silvus radios ---
if ! $SKIP_SILVUS; then
    echo -e "${BOLD}3. Silvus Radios${NC}"

    python3 -c "
import yaml
with open('${TOPOLOGY}') as f:
    data = yaml.safe_load(f)
radios = data.get('silvus_radios', {}).get('radios', {})
for name, info in sorted(radios.items()):
    mgmt = info.get('mgmt_ip', '')
    if mgmt:
        nid = info.get('node_id', '')
        print(f'{name}|{mgmt}|{nid}')
" 2>/dev/null | while IFS='|' read -r name mgmt_ip node_id; do
        [[ -z "$mgmt_ip" ]] && continue

        if ! ping -c 1 -W 2 "$mgmt_ip" &>/dev/null; then
            check_fail "${name} (${mgmt_ip}) unreachable"
            continue
        fi

        # Check battery
        bat=$(python3 -c "
import urllib.request, json
url = 'http://${mgmt_ip}/cgi-bin/streamscape_api'
batch = [{'jsonrpc': '2.0', 'method': 'battery_percent', 'params': [], 'id': 1}]
try:
    payload = json.dumps(batch).encode()
    req = urllib.request.Request(url, data=payload, headers={'Content-Type': 'application/json'})
    with urllib.request.urlopen(req, timeout=5) as resp:
        results = json.loads(resp.read())
    for r in results:
        if r['id'] == 1 and 'error' not in r:
            print(int(float(r['result'][0])))
except:
    print('ERR')
" 2>/dev/null)

        if [[ "$bat" == "ERR" ]]; then
            check_warn "${name} reachable but API query failed"
        elif [[ "$bat" -le "$BATTERY_THRESHOLD" ]] 2>/dev/null; then
            check_fail "${name} battery LOW: ${bat}% (threshold: ${BATTERY_THRESHOLD}%)"
        else
            check_pass "${name} reachable, battery ${bat}%"
        fi
    done
else
    echo -e "${BOLD}3. Silvus Radios${NC}"
    check_warn "Skipped (--skip-silvus)"
fi

# --- Check 4: Zenoh router ---
echo -e "${BOLD}4. Zenoh Router${NC}"

if pgrep -f "rmw_zenohd" &>/dev/null; then
    pid=$(pgrep -f "rmw_zenohd" | head -1)
    check_pass "rmw_zenohd running (PID: ${pid})"
else
    check_fail "rmw_zenohd not running"
fi

# --- Check 5: Zenoh port ---
echo -e "${BOLD}5. Zenoh Port${NC}"

ZENOH_PORT=$(python3 -c "
import yaml
with open('${TOPOLOGY}') as f:
    data = yaml.safe_load(f)
print(data.get('zenoh', {}).get('port', 7447))
" 2>/dev/null || echo "7447")

if ss -tlnp 2>/dev/null | grep -q ":${ZENOH_PORT} "; then
    check_pass "Port ${ZENOH_PORT} listening"
else
    check_fail "Port ${ZENOH_PORT} not listening"
fi

# --- Check 6: Remote Zenoh ports ---
echo -e "${BOLD}6. Remote Zenoh Peers${NC}"

if [[ -n "$NETWORK" ]]; then
    HOSTNAME=$(hostname)
    python3 -c "
import yaml
with open('${TOPOLOGY}') as f:
    data = yaml.safe_load(f)
machines = data.get('machines', {})
for name, info in sorted(machines.items()):
    addrs = info.get('addresses', {})
    if '${NETWORK}' in addrs:
        print(f\"{name} {addrs['${NETWORK}']}\")
" 2>/dev/null | while IFS=' ' read -r name ip; do
        [[ -z "$name" ]] && continue
        [[ "$name" == "$HOSTNAME" ]] && continue
        if nc -z -w 2 "$ip" "$ZENOH_PORT" 2>/dev/null; then
            check_pass "${name} Zenoh port reachable"
        else
            check_warn "${name} Zenoh port ${ZENOH_PORT} unreachable"
        fi
    done
else
    check_warn "No network — skipping remote peer check"
fi

# --- Check 7: SSH access ---
if ! $SKIP_SSH; then
    echo -e "${BOLD}7. SSH Access${NC}"

    if [[ -n "$NETWORK" ]]; then
        HOSTNAME=$(hostname)
        python3 -c "
import yaml
with open('${TOPOLOGY}') as f:
    data = yaml.safe_load(f)
machines = data.get('machines', {})
ssh = data.get('ssh', {})
bs_user = ssh.get('base_station_user', 'rrg')
robot_user = ssh.get('robot_user', 'swarm')
for name, info in sorted(machines.items()):
    addrs = info.get('addresses', {})
    if '${NETWORK}' in addrs:
        role = info.get('role', 'robot')
        user = bs_user if role == 'base_station' else robot_user
        print(f\"{name} {user} {addrs['${NETWORK}']}\")
" 2>/dev/null | while IFS=' ' read -r name user ip; do
            [[ -z "$name" ]] && continue
            [[ "$name" == "$HOSTNAME" ]] && continue
            if ssh -o ConnectTimeout=3 -o BatchMode=yes "${user}@${ip}" "echo ok" &>/dev/null; then
                check_pass "${name} SSH OK (${user}@${ip})"
            else
                check_warn "${name} SSH failed (${user}@${ip})"
            fi
        done
    else
        check_warn "No network — skipping SSH check"
    fi
else
    echo -e "${BOLD}7. SSH Access${NC}"
    check_warn "Skipped (--skip-ssh)"
fi

# --- Check 8: ROS2 workspace ---
echo -e "${BOLD}8. ROS2 Environment${NC}"

if command -v ros2 &>/dev/null; then
    check_pass "ros2 command available"
else
    check_fail "ros2 not found — source workspace first"
fi

if [[ -n "${ADT4_ROBOT_NAME:-}" ]]; then
    check_pass "ADT4_ROBOT_NAME=${ADT4_ROBOT_NAME}"
else
    check_warn "ADT4_ROBOT_NAME not set"
fi

# --- Summary ---
echo ""
echo -e "${BOLD}━━━ Summary ━━━${NC}"
echo -e "  ${GREEN}PASS: ${PASS}${NC}  ${RED}FAIL: ${FAIL}${NC}  ${YELLOW}WARN: ${WARN}${NC}"
echo ""

if [[ $FAIL -eq 0 ]]; then
    echo -e "${GREEN}${BOLD}ALL CHECKS PASSED — GO FOR LAUNCH${NC}"
    exit 0
else
    echo -e "${RED}${BOLD}PRE-FLIGHT FAILED — ${FAIL} issue(s) to resolve${NC}"
    exit 1
fi
