#!/usr/bin/env bash
# collect_logs.sh — Capture diagnostic snapshot to timestamped log file
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TOPOLOGY="${SCRIPT_DIR}/../config/topology.yaml"

RED='\033[0;31m'; GREEN='\033[0;32m'; CYAN='\033[0;36m'; NC='\033[0m'; BOLD='\033[1m'

OUTPUT_DIR="${HOME}/adt4_diagnostics"
NETWORK=""
SKIP_SILVUS=false

while [[ $# -gt 0 ]]; do
    case "$1" in
        --output-dir) OUTPUT_DIR="$2"; shift 2 ;;
        --network) NETWORK="$2"; shift 2 ;;
        --skip-silvus) SKIP_SILVUS=true; shift ;;
        -h|--help)
            echo "Usage: $(basename "$0") [--output-dir <dir>] [--network <name>] [--skip-silvus]"
            echo ""
            echo "Captures a timestamped diagnostic snapshot for post-mortem analysis."
            echo "Default output: ~/adt4_diagnostics/"
            exit 0
            ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

mkdir -p "$OUTPUT_DIR"

TIMESTAMP=$(date '+%Y%m%d_%H%M%S')
HOSTNAME=$(hostname)
LOGFILE="${OUTPUT_DIR}/diag_${HOSTNAME}_${TIMESTAMP}.log"

strip_ansi() {
    sed 's/\x1b\[[0-9;]*m//g'
}

# Helper to run a section and capture output
run_section() {
    local title="$1"
    shift
    echo "============================================" >> "$LOGFILE"
    echo "=== ${title}" >> "$LOGFILE"
    echo "============================================" >> "$LOGFILE"
    echo "" >> "$LOGFILE"
    "$@" 2>&1 | strip_ansi >> "$LOGFILE" || echo "(command failed)" >> "$LOGFILE"
    echo "" >> "$LOGFILE"
}

echo -e "${BOLD}Collecting diagnostics...${NC}"
echo ""

# Header
{
    echo "ADT4 Network Diagnostics Snapshot"
    echo "================================="
    echo "Hostname:  ${HOSTNAME}"
    echo "Date:      $(date)"
    echo "Kernel:    $(uname -r)"
    echo "User:      $(whoami)"
    echo ""
} > "$LOGFILE"

# System info
run_section "IP Addresses" ip -br addr show
run_section "Route Table" ip route show
run_section "Network Interface Stats" ip -s link show

echo -ne "  System info...        "
echo -e "${GREEN}done${NC}"

# Ping sweep
PING_ARGS=("bash" "${SCRIPT_DIR}/ping_sweep.sh" "sweep")
[[ -n "$NETWORK" ]] && PING_ARGS+=("--network" "$NETWORK")
run_section "Ping Sweep" "${PING_ARGS[@]}"
echo -ne "  Ping sweep...         "
echo -e "${GREEN}done${NC}"

# Silvus radio status
if ! $SKIP_SILVUS; then
    run_section "Silvus Radio Status" bash "${SCRIPT_DIR}/silvus_status.sh" status
    run_section "Silvus Radio Config" bash "${SCRIPT_DIR}/silvus_status.sh" radio-config
    run_section "Silvus Config Diff" bash "${SCRIPT_DIR}/silvus_status.sh" config-diff
    run_section "Silvus Topology" bash "${SCRIPT_DIR}/silvus_status.sh" topology
    echo -ne "  Silvus radios...      "
    echo -e "${GREEN}done${NC}"
fi

# Zenoh check
if command -v ros2 &>/dev/null || pgrep -f rmw_zenohd &>/dev/null; then
    ZENOH_ARGS=("bash" "${SCRIPT_DIR}/zenoh_check.sh")
    [[ -n "$NETWORK" ]] && ZENOH_ARGS+=("--network" "$NETWORK")
    run_section "Zenoh Connectivity" "${ZENOH_ARGS[@]}"
    echo -ne "  Zenoh check...        "
    echo -e "${GREEN}done${NC}"
fi

# ROS2 status
if command -v ros2 &>/dev/null; then
    run_section "ROS2 Node List" ros2 node list
    run_section "ROS2 Topic List" ros2 topic list
    run_section "ROS2 Topic Info (connectivity)" bash -c "ros2 topic list 2>/dev/null | grep connectivity_test | while read -r t; do echo \"\$t:\"; ros2 topic info \"\$t\" 2>/dev/null; echo; done"
    echo -ne "  ROS2 status...        "
    echo -e "${GREEN}done${NC}"
else
    {
        echo "============================================"
        echo "=== ROS2 Status"
        echo "============================================"
        echo ""
        echo "ros2 command not available (workspace not sourced)"
        echo ""
    } >> "$LOGFILE"
fi

# SSH check
SSH_ARGS=("bash" "${SCRIPT_DIR}/ssh_check.sh")
[[ -n "$NETWORK" ]] && SSH_ARGS+=("--network" "$NETWORK")
run_section "SSH Access Check" "${SSH_ARGS[@]}" || true
echo -ne "  SSH check...          "
echo -e "${GREEN}done${NC}"

echo ""
echo -e "${GREEN}${BOLD}Diagnostics saved to:${NC} ${LOGFILE}"
echo -e "File size: $(du -h "$LOGFILE" | cut -f1)"
