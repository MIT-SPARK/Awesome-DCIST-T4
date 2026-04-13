#!/usr/bin/env bash
# net_diag.sh — Unified diagnostic runner for ADT4 network
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; CYAN='\033[0;36m'; NC='\033[0m'; BOLD='\033[1m'

network=""
skip_ros=false
skip_zenoh=false

while [[ $# -gt 0 ]]; do
    case "$1" in
        --network) network="$2"; shift 2 ;;
        --skip-ros) skip_ros=true; shift ;;
        --skip-zenoh) skip_zenoh=true; shift ;;
        -h|--help)
            echo "Usage: $(basename "$0") [--network silvus|mit_wifi] [--skip-ros] [--skip-zenoh]"
            echo ""
            echo "Runs ping sweep, Zenoh check, and ROS2 status in sequence."
            exit 0
            ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

echo -e "${BOLD}╔══════════════════════════════════════╗${NC}"
echo -e "${BOLD}║   ADT4 Network Diagnostics Report    ║${NC}"
echo -e "${BOLD}╚══════════════════════════════════════╝${NC}"
echo ""
echo -e "Hostname: ${CYAN}$(hostname)${NC}"
echo -e "Date:     ${CYAN}$(date)${NC}"
[[ -n "$network" ]] && echo -e "Network:  ${CYAN}${network}${NC}"
echo ""

# Step 1: Ping sweep
echo -e "${BOLD}━━━ Step 1: Ping Sweep ━━━${NC}"
echo ""
if [[ -n "$network" ]]; then
    bash "${SCRIPT_DIR}/ping_sweep.sh" sweep --network "$network"
else
    bash "${SCRIPT_DIR}/ping_sweep.sh" sweep
fi
echo ""

# Step 2: Zenoh check
if ! $skip_zenoh; then
    echo -e "${BOLD}━━━ Step 2: Zenoh Connectivity ━━━${NC}"
    echo ""
    if [[ -n "$network" ]]; then
        bash "${SCRIPT_DIR}/zenoh_check.sh" --network "$network"
    else
        bash "${SCRIPT_DIR}/zenoh_check.sh"
    fi
    echo ""
fi

# Step 3: ROS2 status
if ! $skip_ros; then
    echo -e "${BOLD}━━━ Step 3: ROS2 Status ━━━${NC}"
    echo ""
    if command -v ros2 &>/dev/null; then
        echo -e "${BOLD}Active nodes:${NC}"
        ros2 node list 2>/dev/null || echo -e "  ${YELLOW}Could not list nodes${NC}"
        echo ""
        echo -e "${BOLD}Active topics:${NC}"
        ros2 topic list 2>/dev/null || echo -e "  ${YELLOW}Could not list topics${NC}"
        echo ""

        # Check connectivity test topics
        echo -e "${BOLD}Connectivity test heartbeats:${NC}"
        if ros2 topic list 2>/dev/null | grep -q "connectivity_test/heartbeat"; then
            ros2 topic list 2>/dev/null | grep "connectivity_test/heartbeat" | while read -r topic; do
                hz=$(timeout 3 ros2 topic hz "$topic" 2>/dev/null | head -1 || echo "no data")
                echo "  $topic: $hz"
            done
        else
            echo -e "  ${YELLOW}No connectivity test heartbeats found${NC}"
        fi
    else
        echo -e "  ${YELLOW}ros2 command not found — is the workspace sourced?${NC}"
        echo "  source \${ADT4_WS}/install/setup.zsh"
    fi
    echo ""
fi

# Summary
echo -e "${BOLD}━━━ Summary ━━━${NC}"
echo -e "Diagnostics complete. For detailed tests:"
echo "  Bandwidth:  $(basename "$0" | sed 's/net_diag/ping_sweep/') bandwidth <ip>"
echo "  MTU:        $(basename "$0" | sed 's/net_diag/ping_sweep/') mtu <ip>"
echo "  SSH:        ${SCRIPT_DIR}/ssh_check.sh"
echo "  Live view:  ${SCRIPT_DIR}/link_monitor.sh"
