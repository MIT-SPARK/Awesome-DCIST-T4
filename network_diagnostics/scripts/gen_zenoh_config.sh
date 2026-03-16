#!/usr/bin/env bash
# gen_zenoh_config.sh — Generate Zenoh router config from topology
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TOPOLOGY="${SCRIPT_DIR}/../config/topology.yaml"

usage() {
    echo "Usage: $(basename "$0") --network <silvus|mit_wifi|penn_wifi>"
    echo ""
    echo "Generates a Zenoh router JSON5 config to stdout."
    echo ""
    echo "Example:"
    echo "  $(basename "$0") --network silvus > /tmp/zenoh_config.json5"
    echo "  ZENOH_ROUTER_CONFIG_URI=/tmp/zenoh_config.json5 ros2 run rmw_zenoh_cpp rmw_zenohd"
}

network=""
while [[ $# -gt 0 ]]; do
    case "$1" in
        --network) network="$2"; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Unknown option: $1" >&2; usage >&2; exit 1 ;;
    esac
done

if [[ -z "$network" ]]; then
    usage >&2
    exit 1
fi

python3 -c "
import yaml
import json
import socket

hostname = socket.gethostname()

with open('${TOPOLOGY}') as f:
    data = yaml.safe_load(f)

port = data.get('zenoh', {}).get('port', 7447)
machines = data.get('machines', {})

connect_endpoints = []
for name, info in sorted(machines.items()):
    if name == hostname:
        continue
    addrs = info.get('addresses', {})
    if '${network}' in addrs:
        connect_endpoints.append(f\"tcp/{addrs['${network}']}:{port}\")

# Find our own address on this network
listen_addr = None
for name, info in machines.items():
    if name == hostname:
        addrs = info.get('addresses', {})
        listen_addr = addrs.get('${network}')
        break

listen_endpoints = []
if listen_addr:
    listen_endpoints.append(f'tcp/{listen_addr}:{port}')
else:
    listen_endpoints.append(f'tcp/0.0.0.0:{port}')

config = {
    'mode': 'router',
    'listen': {
        'endpoints': listen_endpoints,
    },
    'connect': {
        'endpoints': connect_endpoints,
    },
    'scouting': {
        'multicast': {
            'enabled': False,
        },
    },
    'adminspace': {
        'enabled': True,
    },
    'plugins': {
        'rest': {
            'http_port': 8000,
        },
    },
}

# Output as JSON5-compatible JSON (JSON is valid JSON5)
print(json.dumps(config, indent=2))
"
