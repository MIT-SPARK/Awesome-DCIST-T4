#!/usr/bin/env python3
"""ADT4 Prometheus metrics exporter for Silvus radios, network, and Zenoh."""

import argparse
import json
import os
import socket
import subprocess
import sys
import threading
import time
import urllib.request
from http.server import HTTPServer, BaseHTTPRequestHandler
from pathlib import Path

import yaml

SCRIPT_DIR = Path(__file__).resolve().parent
TOPOLOGY = SCRIPT_DIR / ".." / "config" / "topology.yaml"


class MetricsStore:
    def __init__(self):
        self._lock = threading.Lock()
        self._gauges = {}  # (name, labels_tuple) -> value

    def set_gauge(self, name, value, **labels):
        key = (name, tuple(sorted(labels.items())))
        with self._lock:
            self._gauges[key] = value

    def render(self):
        lines = []
        with self._lock:
            # Group by metric name for TYPE declarations
            by_name = {}
            for (name, labels_tuple), value in sorted(self._gauges.items()):
                by_name.setdefault(name, []).append((labels_tuple, value))

            for name, entries in sorted(by_name.items()):
                lines.append(f"# TYPE {name} gauge")
                for labels_tuple, value in entries:
                    if labels_tuple:
                        label_str = ",".join(
                            f'{k}="{v}"' for k, v in labels_tuple
                        )
                        lines.append(f"{name}{{{label_str}}} {value}")
                    else:
                        lines.append(f"{name} {value}")

        lines.append("")
        return "\n".join(lines)


class MetricsHandler(BaseHTTPRequestHandler):
    store = None

    def do_GET(self):
        if self.path == "/metrics" or self.path == "/":
            body = self.store.render().encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/plain; version=0.0.4")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, format, *args):
        pass  # Suppress access logs


def load_topology():
    with open(TOPOLOGY) as f:
        return yaml.safe_load(f)


def query_radio_api(mgmt_ip, methods):
    url = f"http://{mgmt_ip}/cgi-bin/streamscape_api"
    batch = [
        {"jsonrpc": "2.0", "method": m, "params": [], "id": i + 1}
        for i, m in enumerate(methods)
    ]
    payload = json.dumps(batch).encode()
    req = urllib.request.Request(
        url, data=payload, headers={"Content-Type": "application/json"}
    )
    with urllib.request.urlopen(req, timeout=5) as resp:
        results = json.loads(resp.read())
    out = {}
    for r in results:
        idx = r.get("id", 0) - 1
        if 0 <= idx < len(methods):
            method = methods[idx]
            if "error" not in r:
                out[method] = r["result"]
            else:
                out[method] = None
    return out


def silvus_collector(store, interval):
    """Periodically query Silvus radios and update metrics."""
    while True:
        try:
            topo = load_topology()
            radios = topo.get("silvus_radios", {}).get("radios", {})
            methods = [
                "battery_percent",
                "input_voltage_monitoring",
                "read_current_temperature",
                "routing_tree",
                "nodeid",
            ]
            for name, info in radios.items():
                mgmt_ip = info.get("mgmt_ip", "")
                if not mgmt_ip:
                    continue
                try:
                    data = query_radio_api(mgmt_ip, methods)
                    store.set_gauge(
                        "adt4_radio_reachable", 1, radio=name
                    )

                    bat = data.get("battery_percent")
                    if bat and bat[0] is not None:
                        store.set_gauge(
                            "adt4_radio_battery_percent",
                            float(bat[0]),
                            radio=name,
                        )

                    volt = data.get("input_voltage_monitoring")
                    if volt and volt[0] is not None:
                        store.set_gauge(
                            "adt4_radio_voltage_mv",
                            float(volt[0]),
                            radio=name,
                        )

                    temp = data.get("read_current_temperature")
                    if temp and temp[0] is not None:
                        store.set_gauge(
                            "adt4_radio_temperature_c",
                            float(temp[0]),
                            radio=name,
                        )

                    tree = data.get("routing_tree")
                    if tree is not None:
                        store.set_gauge(
                            "adt4_radio_mesh_nodes",
                            len(tree),
                            radio=name,
                        )
                except Exception:
                    store.set_gauge(
                        "adt4_radio_reachable", 0, radio=name
                    )
        except Exception:
            pass
        time.sleep(interval)


def ping_collector(store, interval):
    """Periodically ping peers and update metrics."""
    while True:
        try:
            topo = load_topology()
            machines = topo.get("machines", {})
            hostname = socket.gethostname()
            for name, info in machines.items():
                if name == hostname:
                    continue
                for network, ip in info.get("addresses", {}).items():
                    try:
                        result = subprocess.run(
                            ["ping", "-c", "1", "-W", "2", ip],
                            capture_output=True,
                            text=True,
                            timeout=5,
                        )
                        if result.returncode == 0:
                            store.set_gauge(
                                "adt4_peer_reachable",
                                1,
                                target=name,
                                network=network,
                            )
                            # Extract RTT
                            for line in result.stdout.split("\n"):
                                if "time=" in line:
                                    rtt = line.split("time=")[1].split()[0]
                                    store.set_gauge(
                                        "adt4_peer_ping_rtt_ms",
                                        float(rtt),
                                        target=name,
                                        network=network,
                                    )
                        else:
                            store.set_gauge(
                                "adt4_peer_reachable",
                                0,
                                target=name,
                                network=network,
                            )
                    except Exception:
                        store.set_gauge(
                            "adt4_peer_reachable",
                            0,
                            target=name,
                            network=network,
                        )
        except Exception:
            pass
        time.sleep(interval)


def zenoh_collector(store, interval, admin_port=8000):
    """Periodically check Zenoh admin space for session info."""
    while True:
        try:
            url = f"http://localhost:{admin_port}/@/router/local/link/**"
            req = urllib.request.Request(url)
            with urllib.request.urlopen(req, timeout=3) as resp:
                data = json.loads(resp.read())

            if data:
                items = data if isinstance(data, list) else [data]
                store.set_gauge("adt4_zenoh_session_count", len(items))
            else:
                store.set_gauge("adt4_zenoh_session_count", 0)

            store.set_gauge("adt4_zenoh_admin_reachable", 1)
        except Exception:
            store.set_gauge("adt4_zenoh_admin_reachable", 0)
            store.set_gauge("adt4_zenoh_session_count", 0)

        # Check if zenohd is running
        try:
            result = subprocess.run(
                ["pgrep", "-f", "rmw_zenohd"],
                capture_output=True,
                timeout=3,
            )
            store.set_gauge(
                "adt4_zenoh_router_running",
                1 if result.returncode == 0 else 0,
            )
        except Exception:
            pass

        time.sleep(interval)


def main():
    parser = argparse.ArgumentParser(
        description="ADT4 Prometheus metrics exporter"
    )
    parser.add_argument(
        "--port", type=int, default=9100, help="HTTP port (default: 9100)"
    )
    parser.add_argument(
        "--interval",
        type=int,
        default=15,
        help="Scrape interval in seconds (default: 15)",
    )
    parser.add_argument(
        "--admin-port",
        type=int,
        default=8000,
        help="Zenoh admin REST port (default: 8000)",
    )
    args = parser.parse_args()

    store = MetricsStore()
    MetricsHandler.store = store

    # Start collector threads
    threading.Thread(
        target=silvus_collector,
        args=(store, args.interval),
        daemon=True,
    ).start()
    threading.Thread(
        target=ping_collector,
        args=(store, args.interval),
        daemon=True,
    ).start()
    threading.Thread(
        target=zenoh_collector,
        args=(store, args.interval, args.admin_port),
        daemon=True,
    ).start()

    server = HTTPServer(("0.0.0.0", args.port), MetricsHandler)
    print(f"ADT4 Prometheus exporter listening on :{args.port}/metrics")
    print(f"Scrape interval: {args.interval}s")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopped.")
        server.server_close()


if __name__ == "__main__":
    main()
