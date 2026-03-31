"""Shared helpers for fleet-adt4: topology loading, SSH, rsync, and remote queries."""

import base64
import ipaddress
import pathlib
import re
import shlex
import subprocess
from concurrent.futures import ThreadPoolExecutor, as_completed

import yaml

_DEFAULT_TOPOLOGY = (
    pathlib.Path(__file__).resolve().parents[3]
    / "network_diagnostics"
    / "config"
    / "topology.yaml"
)

_DEFAULT_OUTPUT_ROOT = "~/adt4_output"


def _quote_path(path):
    """Quote a path for safe shell interpolation, preserving ~ expansion.

    shlex.quote wraps in single quotes, which prevents ~ and $HOME expansion.
    For paths starting with ~, we emit ~/ unquoted followed by the quoted rest.
    """
    if path.startswith("~/"):
        return "~/" + shlex.quote(path[2:])
    if path == "~":
        return "~"
    return shlex.quote(path)


_local_ips_cache = None

def get_local_ips():
    global _local_ips_cache
    if _local_ips_cache is not None:
        return _local_ips_cache
    _local_ips_cache = ["127.0.0.1"]
    try:
        result = subprocess.run(["ip", "-4", "-o", "addr", "show"], capture_output=True, text=True)
        for line in result.stdout.strip().splitlines():
            parts = line.split()
            for p in parts:
                if "/" in p:
                    try:
                        ipaddress.ip_interface(p)
                        _local_ips_cache.append(p.split("/")[0])
                    except ValueError:
                        pass
    except Exception:
        pass
    return _local_ips_cache


def load_topology(path=None):
    """Parse topology.yaml and return the dict."""
    path = pathlib.Path(path) if path else _DEFAULT_TOPOLOGY
    with open(path) as f:
        return yaml.safe_load(f)


def detect_network(topology):
    """Match local IPs against topology subnets to find the active network."""
    result = subprocess.run(
        ["ip", "-4", "-o", "addr", "show"],
        capture_output=True,
        text=True,
        timeout=5,
    )
    local_ips = []
    for line in result.stdout.strip().splitlines():
        parts = line.split()
        for p in parts:
            if "/" in p:
                try:
                    ipaddress.ip_interface(p)
                    local_ips.append(p.split("/")[0])
                except ValueError:
                    continue

    for net_name, net_info in topology.get("networks", {}).items():
        subnet = ipaddress.ip_network(net_info["subnet"], strict=False)
        for lip in local_ips:
            if ipaddress.ip_address(lip) in subnet:
                return net_name
    return None


def resolve_machines(topology, network, roles=None, names=None):
    """Resolve machines reachable on the given network.

    Returns list of dicts: {name, role, ip, user, desc, platform_id}.
    """
    ssh_conf = topology.get("ssh", {})
    robot_user = ssh_conf.get("robot_user", "swarm")
    base_user = ssh_conf.get("base_station_user", "rrg")

    machines = []
    for mname, minfo in topology.get("machines", {}).items():
        if names and mname not in names:
            continue
        if roles and minfo.get("role") not in roles:
            continue
        addrs = minfo.get("addresses", {})
        ip = addrs.get(network)
        if not ip:
            continue
        user = base_user if minfo.get("role") == "base_station" else robot_user
        machines.append(
            {
                "name": mname,
                "role": minfo.get("role", "unknown"),
                "ip": ip,
                "user": user,
                "desc": minfo.get("desc", ""),
                "platform_id": minfo.get("platform_id", ""),
            }
        )
    return machines


def check_machine(machine, timeout=3):
    """Check if a machine is both pingable and SSH-accessible.

    Returns dict with keys: ping (bool), ssh (bool), online (bool).
    """
    if machine["ip"] in get_local_ips():
        return {"ping": True, "ssh": True, "online": True}

    p = ping_host(machine["ip"], timeout=timeout)
    s = False
    if p:
        rc, _, _ = ssh_cmd(machine["user"], machine["ip"], "hostname", timeout=timeout)
        s = rc == 0
    return {"ping": p, "ssh": s, "online": p and s}


def filter_reachable(machines, max_workers=8, quiet=False):
    """Check reachability of machines and return (reachable, unreachable) lists.

    Each machine dict gets 'ping', 'ssh', 'online' keys added.
    """
    def check(m):
        result = check_machine(m)
        m.update(result)
        return m

    results = run_parallel(check, machines, max_workers=max_workers)
    reachable = []
    unreachable = []
    for m, _ in results:
        if m.get("online"):
            reachable.append(m)
        else:
            unreachable.append(m)
    return reachable, unreachable


def ping_host(ip, timeout=2):
    """Single-packet ping, returns True if reachable."""
    try:
        result = subprocess.run(
            ["ping", "-c", "1", "-W", str(timeout), ip],
            capture_output=True,
            timeout=timeout + 2,
        )
        return result.returncode == 0
    except (subprocess.TimeoutExpired, OSError):
        return False


def ssh_cmd(user, ip, cmd, timeout=5):
    """Run a command via SSH (or locally if ip is local). Returns (returncode, stdout, stderr)."""
    if ip in get_local_ips():
        try:
            result = subprocess.run(
                ["bash", "-c", cmd],
                capture_output=True,
                text=True,
                timeout=timeout + 10,
            )
            return result.returncode, result.stdout.strip(), result.stderr.strip()
        except subprocess.TimeoutExpired:
            return -1, "", "Local command timeout"
        except OSError as e:
            return -1, "", str(e)

    ssh_args = [
        "ssh",
        "-o", "ConnectTimeout=" + str(timeout),
        "-o", "BatchMode=yes",
        "-o", "StrictHostKeyChecking=no",
        f"{user}@{ip}",
        cmd,
    ]
    try:
        result = subprocess.run(
            ssh_args,
            capture_output=True,
            text=True,
            timeout=timeout + 10,
        )
        return result.returncode, result.stdout.strip(), result.stderr.strip()
    except subprocess.TimeoutExpired:
        return -1, "", "SSH timeout"
    except OSError as e:
        return -1, "", str(e)


def _run_rsync(cmd, stream=False, timeout=3600, progress_callback=None):
    """Run an rsync command. If stream=True, print output live.

    progress_callback: optional callable(percent: int) called on each progress update.
    """
    if not stream and not progress_callback:
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
            return result.returncode, result.stdout, result.stderr
        except subprocess.TimeoutExpired:
            return -1, "", "rsync timeout"
        except OSError as e:
            return -1, "", str(e)

    try:
        proc = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
        )
        last_line = ""
        for line in proc.stdout:
            line = line.rstrip()
            if not line:
                continue
            if "%" in line:
                if stream:
                    print(f"\r    {line}", end="", flush=True)
                last_line = line
                if progress_callback:
                    m = re.search(r"(\d+)%", line)
                    if m:
                        progress_callback(int(m.group(1)))
        if last_line and stream:
            print()  # newline after progress
        try:
            proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            proc.kill()
            return -1, "", "rsync timeout"
        stderr = proc.stderr.read() if proc.stderr else ""
        return proc.returncode, "", stderr
    except OSError as e:
        return -1, "", str(e)


def rsync_transfer(
    src_user,
    src_ip,
    src_path,
    dst_user,
    dst_ip,
    dst_path,
    delete=False,
    relay=False,
    relay_tmp="/tmp/adt4_transfer",
    stream=False,
    exclude=None,
    progress_callback=None,
):
    """Rsync files between machines.

    Tries direct transfer first; falls back to relay through operator machine.
    If stream=True, print rsync progress to stdout in real time.
    progress_callback: optional callable(percent: int) for progress updates.
    exclude: list of rsync --exclude patterns.
    Returns (success: bool, method: str, message: str).
    """
    rsync_flags = ["-az", "--info=progress2"]
    if delete:
        rsync_flags.append("--delete")
    for pat in (exclude or []):
        rsync_flags += ["--exclude", pat]

    src_remote = f"{src_user}@{src_ip}:{src_path}"
    dst_remote = f"{dst_user}@{dst_ip}:{dst_path}"

    if not relay:
        # Try direct robot-to-robot
        cmd = ["rsync"] + rsync_flags + ["-e", "ssh -A", src_remote, dst_remote]
        rc, out, err = _run_rsync(cmd, stream=stream, progress_callback=progress_callback)
        if rc == 0:
            return True, "direct", out

    # Fallback: relay through operator
    local_tmp = pathlib.Path(relay_tmp) / pathlib.Path(src_path).name
    local_tmp.mkdir(parents=True, exist_ok=True)

    # For relay, split progress: 0-50% for fetch, 50-100% for push
    def relay_fetch_cb(pct):
        if progress_callback:
            progress_callback(pct // 2)

    def relay_push_cb(pct):
        if progress_callback:
            progress_callback(50 + pct // 2)

    # Step 1: source -> operator
    if stream:
        print("    Fetching to local relay...")
    cmd1 = ["rsync"] + rsync_flags + [src_remote + "/", str(local_tmp) + "/"]
    rc1, _, err1 = _run_rsync(cmd1, stream=stream, progress_callback=relay_fetch_cb)
    if rc1 != 0:
        return False, "relay", f"Fetch failed: {err1}"

    # Step 2: operator -> destination
    if stream:
        print("    Pushing from local relay...")
    cmd2 = ["rsync"] + rsync_flags + [str(local_tmp) + "/", dst_remote + "/"]
    rc2, _, err2 = _run_rsync(cmd2, stream=stream, progress_callback=relay_push_cb)
    if rc2 != 0:
        return False, "relay", f"Push failed: {err2}"

    return True, "relay", "Transfer complete via relay"


def run_parallel(fn, items, max_workers=8):
    """Run fn(item) in parallel for each item. Returns list of (item, result)."""
    results = []
    with ThreadPoolExecutor(max_workers=max_workers) as pool:
        futures = {pool.submit(fn, item): item for item in items}
        for future in as_completed(futures):
            item = futures[future]
            try:
                result = future.result()
            except Exception as e:
                result = e
            results.append((item, result))
    return results


def list_remote_experiments(user, ip, output_root=_DEFAULT_OUTPUT_ROOT):
    """List experiment directories on a remote machine.

    Returns list of {name, subdirs: [str], size: str} or empty list on failure.
    """
    # Get experiment dirs and their immediate subdirs + sizes
    # Use _quote_path to handle ~ expansion on the remote side
    q_root = _quote_path(output_root)
    cmd = (
        f"if [ -d {q_root} ]; then "
        f"  cd {q_root} && "
        f"  for d in */; do "
        f'    [ -d "$d" ] || continue; '
        f'    name="${{d%/}}"; '
        f'    subdirs=$(ls -1d "$d"*/ 2>/dev/null | xargs -I{{}} basename {{}} | tr "\\n" ","); '
        f'    size=$(du -sh "$d" 2>/dev/null | cut -f1); '
        f'    echo "$name|$subdirs|$size"; '
        f"  done; "
        f"fi"
    )
    rc, stdout, _ = ssh_cmd(user, ip, cmd, timeout=15)
    if rc != 0 or not stdout:
        return []

    experiments = []
    for line in stdout.splitlines():
        parts = line.split("|", 2)
        if len(parts) < 3:
            continue
        name = parts[0]
        subdirs = [s for s in parts[1].rstrip(",").split(",") if s]
        size = parts[2].strip()
        experiments.append({"name": name, "subdirs": subdirs, "size": size})
    return experiments


def hash_remote_experiment(user, ip, output_root, experiment):
    """Compute md5sums of map files in a remote experiment directory.

    Returns dict of {relative_path: md5hash} or empty dict on failure.
    """
    exp_path = f"{output_root}/{experiment}"
    q_path = _quote_path(exp_path)
    cmd = (
        f"if [ -d {q_path} ]; then "
        f"  cd {q_path} && "
        f"  find . -type f \\( -name '*.json' -o -name '*.pkl' -o -name '*.sparkdsg' "
        f"    -o -name '*.ply' -o -name '*.csv' -o -name '*.bson' \\) "
        f"    -exec md5sum {{}} + 2>/dev/null | sort; "
        f"fi"
    )
    rc, stdout, _ = ssh_cmd(user, ip, cmd, timeout=30)
    if rc != 0 or not stdout:
        return {}

    hashes = {}
    for line in stdout.splitlines():
        parts = line.split(None, 1)
        if len(parts) == 2:
            hashes[parts[1]] = parts[0]
    return hashes


def get_remote_status(user, ip, silvus_ips=None):
    """Get system status from a remote machine.

    Returns dict with keys: tmux_sessions, ros2_procs, load, disk, radio.
    """
    cmd = (
        "echo '---TMUX---'; tmux list-sessions 2>/dev/null || echo 'none'; "
        "echo '---ROS2---'; pgrep -c -f 'ros2|zenoh' 2>/dev/null || echo '0'; "
        "echo '---LOAD---'; uptime | sed 's/.*load average: //'; "
        "echo '---MEM---'; free -h 2>/dev/null | awk '/^Mem:/{print $3 \"/\" $2}' || echo 'N/A'; "
        "echo '---GPU---'; nvidia-smi --query-gpu=utilization.gpu,memory.used,memory.total --format=csv,noheader,nounits 2>/dev/null | head -1 || echo 'N/A'; "
        "echo '---DISK---'; df -h ~ 2>/dev/null | tail -1 | awk '{print $4 \" / \" $2 \" (\" $5 \" used)\"}'; "
        "echo '---BAT---'; cat /sys/class/power_supply/BAT*/capacity 2>/dev/null && cat /sys/class/power_supply/BAT*/status 2>/dev/null || echo 'N/A'"
    )

    if silvus_ips:
        # Inject Python script to query Silvus radio JSON-RPC
        py_script = """
import urllib.request, json, time, sys, glob
mgmt_ips = sys.argv[1:]
local_mgmt = None
# Check cache first
for f in glob.glob('/home/*/.cache/silvus_diagnostics/*.radio'):
    try:
        with open(f) as fp:
            local_mgmt = fp.read().strip().split('|')[1]
            break
    except: pass
if not local_mgmt and mgmt_ips:
    best_t = 999
    for ip in mgmt_ips:
        try:
            t0 = time.time()
            urllib.request.urlopen(f'http://{ip}/cgi-bin/streamscape_api', timeout=0.2)
            dt = time.time()-t0
            if dt < best_t: best_t, local_mgmt = dt, ip
        except: pass
if local_mgmt:
    try:
        url = f'http://{local_mgmt}/cgi-bin/streamscape_api'
        batch = [
            {'jsonrpc':'2.0', 'method':'battery_percent', 'id':1},
            {'jsonrpc':'2.0', 'method':'routing_tree', 'id':2}
        ]
        req = urllib.request.Request(url, data=json.dumps(batch).encode(), headers={'Content-Type':'application/json'})
        with urllib.request.urlopen(req, timeout=2) as resp:
            data = {r['id']: r.get('result') for r in json.loads(resp.read())}
            bat = data.get(1, ['?'])[0]
            if bat and bat != '?':
                bat = f"{float(bat):.0f}%"
            else:
                bat = '?'
            mesh = len(data.get(2) or [])
            print(f"Bat: {bat}, Mesh: {mesh}")
    except Exception as e:
        print('API Error')
else:
    print('N/A')
"""
        encoded = base64.b64encode(py_script.encode()).decode()
        args = " ".join(silvus_ips)
        cmd += f"; echo '---SILVUS---'; echo '{encoded}' | base64 -d | python3 - {args}"

    rc, stdout, _ = ssh_cmd(user, ip, cmd, timeout=10)
    if rc != 0 or not stdout:
        return {"tmux_sessions": "N/A", "ros2_procs": "N/A", "load": "N/A", "mem": "N/A", "gpu": "N/A", "disk": "N/A", "battery": "N/A", "radio": "N/A"}

    sections = {}
    current = None
    for line in stdout.splitlines():
        if line.startswith("---") and line.endswith("---"):
            current = line.strip("-")
            sections[current] = []
        elif current:
            sections[current].append(line)

    tmux_lines = sections.get("TMUX", ["none"])
    if tmux_lines[0] == "none":
        tmux_sessions = "none"
    else:
        # Extract just session names (before the colon)
        names = [l.split(":")[0] for l in tmux_lines if l.strip()]
        tmux_sessions = ", ".join(names) if names else "none"

    # Format load averages (column header has the labels)
    raw_load = (sections.get("LOAD", ["N/A"])[0] or "N/A").strip()
    if raw_load != "N/A":
        parts = [p.strip() for p in raw_load.split(",")]
        if len(parts) == 3:
            raw_load = f"{parts[0]}/{parts[1]}/{parts[2]}"

    # Format GPU info
    raw_gpu = (sections.get("GPU", ["N/A"])[0] or "N/A").strip()
    if raw_gpu != "N/A":
        gpu_parts = [p.strip() for p in raw_gpu.split(",")]
        if len(gpu_parts) == 3:
            raw_gpu = f"{gpu_parts[0]}% {gpu_parts[1]}/{gpu_parts[2]}M"

    # Format battery: lines are [capacity, status] e.g. ["85", "Discharging"]
    bat_lines = sections.get("BAT", ["N/A"])
    if bat_lines and bat_lines[0] != "N/A" and len(bat_lines) >= 2:
        bat_pct = bat_lines[0].strip()
        bat_state = bat_lines[1].strip()
        raw_bat = f"{bat_pct}% ({bat_state})"
    else:
        raw_bat = "N/A"

    return {
        "tmux_sessions": tmux_sessions,
        "ros2_procs": (sections.get("ROS2", ["0"])[0] or "0"),
        "load": raw_load,
        "mem": (sections.get("MEM", ["N/A"])[0] or "N/A").strip(),
        "gpu": raw_gpu,
        "disk": (sections.get("DISK", ["N/A"])[0] or "N/A"),
        "battery": raw_bat,
        "radio": (sections.get("SILVUS", ["N/A"])[0] or "N/A"),
    }


_ZENOH_CONFIG_FILE = "DEFAULT_RMW_ZENOH_ROUTER_CONFIG.json5"


def generate_zenoh_endpoints(topology, network, machine_name, robot_filter=None):
    """Compute the connect endpoints for a machine based on its role.

    Base stations get connect endpoints to robots on the network (star topology).
    Robots get no connect endpoints — base stations connect to them.
    If robot_filter is set (list of robot names), only those robots are included.
    Returns list of endpoint strings, e.g. ["tcp/192.168.100.3:7447"].
    """
    machines = topology.get("machines", {})
    zenoh_conf = topology.get("zenoh", {})
    port = zenoh_conf.get("port", 7447)

    info = machines[machine_name]
    is_base = info.get("role") == "base_station"

    connect_endpoints = []
    if is_base:
        # Base stations connect to robots (star topology)
        for name, m in sorted(machines.items()):
            if name == machine_name:
                continue
            if m.get("role") != "robot":
                continue
            if robot_filter and name not in robot_filter:
                continue
            addrs = m.get("addresses", {})
            if network in addrs:
                connect_endpoints.append(f"tcp/{addrs[network]}:{port}")

    return connect_endpoints


def deploy_zenoh_config(user, ip, endpoints):
    """Update connect.endpoints in the remote zenoh JSON5 config.

    Backs up existing config to .bak, then uses a Python snippet on the remote
    machine to surgically replace only the connect.endpoints array in the JSON5
    file, preserving all other settings (ROS transport, scouting, etc.).

    Returns (success: bool, message: str).
    """
    config_file = f"~/{_ZENOH_CONFIG_FILE}"
    q_file = _quote_path(config_file)

    # Back up existing config
    backup_cmd = f"[ -f {q_file} ] && cp {q_file} {q_file}.bak || true"
    ssh_cmd(user, ip, backup_cmd, timeout=5)

    # Check if config file exists
    rc, _, _ = ssh_cmd(user, ip, f"[ -f {q_file} ] && echo exists", timeout=5)
    if rc != 0:
        return False, f"No {_ZENOH_CONFIG_FILE} found"

    # Build the new endpoints array string for JSON5
    if endpoints:
        ep_lines = ",\n".join(f'      "{ep}"' for ep in endpoints)
        new_array = f"[\n{ep_lines}\n    ]"
    else:
        new_array = "[]"

    # Use a Python script on the remote to do the surgical replacement.
    # This finds the endpoints array inside the connect section and replaces it.
    # Must skip comment lines (/// ...) that also contain 'endpoints:'.
    py_script = r"""
import re, sys, os

config_path = os.path.expanduser(sys.argv[1])
new_endpoints = sys.argv[2]

with open(config_path) as f:
    content = f.read()

# Find the top-level connect section (not inside a comment)
connect_match = re.search(r'^\s*connect\s*:\s*\{', content, re.MULTILINE)
if not connect_match:
    print("ERROR: Could not find connect section", file=sys.stderr)
    sys.exit(1)

# Find the matching closing } for the connect section by counting braces
search_start = connect_match.start()
depth = 0
connect_end = len(content)
for i in range(connect_match.end() - 1, len(content)):
    if content[i] == '{':
        depth += 1
    elif content[i] == '}':
        depth -= 1
        if depth == 0:
            connect_end = i + 1
            break

connect_section = content[search_start:connect_end]

# Find the actual endpoints: [...] — skip lines that are comments (start with //)
# Look for 'endpoints:' on a line that does NOT start with // or ///
ep_pattern = re.compile(r'^(\s*endpoints\s*:\s*)\[(.*?)\]', re.MULTILINE | re.DOTALL)
ep_match = None
for m in ep_pattern.finditer(connect_section):
    # Check if this line is a comment
    line_start = connect_section.rfind('\n', 0, m.start()) + 1
    line_prefix = connect_section[line_start:m.start()].strip()
    if line_prefix.startswith('//'):
        continue
    ep_match = m
    break

if not ep_match:
    print("ERROR: Could not find endpoints in connect section", file=sys.stderr)
    sys.exit(1)

# Build replacement
new_full = ep_match.group(1) + new_endpoints

# Replace in original content (using absolute position)
abs_start = search_start + ep_match.start()
abs_end = search_start + ep_match.end()
new_content = content[:abs_start] + new_full + content[abs_end:]

with open(config_path, 'w') as f:
    f.write(new_content)

print("OK")
"""
    import base64
    encoded_script = base64.b64encode(py_script.encode()).decode()
    # Escape the new_array for shell (base64 encode it too)
    encoded_array = base64.b64encode(new_array.encode()).decode()

    remote_cmd = (
        f"echo '{encoded_script}' | base64 -d > /tmp/_zenoh_update.py && "
        f"python3 /tmp/_zenoh_update.py {q_file} "
        f"\"$(echo '{encoded_array}' | base64 -d)\" && "
        f"rm -f /tmp/_zenoh_update.py"
    )
    rc, stdout, err = ssh_cmd(user, ip, remote_cmd, timeout=15)
    if rc == 0 and "OK" in stdout:
        n = len(endpoints)
        return True, f"Updated ({n} endpoint{'s' if n != 1 else ''})"
    return False, f"Failed: {err or stdout}"


_ZENOH_TEMPLATE = pathlib.Path(__file__).resolve().parent.parent.parent / "config" / "zenoh_router_template.json5"


def patch_zenoh_config_local(endpoints, listen_port=7447, template_path=None):
    """Create a patched zenoh config from the bundled template.

    Surgically replaces connect.endpoints and listen port, preserving all
    ROS transport/timeout/routing settings from the template.

    Returns the path to the temporary config file, or None on failure.
    """
    template_path = pathlib.Path(template_path) if template_path else _ZENOH_TEMPLATE
    if not template_path.exists():
        return None

    content = template_path.read_text()

    # Build the new endpoints array string
    if endpoints:
        ep_lines = ",\n".join(f'      "{ep}"' for ep in endpoints)
        new_array = f"[\n{ep_lines}\n    ]"
    else:
        new_array = "[]"

    # Patch connect.endpoints using the same regex approach as deploy_zenoh_config
    connect_match = re.search(r'^\s*connect\s*:\s*\{', content, re.MULTILINE)
    if not connect_match:
        return None

    # Find the matching closing } for the connect section
    depth = 0
    connect_end = len(content)
    for i in range(connect_match.end() - 1, len(content)):
        if content[i] == '{':
            depth += 1
        elif content[i] == '}':
            depth -= 1
            if depth == 0:
                connect_end = i + 1
                break

    connect_section = content[connect_match.start():connect_end]

    # Find the actual endpoints: [...] — skip comment lines
    ep_pattern = re.compile(r'^(\s*endpoints\s*:\s*)\[(.*?)\]', re.MULTILINE | re.DOTALL)
    ep_match = None
    for m in ep_pattern.finditer(connect_section):
        line_start = connect_section.rfind('\n', 0, m.start()) + 1
        line_prefix = connect_section[line_start:m.start()].strip()
        if line_prefix.startswith('//'):
            continue
        ep_match = m
        break

    if not ep_match:
        return None

    abs_start = connect_match.start() + ep_match.start()
    abs_end = connect_match.start() + ep_match.end()
    content = content[:abs_start] + ep_match.group(1) + new_array + content[abs_end:]

    # Patch listen port: replace the listen endpoints array
    listen_match = re.search(r'^\s*listen\s*:\s*\{', content, re.MULTILINE)
    if listen_match:
        depth = 0
        listen_end = len(content)
        for i in range(listen_match.end() - 1, len(content)):
            if content[i] == '{':
                depth += 1
            elif content[i] == '}':
                depth -= 1
                if depth == 0:
                    listen_end = i + 1
                    break

        listen_section = content[listen_match.start():listen_end]
        lep_match = None
        for m in ep_pattern.finditer(listen_section):
            line_start = listen_section.rfind('\n', 0, m.start()) + 1
            line_prefix = listen_section[line_start:m.start()].strip()
            if line_prefix.startswith('//'):
                continue
            lep_match = m
            break

        if lep_match:
            new_listen = f'[\n      "tcp/[::]:{listen_port}"\n    ]'
            labs_start = listen_match.start() + lep_match.start()
            labs_end = listen_match.start() + lep_match.end()
            content = content[:labs_start] + lep_match.group(1) + new_listen + content[labs_end:]

    import tempfile
    tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix=".json5", prefix="fleet_zenoh_", delete=False,
    )
    tmp.write(content)
    tmp.flush()
    tmp.close()
    return tmp.name


def send_tmux_keys(user, ip, session="adt4_system", target="core.2", keys="Enter"):
    """Send keys to a tmux pane on a remote machine via SSH.

    Returns (success: bool, message: str).
    """
    safe_session = shlex.quote(session)
    safe_target = shlex.quote(f"{session}:{target}")
    cmd = f"tmux send-keys -t {safe_target} {keys}"
    rc, stdout, err = ssh_cmd(user, ip, cmd, timeout=5)
    if rc == 0:
        return True, "Keys sent"
    return False, f"Failed: {err or stdout}"


def check_zenoh_config(user, ip):
    """Check if zenoh config file exists on remote machine."""
    cmd = f"[ -f ~/{_ZENOH_CONFIG_FILE} ] && echo OK"
    rc, out, _ = ssh_cmd(user, ip, cmd, timeout=5)
    return "OK" in out


def get_ros_node_status(timeout=5):
    """Echo the global StatusTable topic and parse node statuses.

    Runs `ros2 topic echo` locally (base station must have ROS + zenoh).
    Collects messages for `timeout` seconds to gather status from all robots.
    Returns dict: {robot_name: [{nickname, status, notes, required}, ...]}

    Status values: 1=NOMINAL, 2=WARNING, 3=ERROR, 4=NO_HB, 5=STARTUP
    """
    cmd = [
        "ros2", "topic", "echo",
        "/global/ros_system_monitor/table_in",
        "ros_system_monitor_msgs/msg/StatusTable",
    ]
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired as e:
        # Expected: we collect messages until timeout
        stdout = e.stdout if e.stdout else ""
        if isinstance(stdout, bytes):
            stdout = stdout.decode("utf-8", errors="replace")
        if not stdout.strip():
            return {}
    except OSError:
        return {}
    else:
        stdout = result.stdout
        if not stdout.strip():
            return {}

    # ros2 topic echo outputs multiple YAML documents separated by ---
    robots = {}
    for doc in stdout.split("---"):
        doc = doc.strip()
        if not doc:
            continue
        try:
            data = yaml.safe_load(doc)
        except Exception:
            continue
        if not data or "status" not in data:
            continue

        monitor_name = data.get("monitor_name", "unknown")

        for node_info in data.get("status", []):
            nickname = node_info.get("nickname", "")
            parts = nickname.split("/", 1)
            if len(parts) == 2:
                robot_name, node_name = parts
            else:
                robot_name = monitor_name
                node_name = nickname

            if robot_name not in robots:
                robots[robot_name] = []

            # Update existing node or append new one
            existing = next(
                (n for n in robots[robot_name] if n["nickname"] == node_name),
                None,
            )
            if existing:
                existing["status"] = node_info.get("status", 5)
                existing["notes"] = node_info.get("notes", "")
            else:
                robots[robot_name].append({
                    "nickname": node_name,
                    "status": node_info.get("status", 5),
                    "notes": node_info.get("notes", ""),
                    "required": node_info.get("required", True),
                })

    return robots


# Nodes that are acceptable to be non-NOMINAL for indoor tests
_INDOOR_OPTIONAL_NODES = {"gps_monitor", "ntrip_monitor"}

# Status constants matching NodeInfoMsg
_STATUS_NOMINAL = 1
_STATUS_WARNING = 2
_STATUS_ERROR = 3
_STATUS_NO_HB = 4
_STATUS_STARTUP = 5

_STATUS_NAMES = {
    _STATUS_NOMINAL: "NOMINAL",
    _STATUS_WARNING: "WARNING",
    _STATUS_ERROR: "ERROR",
    _STATUS_NO_HB: "NO_HB",
    _STATUS_STARTUP: "STARTUP",
}

_STATUS_COLORS = {
    _STATUS_NOMINAL: "green",
    _STATUS_WARNING: "yellow",
    _STATUS_ERROR: "red",
    _STATUS_NO_HB: "yellow",
    _STATUS_STARTUP: "yellow",
}


def compute_robot_readiness(nodes):
    """Compute overall readiness for a robot from its node statuses.

    Returns (color: str, label: str) where color is green/yellow/red.
    - Green: all nodes NOMINAL
    - Yellow: all NOMINAL except GPS/NTRIP (indoor test acceptable)
    - Red: any non-GPS/NTRIP node is ERROR or NO_HB
    """
    if not nodes:
        return "red", "No data"

    all_nominal = True
    non_optional_ok = True

    for node in nodes:
        status = node["status"]
        is_optional = node["nickname"] in _INDOOR_OPTIONAL_NODES
        if status != _STATUS_NOMINAL:
            all_nominal = False
            if not is_optional:
                non_optional_ok = False

    if all_nominal:
        return "green", "Ready"
    elif non_optional_ok:
        return "yellow", "Ready (indoor)"
    else:
        return "red", "Not ready"


# Topics in dcist.rviz that need robot namespace prefixing.
# These are relative topics (no leading /) that appear in the rviz config.
_RVIZ_TOPICS_TO_NAMESPACE = [
    "hydra_visualizer/graph",
    "hydra_multi_visualizer/graph",
    "frontright/semantic_overlay/image_raw",
    "robot_description",
]

_DEFAULT_RVIZ_TEMPLATE = (
    pathlib.Path(__file__).resolve().parents[3]
    / "dcist_launch_system"
    / "rviz"
    / "dcist.rviz"
)


def generate_namespaced_rviz(robot_name, template_path=None, output_path=None):
    """Generate an rviz config with topics prefixed by robot namespace.

    Reads the template .rviz file, replaces known relative topic values
    with namespaced versions (e.g., hydra_visualizer/graph -> robot/hydra_visualizer/graph).
    Also replaces hardcoded TF frame prefixes like 'euclid/' with the target robot name.

    Returns the output path.
    """
    template_path = pathlib.Path(template_path) if template_path else _DEFAULT_RVIZ_TEMPLATE
    if output_path is None:
        output_path = pathlib.Path(f"/tmp/dcist_{robot_name}.rviz")

    with open(template_path) as f:
        content = f.read()

    # Replace known relative topic values with namespaced versions.
    # In RVIZ YAML, topics appear as: "Value: hydra_visualizer/graph"
    for topic in _RVIZ_TOPICS_TO_NAMESPACE:
        # Match "Value: topic" in the YAML (handles leading whitespace)
        content = content.replace(
            f"Value: {topic}",
            f"Value: {robot_name}/{topic}",
        )

    # Replace hardcoded robot namespace in TF frames (e.g., euclid/ -> hamilton/)
    # Find the existing robot name by looking for known patterns like "XXX/base_link"
    existing_robot = None
    for line in content.splitlines():
        stripped = line.strip()
        if stripped.endswith("/base_link:") or stripped.endswith("/body:"):
            candidate = stripped.split("/")[0]
            if candidate and candidate != robot_name:
                existing_robot = candidate
                break

    if existing_robot and existing_robot != robot_name:
        content = content.replace(f"{existing_robot}/", f"{robot_name}/")
        content = content.replace(f"{existing_robot}_", f"{robot_name}_")

    with open(output_path, "w") as f:
        f.write(content)

    return str(output_path)


def check_silvus_route(user=None, ip=None, mgmt_subnet="172.20.0.0/16"):
    """Check if a route to the Silvus management subnet exists.

    If user/ip are None, checks locally. Otherwise checks via SSH.
    Returns dict: {has_route: bool, interface: str or None, silvus_iface: str or None}
    """
    # Find the silvus data interface (192.168.100.x) and check for mgmt route
    cmd = (
        "iface=$(ip -4 -o addr show | grep '192\\.168\\.100\\.' | awk '{print $2}' | head -1); "
        f"route=$(ip route show {mgmt_subnet} 2>/dev/null | head -1); "
        "echo \"$iface|$route\""
    )
    if user and ip:
        rc, out, _ = ssh_cmd(user, ip, cmd, timeout=5)
    else:
        rc = 0
        try:
            result = subprocess.run(
                ["bash", "-c", cmd], capture_output=True, text=True, timeout=5
            )
            out = result.stdout.strip()
            rc = result.returncode
        except Exception:
            return {"has_route": False, "interface": None, "silvus_iface": None}

    if rc != 0 or not out:
        return {"has_route": False, "interface": None, "silvus_iface": None}

    parts = out.strip().split("|", 1)
    silvus_iface = parts[0].strip() if parts[0].strip() else None
    route_line = parts[1].strip() if len(parts) > 1 else ""
    has_route = bool(route_line)

    return {
        "has_route": has_route,
        "interface": silvus_iface,
        "silvus_iface": silvus_iface,
        "route_info": route_line if route_line else None,
    }


def add_silvus_route(user=None, ip=None, mgmt_subnet="172.20.0.0/16", interface=None):
    """Add a route to the Silvus management subnet via the Silvus data interface.

    If user/ip are None, runs locally. Otherwise runs via SSH.
    Requires sudo. Returns (success: bool, message: str).
    """
    if not interface:
        info = check_silvus_route(user, ip, mgmt_subnet)
        interface = info.get("silvus_iface")
        if not interface:
            return False, "No Silvus interface found (no 192.168.100.x address)"
        if info["has_route"]:
            return True, f"Route already exists: {info['route_info']}"

    cmd = f"sudo ip route add {mgmt_subnet} dev {shlex.quote(interface)}"
    if user and ip:
        rc, out, err = ssh_cmd(user, ip, cmd, timeout=10)
    else:
        try:
            result = subprocess.run(
                ["bash", "-c", cmd], capture_output=True, text=True, timeout=10
            )
            rc, out, err = result.returncode, result.stdout.strip(), result.stderr.strip()
        except Exception as e:
            return False, str(e)

    if rc == 0:
        return True, f"Route added: {mgmt_subnet} dev {interface}"
    elif "File exists" in err:
        return True, f"Route already exists on {interface}"
    else:
        return False, f"Failed (rc={rc}): {err}"


def check_zenoh_port(ip, port=7447, timeout=2):
    """Check if zenoh port is reachable on a remote machine via TCP connect.

    Returns True if the port is open, False otherwise.
    """
    import socket
    try:
        with socket.create_connection((ip, port), timeout=timeout):
            return True
    except (socket.timeout, ConnectionRefusedError, OSError):
        return False


def get_silvus_link_quality(user, ip, silvus_mgmt_ips):
    """Query Silvus radio for link quality metrics (RSSI, SNR, neighbors).

    Returns dict with keys: battery, mesh_nodes, neighbors (list of
    {node_id, rssi, snr}), or None on failure.
    """
    if not silvus_mgmt_ips:
        return None

    py_script = """
import urllib.request, json, sys, glob, math
mgmt_ips = sys.argv[1:]
local_mgmt = None
for f in glob.glob('/home/*/.cache/silvus_diagnostics/*.radio'):
    try:
        with open(f) as fp:
            local_mgmt = fp.read().strip().split('|')[1]
            break
    except: pass
if not local_mgmt and mgmt_ips:
    import time
    best_t = 999
    for mip in mgmt_ips:
        try:
            t0 = time.time()
            urllib.request.urlopen(f'http://{mip}/cgi-bin/streamscape_api', timeout=0.3)
            dt = time.time()-t0
            if dt < best_t: best_t, local_mgmt = dt, mip
        except: pass
if not local_mgmt:
    print('N/A')
    sys.exit(0)
try:
    url = f'http://{local_mgmt}/cgi-bin/streamscape_api'
    batch = [
        {'jsonrpc':'2.0', 'method':'battery_percent', 'id':1},
        {'jsonrpc':'2.0', 'method':'routing_tree', 'id':2},
        {'jsonrpc':'2.0', 'method':'neighbor_list', 'id':3},
        {'jsonrpc':'2.0', 'method':'rssi', 'id':4},
        {'jsonrpc':'2.0', 'method':'snr_stats', 'id':5},
    ]
    req = urllib.request.Request(url, data=json.dumps(batch).encode(),
                                headers={'Content-Type':'application/json'})
    with urllib.request.urlopen(req, timeout=3) as resp:
        parsed = {r['id']: r.get('result') for r in json.loads(resp.read())}
    bat = parsed.get(1, ['?'])[0]
    bat = f'{float(bat):.0f}' if bat and bat != '?' else '?'
    mesh = len(parsed.get(2) or [])
    neighbors = parsed.get(3) or []
    rssi_raw = parsed.get(4)
    snr_raw = parsed.get(5)
    # Format: bat|mesh|neighbor_count|rssi|snr
    rssi_str = str(rssi_raw) if rssi_raw else 'N/A'
    snr_str = str(snr_raw) if snr_raw else 'N/A'
    print(f'{bat}|{mesh}|{len(neighbors)}|{rssi_str}|{snr_str}')
except Exception as e:
    print(f'ERR:{e}')
"""
    import base64
    encoded = base64.b64encode(py_script.encode()).decode()
    args = " ".join(silvus_mgmt_ips)
    cmd = f"echo '{encoded}' | base64 -d | python3 - {args}"
    rc, stdout, _ = ssh_cmd(user, ip, cmd, timeout=8)
    if rc != 0 or not stdout or stdout.strip() in ("N/A", ""):
        return None

    line = stdout.strip()
    if line.startswith("ERR:"):
        return {"error": line}

    parts = line.split("|")
    if len(parts) >= 5:
        return {
            "battery": parts[0],
            "mesh_nodes": parts[1],
            "neighbor_count": parts[2],
            "rssi": parts[3],
            "snr": parts[4],
        }
    return {"raw": line}
