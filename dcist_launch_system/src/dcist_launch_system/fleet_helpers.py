"""Shared helpers for fleet-adt4: topology loading, SSH, rsync, and remote queries."""

import base64
import ipaddress
import pathlib
import re
import shlex
import shutil
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
            # Use zsh login shell for local commands — sources .zshrc env vars
            # and handles ROS workspace setup.zsh syntax
            shell = "zsh" if shutil.which("zsh") else "bash"
            result = subprocess.run(
                [shell, "-l", "-c", cmd],
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

    ssh_opts = "ssh -o BatchMode=yes -o StrictHostKeyChecking=no -o ConnectTimeout=10"
    src_remote = f"{src_user}@{src_ip}:{src_path}"
    dst_remote = f"{dst_user}@{dst_ip}:{dst_path}"

    local_ips = get_local_ips()
    src_is_local = src_ip in local_ips
    dst_is_local = dst_ip in local_ips

    if src_is_local or dst_is_local:
        # One endpoint is local — single SSH hop using base station's keys.
        src_arg = (src_path if src_is_local else src_remote) + "/"
        dst_arg = (dst_path if dst_is_local else dst_remote) + "/"
        cmd = ["rsync"] + rsync_flags + ["-e", ssh_opts, src_arg, dst_arg]
        rc, out, err = _run_rsync(cmd, stream=stream, progress_callback=progress_callback)
        if rc == 0:
            return True, "direct", out
        return False, "direct", err.strip()

    if not relay:
        # Both endpoints remote — try direct robot-to-robot with agent forwarding.
        # BatchMode=yes ensures no password prompt; if it fails we relay instead.
        cmd = ["rsync"] + rsync_flags + ["-e", f"{ssh_opts} -A", src_remote + "/", dst_remote + "/"]
        rc, out, err = _run_rsync(cmd, stream=stream, progress_callback=progress_callback)
        if rc == 0:
            return True, "direct", out

    # Both endpoints remote and direct failed: relay through base station
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
            if robot_filter is not None and name not in robot_filter:
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


def _find_endpoints_array(text, section_start):
    """Find the endpoints array in a config section, returning (prefix_end, array_start, array_end).

    Uses bracket-counting so `]` inside IPv6 addresses like [::] are not confused
    with the array closing bracket.  Returns None if not found.
    """
    ep_prefix = re.compile(r'^(\s*endpoints\s*:\s*)\[', re.MULTILINE)
    for m in ep_prefix.finditer(text):
        # Skip commented-out lines
        line_start = text.rfind('\n', 0, m.start()) + 1
        if text[line_start:m.start()].strip().startswith('//'):
            continue
        # m.end() - 1 is the position of the opening '['; count until the matching ']'
        bracket_start = m.end() - 1  # position of '['
        depth = 0
        for i in range(bracket_start, len(text)):
            if text[i] == '[':
                depth += 1
            elif text[i] == ']':
                depth -= 1
                if depth == 0:
                    # section_start offsets positions back to the full content
                    return section_start + m.start(), section_start + m.end() - 1, section_start + i + 1
        break  # malformed — opening bracket with no match
    return None


def patch_zenoh_config_local(endpoints, template_path=None):
    """Create a patched zenoh config from the bundled template.

    Replaces connect.endpoints, preserving all other ROS transport/timeout/
    routing settings from the template.  The listen port is left at the
    template default (7447) since all zenoh instances — fleet or base station
    — share the same port.

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

    # Locate the connect: { ... } section
    connect_match = re.search(r'^\s*connect\s*:\s*\{', content, re.MULTILINE)
    if not connect_match:
        return None
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

    result = _find_endpoints_array(content[connect_match.start():connect_end], connect_match.start())
    if result is None:
        return None
    ep_prefix_start, ep_bracket_open, ep_bracket_close = result
    prefix_text = content[ep_prefix_start:ep_bracket_open]
    content = content[:ep_prefix_start] + prefix_text + new_array + content[ep_bracket_close:]

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


def _parse_status_yaml(yaml_text, robots):
    """Parse a single YAML document from ros2 topic echo into the robots dict."""
    try:
        data = yaml.safe_load(yaml_text)
    except Exception:
        return
    if not data or "status" not in data:
        return

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


class NodeStatusPoller:
    """Persistent background poller for ROS node status.

    Starts a single `ros2 topic echo` process that runs continuously.
    Parses YAML documents as they arrive and updates a shared status dict.
    Call `get_status()` to get the latest snapshot.
    """

    def __init__(self):
        self._proc = None
        self._status = {}  # {robot_name: [node dicts]}
        self._lock = __import__("threading").Lock()
        self._thread = None
        self._error = None

    def start(self):
        """Start the background poller. Safe to call multiple times."""
        if self._thread and self._thread.is_alive():
            return True, "already running"
        if self._proc and self._proc.poll() is None:
            self._proc.kill()
            self._proc = None

        shell = "zsh" if shutil.which("zsh") else "bash"
        ros_cmd = (
            "source ~/dcist_ws/install/setup.zsh 2>/dev/null || "
            "source ~/dcist_ws/install/setup.bash 2>/dev/null; "
            "ros2 topic echo /global/ros_system_monitor/table_in"
        )
        try:
            self._proc = subprocess.Popen(
                [shell, "-c", ros_cmd],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
        except OSError as e:
            self._error = str(e)
            return False, str(e)

        self._thread = __import__("threading").Thread(
            target=self._read_loop, daemon=True
        )
        self._thread.start()
        return True, "started"

    def stop(self):
        if self._proc and self._proc.poll() is None:
            self._proc.kill()
            self._proc = None

    def get_status(self):
        """Return a snapshot of the latest status dict."""
        with self._lock:
            import copy
            return copy.deepcopy(self._status)

    def get_error(self):
        return self._error

    def is_running(self):
        return self._proc is not None and self._proc.poll() is None

    def _read_loop(self):
        """Read stdout line-by-line, accumulate YAML docs, parse on '---'."""
        buf = []
        try:
            for line in self._proc.stdout:
                stripped = line.rstrip("\n")
                if stripped == "---":
                    if buf:
                        doc = "\n".join(buf)
                        with self._lock:
                            _parse_status_yaml(doc, self._status)
                        buf = []
                else:
                    buf.append(stripped)
        except Exception:
            pass
        # Process died — check stderr
        if self._proc:
            _, stderr = self._proc.communicate(timeout=2)
            if stderr and stderr.strip():
                self._error = stderr.strip()[:500]


def get_ros_node_status(timeout=5):
    """One-shot fetch of ROS node status (legacy interface).

    For continuous monitoring, use NodeStatusPoller instead.
    """
    shell = "zsh" if shutil.which("zsh") else "bash"
    ros_cmd = (
        "source ~/dcist_ws/install/setup.zsh 2>/dev/null || "
        "source ~/dcist_ws/install/setup.bash 2>/dev/null; "
        "ros2 topic echo /global/ros_system_monitor/table_in"
    )
    effective_timeout = timeout + 5
    try:
        proc = subprocess.Popen(
            [shell, "-c", ros_cmd],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        try:
            stdout, stderr = proc.communicate(timeout=effective_timeout)
        except subprocess.TimeoutExpired:
            proc.kill()
            stdout, stderr = proc.communicate(timeout=5)
        if not stdout or not stdout.strip():
            if stderr and stderr.strip():
                return {"_debug_stderr": stderr.strip()[:500]}
            return {}
    except OSError as e:
        return {"_debug_error": str(e)}

    robots = {}
    for doc in stdout.split("---"):
        doc = doc.strip()
        if not doc:
            continue
        _parse_status_yaml(doc, robots)

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


# ---- Bandwidth testing (iperf3) ----

def get_local_ip_for_network(topology, network):
    """Return the local machine's IP on the given network, or None."""
    local_ips = get_local_ips()
    subnet_str = topology.get("networks", {}).get(network, {}).get("subnet")
    if not subnet_str:
        return None
    subnet = ipaddress.ip_network(subnet_str, strict=False)
    for ip in local_ips:
        try:
            if ipaddress.ip_address(ip) in subnet:
                return ip
        except ValueError:
            pass
    return None


def check_iperf3(user, ip):
    """Check if iperf3 is installed on the machine.

    Returns True if installed, False otherwise.
    """
    rc, _, _ = ssh_cmd(user, ip, "which iperf3", timeout=5)
    return rc == 0


_IPERF3_INTERVAL_RE = re.compile(
    r"\[\s*\d+\]\s+([\d.]+)-([\d.]+)\s+sec\s+"   # [ 5]  0.00-1.00 sec
    r"[\d.]+\s+\S+Bytes\s+"                        # transfer (ignored)
    r"([\d.]+)\s+(G|M|K)?bits/sec"                 # bitrate
    r"(?:\s+([\d]+))?"                             # optional retransmits
)


def _parse_iperf3_line(line):
    """Parse one iperf3 text output line.  Returns dict or None."""
    m = _IPERF3_INTERVAL_RE.search(line)
    if not m:
        return None
    rate, unit = float(m.group(3)), m.group(4) or ""
    mbps = rate * 1000 if unit == "G" else rate if unit == "M" else rate / 1000 if unit == "K" else rate / 1e6
    return {
        "t_start": float(m.group(1)),
        "t_end": float(m.group(2)),
        "mbps": mbps,
        "retransmits": int(m.group(5)) if m.group(5) else None,
        "is_summary": "sender" in line or "receiver" in line,
        "is_receiver": "receiver" in line,
    }


def run_iperf3_test(server_ip, client_user, client_ip, port=5201, duration=5,
                    interval_cb=None):
    """Run a single iperf3 throughput test with optional per-second streaming.

    Starts an iperf3 server locally (single-run mode), then opens a streaming
    SSH connection to client_ip to run the iperf3 client with 1-second intervals.

    interval_cb(t_end, mbps, retransmits) is called once per second during the
    test (retransmits is None when not reported by iperf3).

    Returns {'mbps', 'retransmits', 'duration'} or {'error': str}.
    """
    import time

    try:
        server = subprocess.Popen(
            ["iperf3", "-s", "-1", "-p", str(port)],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
        )
        time.sleep(0.5)
        if server.poll() is not None:
            err = server.stderr.read().decode(errors="replace").strip()
            return {"error": f"iperf3 server failed to start: {err or 'unknown error'}"}

        use_tunnel = False
        if client_ip in get_local_ips():
            iperf_cmd = f"iperf3 -c {server_ip} -p {port} -t {duration} -i 1"
            proc = subprocess.Popen(
                ["zsh", "-l", "-c", iperf_cmd],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
            )
        else:
            # Probe whether the robot can reach our iperf3 port directly.
            # If not (e.g. MIT WiFi client isolation), tunnel through SSH so
            # the test still runs — but flag the result as tunneled since SSH
            # encryption will reduce measured throughput.
            rc, _, _ = ssh_cmd(client_user, client_ip,
                               f"nc -zw3 {server_ip} {port}", timeout=8)
            use_tunnel = (rc != 0)
            if use_tunnel:
                iperf_cmd = f"iperf3 -c localhost -p {port} -t {duration} -i 1"
                ssh_extra = ["-R", f"{port}:localhost:{port}"]
            else:
                iperf_cmd = f"iperf3 -c {server_ip} -p {port} -t {duration} -i 1"
                ssh_extra = []
            proc = subprocess.Popen(
                [
                    "ssh",
                    "-o", "ConnectTimeout=10",
                    "-o", "BatchMode=yes",
                    "-o", "StrictHostKeyChecking=no",
                    *ssh_extra,
                    f"{client_user}@{client_ip}",
                    iperf_cmd,
                ],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
            )

        final_mbps = None
        final_retr = 0
        final_duration = float(duration)

        for line in proc.stdout:
            parsed = _parse_iperf3_line(line)
            if parsed is None:
                continue
            if parsed["is_receiver"]:
                final_mbps = parsed["mbps"]
                final_retr = parsed["retransmits"] or 0
                final_duration = parsed["t_end"]
            elif not parsed["is_summary"] and interval_cb:
                interval_cb(parsed["t_end"], parsed["mbps"], parsed["retransmits"])

        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()

        if final_mbps is None:
            err = proc.stderr.read().strip()
            return {"error": err or "iperf3 produced no results"}

        return {
            "mbps": final_mbps,
            "retransmits": final_retr,
            "duration": final_duration,
            "tunneled": use_tunnel,
        }

    except Exception as e:
        return {"error": str(e)}
    finally:
        try:
            server.terminate()
            server.wait(timeout=3)
        except Exception:
            try:
                server.kill()
            except Exception:
                pass
