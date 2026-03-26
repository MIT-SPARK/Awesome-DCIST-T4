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
        "echo '---DISK---'; df -h ~ 2>/dev/null | tail -1 | awk '{print $4 \" / \" $2 \" (\" $5 \" used)\"}'"
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
        return {"tmux_sessions": "N/A", "ros2_procs": "N/A", "load": "N/A", "disk": "N/A", "radio": "N/A"}

    sections = {}
    current = None
    for line in stdout.splitlines():
        if line.startswith("---") and line.endswith("---"):
            current = line.strip("-")
            sections[current] = []
        elif current:
            sections[current].append(line)

    tmux_lines = sections.get("TMUX", ["none"])
    tmux_sessions = "; ".join(tmux_lines) if tmux_lines[0] != "none" else "none"

    return {
        "tmux_sessions": tmux_sessions,
        "ros2_procs": (sections.get("ROS2", ["0"])[0] or "0"),
        "load": (sections.get("LOAD", ["N/A"])[0] or "N/A"),
        "disk": (sections.get("DISK", ["N/A"])[0] or "N/A"),
        "radio": (sections.get("SILVUS", ["N/A"])[0] or "N/A"),
    }
