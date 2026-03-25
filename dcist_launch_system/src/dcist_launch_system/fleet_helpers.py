"""Shared helpers for fleet-adt4: topology loading, SSH, rsync, and remote queries."""

import ipaddress
import pathlib
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
    """Run a command via SSH. Returns (returncode, stdout, stderr)."""
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


def _run_rsync(cmd, stream=False, timeout=3600):
    """Run an rsync command. If stream=True, print output live."""
    if not stream:
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
            # rsync --info=progress2 outputs lines like: 1.23G  45%  12.34MB/s  0:01:23
            if "%" in line:
                print(f"\r    {line}", end="", flush=True)
                last_line = line
            else:
                pass  # skip file listing noise
        proc.wait()
        if last_line:
            print()  # newline after progress
        stderr = proc.stderr.read() if proc.stderr else ""
        return proc.returncode, "", stderr
    except (subprocess.TimeoutExpired, OSError) as e:
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
):
    """Rsync files between machines.

    Tries direct transfer first; falls back to relay through operator machine.
    If stream=True, print rsync progress to stdout in real time.
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
        rc, out, err = _run_rsync(cmd, stream=stream)
        if rc == 0:
            return True, "direct", out

    # Fallback: relay through operator
    local_tmp = pathlib.Path(relay_tmp) / pathlib.Path(src_path).name
    local_tmp.mkdir(parents=True, exist_ok=True)

    # Step 1: source -> operator
    if stream:
        print("    Fetching to local relay...")
    cmd1 = ["rsync"] + rsync_flags + [src_remote + "/", str(local_tmp) + "/"]
    rc1, _, err1 = _run_rsync(cmd1, stream=stream)
    if rc1 != 0:
        return False, "relay", f"Fetch failed: {err1}"

    # Step 2: operator -> destination
    if stream:
        print("    Pushing from local relay...")
    cmd2 = ["rsync"] + rsync_flags + [str(local_tmp) + "/", dst_remote + "/"]
    rc2, _, err2 = _run_rsync(cmd2, stream=stream)
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
    cmd = (
        f"if [ -d {output_root} ]; then "
        f"  cd {output_root} && "
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
    cmd = (
        f"if [ -d {exp_path} ]; then "
        f"  cd {exp_path} && "
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


def get_remote_status(user, ip):
    """Get system status from a remote machine.

    Returns dict with keys: tmux_sessions, ros2_procs, load, disk.
    """
    cmd = (
        "echo '---TMUX---'; tmux list-sessions 2>/dev/null || echo 'none'; "
        "echo '---ROS2---'; pgrep -c -f 'ros2|zenoh' 2>/dev/null || echo '0'; "
        "echo '---LOAD---'; uptime | sed 's/.*load average: //'; "
        "echo '---DISK---'; df -h ~ 2>/dev/null | tail -1 | awk '{print $4 \" / \" $2 \" (\" $5 \" used)\"}'"
    )
    rc, stdout, _ = ssh_cmd(user, ip, cmd, timeout=10)
    if rc != 0 and not stdout:
        return {"tmux_sessions": "N/A", "ros2_procs": "N/A", "load": "N/A", "disk": "N/A"}

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
    }
