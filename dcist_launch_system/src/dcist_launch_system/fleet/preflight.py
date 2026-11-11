"""Pre-launch checks: is every machine of a launch ready, per platform.

One ssh round trip per machine runs a generated shell script that only reads state
(nothing is installed or changed) and prints one ``@@C`` line per check::

    @@C|<name>|<go|warn|nogo|info>|<detail>

plus ``@@V`` (code version) and ``@@T`` (clock) lines that are compared across the
fleet here.  Checks per machine:

* every machine: disk free under the output root, code version (superproject HEAD,
  dirty files, submodule pointers) against the reference machine, clock offset
  against this machine, GPU busy with leftover processes
* robots: the base station answers on the robot link network
* spot: Spot answers at ``$ADT4_BOSDYN_IP`` (ping + 443), the Spot SDK logs in and
  reports battery / estop / lease owner / motor power, the ZED is on USB
* phoenix: the Ouster (``sensor_hostname`` from the robot's phoenix hardware config)
  answers ping and reports RUNNING, the Clearpath services are active, BMS battery,
  ``platform/emergency_stop`` released, ZED on USB

:func:`verify_prior` checks, after the prior has been copied, that the exact prior
directory every machine will load holds the source's map.

A NO-GO does not stop a launch by itself: ``run_launch`` asks the user to confirm
the specific issues (:func:`nogo_confirmation`).
"""

from __future__ import annotations

import base64
import shlex
import time
from dataclasses import asdict, dataclass

GO = "go"
WARN = "warn"
NOGO = "nogo"
INFO = "info"
STATUSES = (GO, WARN, NOGO, INFO)

DISK_NOGO_GB = 10
DISK_WARN_GB = 30
CLOCK_WARN_S = 0.2
CLOCK_NOGO_S = 1.0
BATTERY_NOGO_PCT = 20
BATTERY_WARN_PCT = 40
SSH_TIMEOUT = 25

# where a phoenix robot's Ouster settings live, relative to $ADT4_PHX_ROOT
OUSTER_CONFIG = "src/phoenix-hardware/src/hardware_launch/config/{robot}/ouster.yaml"
CLEARPATH_UNITS = ("clearpath-robot", "clearpath-platform")
ZED_USB_VENDOR = "2b03"  # Stereolabs


@dataclass
class Check:
    machine: str
    name: str
    status: str
    detail: str
    hint: str = ""

    def to_dict(self) -> dict:
        return asdict(self)


HINTS = {
    "ssh": "power the machine on and check it is on the network",
    "disk": "free space under the output root (old runs, bags)",
    "code": "deploy the same commit to every machine (git status / submodule update)",
    "clock": "sync clocks (chrony) before launching; TF and fleetlink use timestamps",
    "gpu": "kill the leftover GPU processes",
    "base_link": "check the radio / network between the robot and the base station",
    "spot_reach": "power Spot on and check the robot computer's ethernet to Spot",
    "spot_state": "release the estop / take the lease back / charge Spot",
    "zed": "plug the ZED in (USB) and check it with lsusb",
    "ouster": "power the Ouster and check the robot's sensor network (br0)",
    "clearpath": "sudo systemctl restart clearpath-robot on the robot",
    "battery": "charge or swap the battery",
    "estop": "release the emergency stop",
    "prior": "re-copy the prior map (launch copies it again when it differs)",
}


# --- remote script --------------------------------------------------------------

# env the checks need from the robot's interactive shell (ssh runs `zsh -l -c`, which
# does not read ~/.zshrc, where the ADT4_* exports live)
_ENV_VARS = (
    "ADT4_WS",
    "ADT4_ENV",
    "ADT4_BOSDYN_IP",
    "ADT4_BOSDYN_USERNAME",
    "ADT4_BOSDYN_PASSWORD",
    "ADT4_PHX_ROOT",
    "RMW_IMPLEMENTATION",
    "ROS_DOMAIN_ID",
    "ZENOH_ROUTER_CONFIG_URI",
    "ZENOH_SESSION_CONFIG_URI",
)

_COMMON = r"""
c() { printf '@@C|%s|%s|%s\n' "$1" "$2" "$3"; }
eval "$(zsh -ic 'typeset -px __VARS__' </dev/null 2>/dev/null | grep -E '^(export|typeset)' )" 2>/dev/null
for v in __VARS__; do [ -n "${(P)v}" ] && export "$v"; done 2>/dev/null
ws="${ADT4_WS:-$HOME/dcist_ws}"
out_root=__OUT__
out_root="${out_root/#\~/$HOME}"
d="$out_root"; while [ ! -d "$d" ] && [ "$d" != / ]; do d=$(dirname "$d"); done
avail_kb=$(df -Pk "$d" 2>/dev/null | awk 'NR==2{print $4}')
if [ -z "$avail_kb" ]; then c disk warn "could not read free space under $out_root"
else
  gb=$((avail_kb / 1048576))
  if [ "$gb" -lt __DISK_NOGO__ ]; then c disk nogo "only ${gb} GB free under $out_root"
  elif [ "$gb" -lt __DISK_WARN__ ]; then c disk warn "${gb} GB free under $out_root"
  else c disk go "${gb} GB free under $out_root"; fi
fi
repo="$ws/src/awesome_dcist_t4"
if git -C "$repo" rev-parse HEAD >/dev/null 2>&1; then
  head=$(git -C "$repo" rev-parse HEAD)
  dirty=$(git -C "$repo" status --porcelain --ignore-submodules=none 2>/dev/null | grep -v '^??' | wc -l)
  subs=$(git -C "$repo" submodule status 2>/dev/null | awk '{print $1" "$2}' | sed 's/^[-+U ]//' | sha1sum | cut -c1-12)
  branch=$(git -C "$repo" rev-parse --abbrev-ref HEAD 2>/dev/null)
  printf '@@V|%s|%s|%s|%s\n' "$head" "$dirty" "$subs" "$branch"
else c code nogo "no git checkout at $repo"; fi
if command -v nvidia-smi >/dev/null 2>&1; then
  apps=$(timeout 5 nvidia-smi --query-compute-apps=pid,process_name --format=csv,noheader 2>/dev/null | tr '\n' ';' | sed 's/;$//')
  if [ -n "$apps" ]; then c gpu warn "GPU already in use: $apps"
  else c gpu go "$(timeout 5 nvidia-smi -L 2>/dev/null | head -1 | cut -c1-60)"; fi
fi
"""

_BASE_LINK = r"""
for a in __BASE_ADDRS__; do
  if ping -c1 -W2 "$a" >/dev/null 2>&1; then c base_link go "base station $a reachable"; base_ok=1; break; fi
done
[ -z "$base_ok" ] && c base_link nogo "base station unreachable at __BASE_ADDRS_TEXT__"
"""

_ZED = r"""
if command -v lsusb >/dev/null 2>&1; then
  if lsusb 2>/dev/null | grep -qi 'ID __ZED__:'; then c zed go "ZED on USB"
  else c zed nogo "no ZED camera on USB"; fi
fi
"""

_ROS_ENV = r"""
ros_src() {
  for distro in ${=ROS_DISTRO:-jazzy humble}; do source /opt/ros/$distro/setup.zsh 2>/dev/null && break; done
  source "$ws/install/setup.zsh" 2>/dev/null
}
"""

# Spot SDK probe: battery, estop, lease owner, motor power.  Credentials come from
# the robot's own env and are never printed.
_SPOT_PY = r"""
import os, sys
try:
    import bosdyn.client
    from bosdyn.client.lease import LeaseClient
    from bosdyn.client.robot_state import RobotStateClient
except Exception as e:
    print("warn|Spot SDK not importable (%s)" % type(e).__name__); sys.exit()
try:
    sdk = bosdyn.client.create_standard_sdk("adt4-preflight")
    robot = sdk.create_robot(os.environ["ADT4_BOSDYN_IP"])
    robot.authenticate(os.environ.get("ADT4_BOSDYN_USERNAME", ""), os.environ.get("ADT4_BOSDYN_PASSWORD", ""), timeout=5)
    st = robot.ensure_client(RobotStateClient.default_service_name).get_robot_state(timeout=5)
except Exception as e:
    print("nogo|Spot SDK login/state failed: %s" % (str(e).splitlines() or [type(e).__name__])[0][:120]); sys.exit()
bad, notes = [], []
bats = [b.charge_percentage.value for b in st.battery_states if b.HasField("charge_percentage")]
if bats:
    pct = int(min(bats))
    notes.append("battery %d%%" % pct)
    if pct < int(sys.argv[1]):
        bad.append("battery %d%%" % pct)
stopped = [e.name for e in st.estop_states if e.state == e.STATE_ESTOPPED]
if stopped:
    bad.append("estop engaged: " + ",".join(stopped))
try:
    for l in robot.ensure_client(LeaseClient.default_service_name).list_leases(timeout=5):
        if l.resource == "body" and l.lease_owner.client_name:
            notes.append("lease held by " + l.lease_owner.client_name)
except Exception:
    pass
notes.append("motors " + st.power_state.MotorPowerState.Name(st.power_state.motor_power_state).replace("STATE_", "").lower())
print(("nogo|" if bad else "go|") + "; ".join(bad + notes))
"""

_SPOT = r"""
ip="$ADT4_BOSDYN_IP"
if [ -z "$ip" ]; then c spot_reach nogo "ADT4_BOSDYN_IP is not set on this robot"
elif ! ping -c1 -W2 "$ip" >/dev/null 2>&1; then c spot_reach nogo "Spot does not answer ping at $ip"
elif ! timeout 3 bash -c "</dev/tcp/$ip/443" 2>/dev/null; then c spot_reach nogo "Spot at $ip answers ping but not its API port 443"
else
  c spot_reach go "Spot reachable at $ip"
  py=python3; [ -x "$ADT4_ENV/bin/python" ] && py="$ADT4_ENV/bin/python"
  r=$(echo __SPOT_PY__ | base64 -d | ADT4_BOSDYN_IP="$ip" timeout 20 "$py" - __BAT_NOGO__ 2>/dev/null | tail -1)
  [ -z "$r" ] && r="warn|Spot SDK check did not answer"
  c spot_state "${r%%|*}" "${r#*|}"
fi
"""

_PHOENIX = r"""
cfg="${ADT4_PHX_ROOT}/__OUSTER_CFG__"
oip=$(sed -n "s/^[[:space:]]*sensor_hostname:[[:space:]]*['\"]\{0,1\}\([^'\"]*\)['\"]\{0,1\}.*/\1/p" "$cfg" 2>/dev/null | head -1)
if [ -z "$oip" ]; then c ouster warn "no Ouster sensor_hostname in $cfg"
elif ! ping -c1 -W2 "$oip" >/dev/null 2>&1; then c ouster nogo "Ouster does not answer ping at $oip"
else
  ost=$(curl -s -m 3 "http://$oip/api/v1/sensor/metadata/sensor_info" 2>/dev/null | grep -o '"status" *: *"[A-Z_]*"' | grep -o '[A-Z_]*"$' | tr -d '"')
  if [ "$ost" = RUNNING ]; then c ouster go "Ouster at $oip RUNNING"
  elif [ -n "$ost" ]; then c ouster nogo "Ouster at $oip is $ost"
  else c ouster warn "Ouster at $oip answers ping; status unknown (HTTP API did not answer)"; fi
fi
down=""
for u in __UNITS__; do [ "$(systemctl is-active "$u" 2>/dev/null)" = active ] || down="$down $u"; done
if [ -n "$down" ]; then c clearpath nogo "not active:$down"; else c clearpath go "__UNITS__ active"; fi
(
  ros_src
  bat=$(timeout 5 ros2 topic echo /__ROBOT__/platform/bms/state --once --no-daemon 2>/dev/null | sed -n 's/^percentage:[[:space:]]*//p' | head -1)
  if [ -z "$bat" ]; then c battery warn "no message on /__ROBOT__/platform/bms/state"
  else
    pct=$(python3 -c "print(int(round(float('$bat')*100)))" 2>/dev/null)
    if [ -z "$pct" ]; then c battery warn "unreadable BMS percentage '$bat'"
    elif [ "$pct" -lt __BAT_NOGO__ ]; then c battery nogo "battery ${pct}%"
    elif [ "$pct" -lt __BAT_WARN__ ]; then c battery warn "battery ${pct}%"
    else c battery go "battery ${pct}%"; fi
  fi
  es=$(timeout 5 ros2 topic echo /__ROBOT__/platform/emergency_stop --once --no-daemon 2>/dev/null | sed -n 's/^data:[[:space:]]*//p' | head -1)
  case "$es" in
    false) c estop go "emergency stop released" ;;
    true) c estop nogo "emergency stop pressed" ;;
    *) c estop warn "no message on /__ROBOT__/platform/emergency_stop" ;;
  esac
)
"""

_CLOCK = "printf '@@T|%s\\n' \"$(date +%s.%N)\"\n"


def _b64(text: str) -> str:
    return base64.b64encode(text.encode()).decode()


def machine_script(machine: dict, platform: str | None, topo: dict | None, output_root: str, base_addrs=()) -> str:
    """The read-only check script for one machine (robot or base station)."""
    s = (
        _COMMON.replace("__VARS__", " ".join(_ENV_VARS))
        .replace("__OUT__", shlex.quote(output_root))
        .replace("__DISK_NOGO__", str(DISK_NOGO_GB))
        .replace("__DISK_WARN__", str(DISK_WARN_GB))
    )
    s += _ROS_ENV
    if machine.get("role") == "robot":
        if base_addrs:
            s += _BASE_LINK.replace("__BASE_ADDRS__", " ".join(shlex.quote(a) for a in base_addrs)).replace(
                "__BASE_ADDRS_TEXT__", "/".join(base_addrs)
            )
        if platform == "spot":
            s += _SPOT.replace("__SPOT_PY__", _b64(_SPOT_PY)).replace("__BAT_NOGO__", str(BATTERY_NOGO_PCT))
            s += _ZED.replace("__ZED__", ZED_USB_VENDOR)
        elif platform == "phoenix":
            robot = machine["name"]
            s += (
                _PHOENIX.replace("__OUSTER_CFG__", OUSTER_CONFIG.format(robot=robot))
                .replace("__UNITS__", " ".join(CLEARPATH_UNITS))
                .replace("__ROBOT__", robot)
                .replace("__BAT_NOGO__", str(BATTERY_NOGO_PCT))
                .replace("__BAT_WARN__", str(BATTERY_WARN_PCT))
            )
            s += _ZED.replace("__ZED__", ZED_USB_VENDOR)
    s += _CLOCK
    return s


# --- parsing / comparison -------------------------------------------------------


def parse_output(machine: str, out: str) -> tuple[list[Check], dict | None, float | None]:
    """``@@C`` checks, the ``@@V`` version dict and the ``@@T`` remote time."""
    checks, version, remote_t = [], None, None
    for line in (out or "").splitlines():
        if line.startswith("@@C|"):
            parts = line.split("|", 3)
            if len(parts) == 4:
                _, name, status, detail = parts
                status = status if status in STATUSES else WARN
                checks.append(Check(machine, name, status, detail.strip(), HINTS.get(name, "")))
        elif line.startswith("@@V|"):
            parts = line.split("|")
            if len(parts) >= 4:
                version = {
                    "head": parts[1],
                    "dirty": int(parts[2]) if parts[2].strip().isdigit() else 0,
                    "subs": parts[3],
                    "branch": parts[4] if len(parts) > 4 else "",
                }
        elif line.startswith("@@T|"):
            try:
                remote_t = float(line[4:])
            except ValueError:
                pass
    return checks, version, remote_t


def clock_check(machine: str, remote_t: float | None, t_returned: float) -> Check | None:
    """Offset of the remote clock: its time printed just before exit vs when the
    output arrived here (the error is the one-way latency, ms on a LAN)."""
    if remote_t is None:
        return None
    off = remote_t - t_returned
    a = abs(off)
    status = NOGO if a > CLOCK_NOGO_S else WARN if a > CLOCK_WARN_S else GO
    return Check(machine, "clock", status, f"clock offset {off:+.2f} s vs this machine", HINTS["clock"])


def version_checks(versions: dict, reference: str | None) -> list[Check]:
    """Every machine's code against ``reference`` (the base station, else the first)."""
    if not versions:
        return []
    ref_name = reference if reference in versions else sorted(versions)[0]
    ref = versions[ref_name]
    out = []
    for name, v in sorted(versions.items()):
        short = f"{v['head'][:8]} ({v['branch']})"
        problems = []
        if v["head"] != ref["head"]:
            problems.append(f"superproject {short} != {ref_name} {ref['head'][:8]}")
        elif v["subs"] != ref["subs"]:
            problems.append(f"submodule pointers differ from {ref_name}")
        if v["dirty"]:
            problems.append(f"{v['dirty']} uncommitted change(s)")
        if v["head"] != ref["head"] or v["subs"] != ref["subs"]:
            status = NOGO
        elif v["dirty"]:
            status = WARN
        else:
            status = GO
        detail = "; ".join(problems) if problems else f"{short}, same as {ref_name}" if name != ref_name else f"{short} (reference)"
        out.append(Check(name, "code", status, detail, HINTS["code"]))
    return out


def run_checks(machines: list[dict], topo: dict | None, output_root: str, deps, reference: str | None = None) -> list[Check]:
    """Run the checks on ``machines`` in parallel; returns every machine's checks."""
    from dcist_launch_system.fleet_helpers import fleetlink_base, platform_of

    machines_topo = (topo or {}).get("machines") or {}
    base = fleetlink_base(topo) or reference
    base_addrs = list(((machines_topo.get(base) or {}).get("addresses") or {}).values()) if base else []

    def one(m):
        platform = platform_of(machines_topo.get(m["name"]) or m) if m.get("role") == "robot" else None
        script = machine_script(m, platform, topo, output_root, base_addrs if m["name"] != base else ())
        rc, out, err = deps.ssh(m["user"], m["ip"], script, timeout=SSH_TIMEOUT)
        return rc, out, err, time.time()

    results = deps.parallel(one, machines)
    checks, versions = [], {}
    for m, res in sorted(results, key=lambda r: r[0]["name"]):
        name = m["name"]
        # the script always ends with @@T: without it the checks did not (all) run
        if isinstance(res, Exception) or "@@T|" not in (res[1] or ""):
            why = str(res) if isinstance(res, Exception) else (res[2] or f"ssh exit {res[0]}, no check output")
            checks.append(Check(name, "ssh", NOGO, f"cannot run checks: {why.strip()[:160]}", HINTS["ssh"]))
            continue
        rc, out, _, t_ret = res
        cs, version, remote_t = parse_output(name, out)
        checks.extend(cs)
        if version:
            versions[name] = version
        ck = clock_check(name, remote_t, t_ret)
        if ck:
            checks.append(ck)
    checks.extend(version_checks(versions, reference))
    return checks


# --- prior map ------------------------------------------------------------------


def _digests(entry: dict | None) -> tuple[str | None, str | None, str | None]:
    man = (entry or {}).get("manifest") or {}
    return man.get("map_digest"), man.get("digest"), (entry or {}).get("dsg_sig")


def compare_prior(machine: str, path: str, source: dict, copy: dict | None) -> Check:
    """One machine's prior directory against the source run.

    GO when the map files (hydra/ + roman/) match, else when the whole manifest
    matches; WARN when only the saved scene graph matches; NO-GO otherwise.
    """
    if copy is None:
        return Check(machine, "prior", NOGO, f"prior map missing at {path}", HINTS["prior"])
    missing = [d for d in ("hydra", "roman") if d not in (copy.get("subdirs") or [])]
    if missing:
        return Check(machine, "prior", NOGO, f"{path} has no {'/'.join(missing)}/", HINTS["prior"])
    s_map, s_all, s_dsg = _digests(source)
    c_map, c_all, c_dsg = _digests(copy)
    if s_map and c_map:
        if s_map == c_map:
            return Check(machine, "prior", GO, f"{path}: same map as the source", "")
    elif s_all and s_all == c_all:
        return Check(machine, "prior", GO, f"{path}: same files as the source", "")
    if s_dsg and s_dsg == c_dsg:
        return Check(machine, "prior", WARN, f"{path}: same scene graph, other map files differ", HINTS["prior"])
    return Check(machine, "prior", NOGO, f"{path} holds a different map than the source", HINTS["prior"])


def verify_prior(source_machine: dict, source_path: str, targets: dict, deps, scan) -> list[Check]:
    """``targets``: machine name -> (machine, prior path it will load).

    ``scan(machine, parent, [name], manifest=True)`` returns run_scan identity
    entries (``launch_workflow.scan_identities``)."""
    import pathlib

    def entry(m, path):
        p = pathlib.PurePosixPath(path)
        runs = scan(deps, m, str(p.parent), [p.name], manifest=True)
        return runs[0] if runs else None

    src = entry(source_machine, source_path)
    if src is None:
        return [Check(source_machine["name"], "prior", NOGO, f"cannot read the source prior {source_path}", HINTS["prior"])]
    out = []
    for name, (m, path) in sorted(targets.items()):
        if name == source_machine["name"] and path == source_path:
            out.append(Check(name, "prior", GO, f"{path} (the source)", ""))
            continue
        out.append(compare_prior(name, path, src, entry(m, path)))
    return out


# --- summary --------------------------------------------------------------------


def nogos(checks: list[Check]) -> list[Check]:
    return [c for c in checks if c.status == NOGO]


def verdicts(checks: list[Check]) -> dict:
    """machine -> "go" | "warn" | "nogo" (worst check)."""
    rank = {GO: 0, INFO: 0, WARN: 1, NOGO: 2}
    out: dict = {}
    for c in checks:
        cur = out.get(c.machine, GO)
        out[c.machine] = c.status if rank[c.status] > rank[cur] else cur
    return {m: (GO if v == INFO else v) for m, v in out.items()}


def issue_text(c: Check) -> str:
    return f"{c.machine} / {c.name}: {c.detail}" + (f"  → {c.hint}" if c.hint else "")


def nogo_confirmation(issues: list[Check], stage: str = "Preflight"):
    """The Confirmation that names each NO-GO; the console makes the user tick each
    issue (``detail.issues``) before "Launch anyway" is enabled."""
    from dcist_launch_system.fleet.launch_workflow import Confirmation

    lines = "\n".join(f"• {issue_text(c)}" for c in issues)
    return Confirmation(
        kind="preflight_nogo",
        message=f"{stage}: {len(issues)} NO-GO issue(s):\n{lines}\n\nLaunch anyway?",
        yes_label="Launch anyway",
        no_label="Abort",
        detail={"stage": stage, "issues": [c.to_dict() for c in issues]},
    )
