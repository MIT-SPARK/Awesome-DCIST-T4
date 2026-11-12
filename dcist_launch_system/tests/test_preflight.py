"""Pre-launch checks (fleet.preflight) and their place in the launch workflow."""

import os
import shutil
import subprocess
import time

import pytest

from dcist_launch_system.fleet import preflight as pf
from dcist_launch_system.fleet.launch_workflow import LaunchDeps, LaunchPlan, run_launch

OUT = "~/adt4_output"

RUNTIME = {
    "hamilton": {"name": "hamilton", "role": "robot", "ip": "10.0.0.3", "user": "swarm", "platform_id": "hamilton"},
    "gauss": {"name": "gauss", "role": "robot", "ip": "10.0.0.4", "user": "swarm", "platform_id": "gauss"},
    "willow": {"name": "willow", "role": "base_station", "ip": "10.0.0.9", "user": "rrg", "platform_id": "base_station"},
}

TOPO = {
    "machines": {
        "hamilton": {"role": "robot", "platform_type": "spot", "addresses": {"silvus": "203.0.113.3"}},
        "gauss": {"role": "robot", "platform_type": "husky", "addresses": {"silvus": "203.0.113.4"}},
        "willow": {"role": "base_station", "addresses": {"silvus": "203.0.113.9", "mit_wifi": "10.0.0.9"}},
    },
    "fleet_defaults": {
        "sessions": {"mapping": {"spot": "spot-default", "phoenix": "phoenix-default"}},
        "base_station_session": "base_station-default",
        "fleetlink_base": "willow",
    },
}

HEAD = "a" * 40


def ok_output(*checks, head=HEAD, subs="s1", dirty=0, t=None):
    lines = [f"@@C|{n}|{s}|{d}" for n, s, d in checks]
    lines.append(f"@@V|{head}|{dirty}|{subs}|main")
    lines.append(f"@@T|{time.time() if t is None else t}")
    return "\n".join(lines)


class ScriptSsh:
    """ssh fake: preflight scripts (they print @@T) answer per ip; a tmux session
    exists on a machine once run-adt4 ran there; anything else fails."""

    def __init__(self, outputs, fail=()):
        self.outputs = outputs
        self.fail = set(fail)
        self.calls = []
        self.launched = set()

    def __call__(self, user, ip, cmd, timeout=5):
        self.calls.append((ip, cmd))
        if "@@T" in cmd:
            if ip in self.fail:
                return 255, "", "ssh: connect to host port 22: No route to host"
            return 0, self.outputs.get(ip, ok_output()), ""
        if "run-adt4" in cmd:
            self.launched.add(ip)
            return 0, "", ""
        if "tmux has-session" in cmd:
            return (0, "OK", "") if ip in self.launched else (1, "", "")
        return 1, "", ""


class Rec:
    def __init__(self, answers=()):
        self.lines, self.requests, self.answers = [], [], list(answers)

    def log(self, level, text):
        self.lines.append((level, text))

    def confirm(self, request):
        self.requests.append(request)
        return self.answers.pop(0) if self.answers else None

    def progress(self, machine, pct):
        pass

    def local_zenohd_running(self):
        return False

    def stop_local_zenohd(self):
        pass

    def text(self):
        return "\n".join(t for _, t in self.lines)


def deps(ssh, scan=None):
    return LaunchDeps(
        ssh=ssh,
        rsync=lambda *a, **k: (True, "direct", "ok"),
        deploy_zenoh=lambda user, ip, eps: (True, "deployed"),
        zenoh_endpoints=lambda topo, net, name, robot_filter=None: [],
        rviz=lambda name: f"/tmp/dcist_{name}.rviz",
        parallel=lambda fn, items: [(i, fn(i)) for i in items],
        sleep=lambda s: None,
        local_ips=lambda: [],
        scan=scan,
    )


# --- parsing / comparisons ----------------------------------------------------


def test_parse_output_checks_version_and_time():
    out = "junk\n@@C|ouster|nogo|Ouster does not answer ping at 192.0.2.99\n@@V|%s|2|abc|main\n@@T|1700000000.5" % HEAD
    checks, version, t = pf.parse_output("gauss", out)
    assert [(c.name, c.status) for c in checks] == [("ouster", "nogo")]
    assert checks[0].hint == pf.HINTS["ouster"]
    assert version == {"head": HEAD, "dirty": 2, "subs": "abc", "branch": "main"}
    assert t == 1700000000.5


def test_unknown_status_becomes_warn():
    checks, _, _ = pf.parse_output("x", "@@C|disk|weird|?")
    assert checks[0].status == pf.WARN


def test_version_checks_against_reference():
    v = {
        "willow": {"head": HEAD, "dirty": 0, "subs": "s1", "branch": "main"},
        "hamilton": {"head": HEAD, "dirty": 3, "subs": "s1", "branch": "main"},
        "gauss": {"head": "b" * 40, "dirty": 0, "subs": "s1", "branch": "old"},
        "euclid": {"head": HEAD, "dirty": 0, "subs": "s2", "branch": "main"},
    }
    got = {c.machine: c for c in pf.version_checks(v, "willow")}
    assert got["willow"].status == pf.GO and "reference" in got["willow"].detail
    assert got["hamilton"].status == pf.WARN and "3 uncommitted" in got["hamilton"].detail
    assert got["gauss"].status == pf.NOGO and "bbbbbbbb (old) != willow aaaaaaaa" in got["gauss"].detail
    assert got["euclid"].status == pf.NOGO and "submodule pointers" in got["euclid"].detail


@pytest.mark.parametrize("off,status", [(0.05, pf.GO), (-0.5, pf.WARN), (3.0, pf.NOGO)])
def test_clock_check(off, status):
    assert pf.clock_check("gauss", 100.0 + off, 100.0).status == status


def _entry(map_digest="m1", digest="d1", dsg="10:1", subdirs=("hydra", "roman")):
    return {"subdirs": list(subdirs), "manifest": {"digest": digest, "map_digest": map_digest}, "dsg_sig": dsg}


def test_compare_prior_cases():
    src = _entry()
    assert pf.compare_prior("g", "p", src, _entry(digest="other")).status == pf.GO  # same map, extra files
    assert pf.compare_prior("g", "p", src, _entry(map_digest="m2", dsg="99:2")).status == pf.NOGO
    assert pf.compare_prior("g", "p", src, _entry(map_digest="m2")).status == pf.WARN  # same scene graph
    assert pf.compare_prior("g", "p", src, None).status == pf.NOGO
    assert "roman/" in pf.compare_prior("g", "p", src, _entry(subdirs=("hydra",))).detail
    # scans from before map_digest existed: whole-manifest comparison
    old = lambda d: {"subdirs": ["hydra", "roman"], "manifest": {"digest": d}, "dsg_sig": None}
    assert pf.compare_prior("g", "p", old("d1"), old("d1")).status == pf.GO
    assert pf.compare_prior("g", "p", old("d1"), old("d2")).status == pf.NOGO


def test_verdicts_take_the_worst_check():
    cs = [pf.Check("a", "x", pf.GO, ""), pf.Check("a", "y", pf.WARN, ""), pf.Check("b", "x", pf.NOGO, ""), pf.Check("c", "x", pf.INFO, "")]
    assert pf.verdicts(cs) == {"a": pf.WARN, "b": pf.NOGO, "c": pf.GO}


def test_nogo_confirmation_names_each_issue():
    issues = [pf.Check("gauss", "ouster", pf.NOGO, "Ouster does not answer ping at 192.0.2.99", pf.HINTS["ouster"])]
    req = pf.nogo_confirmation(issues)
    assert req.kind == "preflight_nogo" and req.yes_label == "Launch anyway"
    assert "gauss / ouster: Ouster does not answer ping at 192.0.2.99" in req.message
    assert req.detail["issues"][0]["name"] == "ouster"


# --- per-platform scripts -----------------------------------------------------


def test_scripts_differ_by_platform():
    spot = pf.machine_script(RUNTIME["hamilton"], "spot", TOPO, OUT, ["203.0.113.9"])
    phx = pf.machine_script(RUNTIME["gauss"], "phoenix", TOPO, OUT, ["203.0.113.9"])
    base = pf.machine_script(RUNTIME["willow"], None, TOPO, OUT)
    assert "spot_reach" in spot and "ouster" not in spot
    assert "hardware_launch/config/gauss/ouster.yaml" in phx and "/gauss/platform/bms/state" in phx
    assert "spot_reach" not in phx and "clearpath-platform" in phx
    assert "base_link" in spot and "base_link" not in base and "spot_reach" not in base
    for s in (spot, phx, base):
        assert "@@T" in s and "@@V" in s


# The generated scripts run in a real zsh with stub commands on PATH, so quoting and
# parsing bugs show up here, not on the robot.
STUBS = {
    "ping": 'for a; do :; done; [ -n "$PING_OK" ] && case " $PING_OK " in *" $a "*) exit 0;; esac; exit 1',
    "curl": 'echo "{\\"status\\": \\"${OUSTER_STATUS:-RUNNING}\\", \\"prod_sn\\": 1}"',
    "systemctl": 'case " $DOWN_UNITS " in *" $2 "*) echo inactive;; *) echo active;; esac',
    "lsusb": '[ -n "$ZED" ] && echo "Bus 002 Device 003: ID 2b03:f880 STEREOLABS ZED 2i"; echo "Bus 001 Device 001: ID 1d6b:0002 Linux"',
    "ros2": 'case "$*" in *bms/state*) printf "header:\\n  frame_id: x\\npercentage: ${BMS:-0.83}\\n";; *emergency_stop*) echo "data: ${ESTOP:-false}";; esac',
    "nvidia-smi": 'exit 0',
}


def run_script(tmp_path, script, env):
    if not shutil.which("zsh"):
        pytest.skip("zsh not installed")
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir(exist_ok=True)
    for name, body in STUBS.items():
        f = bin_dir / name
        f.write_text("#!/bin/bash\n" + body + "\n")
        f.chmod(0o755)
    phx = tmp_path / "phx" / "src/phoenix-hardware/src/hardware_launch/config/gauss"
    phx.mkdir(parents=True, exist_ok=True)
    (phx / "ouster.yaml").write_text("/**:\n  ros__parameters:\n    sensor_hostname: '192.0.2.99'\n")
    full = {
        "PATH": f"{bin_dir}:/usr/bin:/bin",
        "HOME": str(tmp_path),
        "ADT4_PHX_ROOT": str(tmp_path / "phx"),
        "ADT4_WS": str(tmp_path / "ws"),
        "ROS_DISTRO": "none",  # keep the stub ros2 (a real /opt/ros setup would shadow it)
        **env,
    }
    r = subprocess.run(["zsh", "-c", script], capture_output=True, text=True, env=full, timeout=60)
    checks, _, t = pf.parse_output("gauss", r.stdout)
    return {c.name: c for c in checks}, t, r


def test_phoenix_script_all_go(tmp_path):
    script = pf.machine_script(RUNTIME["gauss"], "phoenix", TOPO, str(tmp_path), ["203.0.113.9"])
    got, t, r = run_script(tmp_path, script, {"PING_OK": "192.0.2.99 203.0.113.9", "ZED": "1"})
    assert t is not None, r.stderr
    for name in ("ouster", "clearpath", "battery", "estop", "zed", "base_link"):
        assert got[name].status == pf.GO, (name, got[name].detail, r.stderr)
    assert got["battery"].detail == "battery 83%"
    assert got["ouster"].detail == "Ouster at 192.0.2.99 RUNNING"


def test_phoenix_script_reports_each_problem(tmp_path):
    script = pf.machine_script(RUNTIME["gauss"], "phoenix", TOPO, str(tmp_path), ["203.0.113.9"])
    got, _, r = run_script(
        tmp_path, script, {"PING_OK": "", "DOWN_UNITS": "clearpath-platform", "BMS": "0.12", "ESTOP": "true"}
    )
    assert got["ouster"].status == pf.NOGO and "192.0.2.99" in got["ouster"].detail
    assert got["clearpath"].status == pf.NOGO and "clearpath-platform" in got["clearpath"].detail
    assert got["battery"].status == pf.NOGO and got["battery"].detail == "battery 12%"
    assert got["estop"].status == pf.NOGO
    assert got["zed"].status == pf.NOGO
    assert got["base_link"].status == pf.NOGO


def test_ouster_not_running(tmp_path):
    script = pf.machine_script(RUNTIME["gauss"], "phoenix", TOPO, str(tmp_path))
    got, _, _ = run_script(tmp_path, script, {"PING_OK": "192.0.2.99", "OUSTER_STATUS": "INITIALIZING"})
    assert got["ouster"].status == pf.NOGO and got["ouster"].detail.endswith("is INITIALIZING")


def test_spot_script_unreachable_and_unset(tmp_path):
    script = pf.machine_script(RUNTIME["hamilton"], "spot", TOPO, str(tmp_path))
    got, _, _ = run_script(tmp_path, script, {"ADT4_BOSDYN_IP": "10.0.0.3", "PING_OK": ""})
    assert got["spot_reach"].status == pf.NOGO and got["spot_reach"].detail == "Spot does not answer ping at 10.0.0.3"
    got, _, _ = run_script(tmp_path, script, {})
    assert got["spot_reach"].detail == "ADT4_BOSDYN_IP is not set on this robot"


def test_disk_threshold(tmp_path):
    script = pf.machine_script(RUNTIME["willow"], None, TOPO, str(tmp_path / "missing" / "deeper"))
    got, _, _ = run_script(tmp_path, script, {})
    assert got["disk"].status in (pf.GO, pf.WARN, pf.NOGO) and "GB free" in got["disk"].detail


# --- fleet run ------------------------------------------------------------------


def test_run_checks_flags_unreachable_machine_and_version_drift():
    ssh = ScriptSsh(
        {"10.0.0.3": ok_output(("spot_reach", "go", "Spot reachable at 10.0.0.3")), "10.0.0.9": ok_output()},
        fail={"10.0.0.4"},
    )
    checks = pf.run_checks([RUNTIME["hamilton"], RUNTIME["gauss"], RUNTIME["willow"]], TOPO, OUT, deps(ssh), reference="willow")
    by = {(c.machine, c.name): c for c in checks}
    assert by[("gauss", "ssh")].status == pf.NOGO and "No route to host" in by[("gauss", "ssh")].detail
    assert by[("hamilton", "code")].status == pf.GO
    assert by[("hamilton", "clock")].status == pf.GO
    # robots get the base station's link addresses; the base station does not
    ham = next(c for ip, c in ssh.calls if ip == "10.0.0.3")
    assert "203.0.113.9" in ham and "spot_reach" in ham
    assert "base_link" not in next(c for ip, c in ssh.calls if ip == "10.0.0.9")


def plan(**kw):
    base = dict(robots=["hamilton", "gauss"], session="spot-default", experiment="e1", base_station="willow", bs_session="base_station-default")
    base.update(kw)
    return LaunchPlan(**base)


def launch(p, ssh, rec, scan=None):
    return run_launch(p, topo=TOPO, active_network="mit_wifi", runtime_config=RUNTIME, output_root=OUT, callbacks=rec, deps=deps(ssh, scan))


def test_nogo_asks_with_the_specific_issues_and_abort_stops_before_anything():
    ssh = ScriptSsh({"10.0.0.4": ok_output(("ouster", "nogo", "Ouster does not answer ping at 192.0.2.99"))})
    rec = Rec(answers=[False])
    assert launch(plan(), ssh, rec) is False
    [req] = rec.requests
    assert req.kind == "preflight_nogo"
    assert [i["machine"] + "/" + i["name"] for i in req.detail["issues"]] == ["gauss/ouster"]
    assert "Launch aborted: Preflight NO-GO not confirmed." in rec.text()
    assert not any("run-adt4" in c for _, c in ssh.calls)


def test_confirmed_nogo_launches():
    ssh = ScriptSsh({"10.0.0.4": ok_output(("estop", "nogo", "emergency stop pressed"))})
    rec = Rec(answers=[True])
    assert launch(plan(), ssh, rec) is True
    assert "launching despite 1 NO-GO issue(s) (confirmed)" in rec.text()
    assert sum("run-adt4" in c for _, c in ssh.calls) == 3  # 2 robots + base station


def test_all_go_asks_nothing():
    rec = Rec()
    assert launch(plan(), ScriptSsh({}), rec) is True
    assert rec.requests == [] and "Preflight: GO" in rec.text()


def test_preflight_off_runs_no_checks():
    ssh = ScriptSsh({}, fail={"10.0.0.3", "10.0.0.4", "10.0.0.9"})
    assert launch(plan(preflight=False), ssh, Rec()) is True
    assert not any("@@T" in c for _, c in ssh.calls)


# --- prior map ------------------------------------------------------------------


SRC_PRIOR = "~/adt4_output/map_x"
PRIOR_ID = {"source": "run_info", "run_id": "r-1"}


def make_scan(copies):
    """``copies``: (machine, dir name) -> entry; identity-only scans list them all."""

    def scan(machine, root, only=None, manifest=False):
        out = []
        for (m, name), e in copies.items():
            if m != machine["name"] or (only and name not in only):
                continue
            out.append({"name": name, "path": f"{root}/{name}", "identity": PRIOR_ID, "origin": None, **e})
        return out

    return scan


def test_prior_verified_on_every_machine_and_mismatch_is_named():
    copies = {
        ("willow", "map_x"): _entry(),
        ("hamilton", "prior_map_x"): _entry(),
        ("gauss", "prior_map_x"): _entry(map_digest="stale", dsg="5:5"),
    }
    scan = make_scan(copies)
    # gauss's copy has the run id but another map: launch re-copies it, then verifies;
    # the fake rsync changes nothing, so the verification still finds it stale
    rec = Rec(answers=[False])
    ok = launch(plan(mode="relocalize", prior_path=SRC_PRIOR, prior_source="willow", session="spot-relocalize", sessions={"phoenix": "phoenix-relocalize"}), ScriptSsh({}), rec, scan)
    assert ok is False
    assert "gauss: prior_map_x has the prior's run id but a different map" in rec.text()
    [req] = rec.requests
    assert req.detail["stage"] == "Prior map"
    assert [i["machine"] for i in req.detail["issues"]] == ["gauss"]
    assert "hamilton / prior: ~/adt4_output/prior_map_x: same map as the source" in rec.text()


def test_matching_prior_copies_need_no_confirmation():
    copies = {("willow", "map_x"): _entry(), ("hamilton", "prior_map_x"): _entry(), ("gauss", "prior_map_x"): _entry()}
    rec = Rec()
    ok = launch(plan(mode="relocalize", prior_path=SRC_PRIOR, prior_source="willow", session="spot-relocalize", sessions={"phoenix": "phoenix-relocalize"}), ScriptSsh({}), rec, make_scan(copies))
    assert ok is True and rec.requests == []
    assert "Prior map: GO" in rec.text()
    assert "already present as prior_map_x" in rec.text()


def test_manifest_has_map_digest(tmp_path):
    from dcist_launch_system.fleet import run_scan

    for rel in ("hydra/dsg.json", "roman/map.pkl", "notes.txt"):
        p = tmp_path / rel
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(rel)
    a = run_scan.manifest(str(tmp_path))
    (tmp_path / "notes.txt").write_text("changed, longer")
    b = run_scan.manifest(str(tmp_path))
    assert a["map_digest"] == b["map_digest"] and a["digest"] != b["digest"]
    (tmp_path / "roman/map.pkl").write_text("another map")
    assert run_scan.manifest(str(tmp_path))["map_digest"] != a["map_digest"]


def test_every_remote_timeout_kills_a_command_that_ignores_sigterm():
    # `timeout N cmd` only sends SIGTERM; a ros2 CLI blocked in rmw_zenoh ignores it and
    # hangs the whole check script past SSH_TIMEOUT ("SSH timeout" while ssh works).
    import re

    from dcist_launch_system.fleet import preflight

    src = open(preflight.__file__).read()
    plain = re.findall(r"\btimeout (?!-k )\d+ ", src)
    assert plain == [], f"timeout without -k in preflight scripts: {plain}"
    assert "timeout -k 2 5 ros2 topic echo" in src
