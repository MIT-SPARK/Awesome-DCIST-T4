"""Tests for the UI-independent launch workflow — all side effects are faked."""

import pytest

from dcist_launch_system.fleet.launch_workflow import (
    TMUX_SESSION_NAME,
    LaunchDeps,
    LaunchError,
    LaunchPlan,
    build_base_station_command,
    build_robot_commands,
    default_selection,
    dry_run_lines,
    run_launch,
    session_launches_rviz,
)

OUT = "~/adt4_output"

RUNTIME = {
    "smaug": {
        "name": "smaug",
        "role": "robot",
        "ip": "10.0.0.1",
        "user": "swarm",
        "platform_id": "smaug",
        "online": True,
    },
    "topaz": {
        "name": "topaz",
        "role": "robot",
        "ip": "10.0.0.2",
        "user": "swarm",
        "platform_id": "topaz",
        "online": True,
    },
    "willow": {
        "name": "willow",
        "role": "base_station",
        "ip": "10.0.0.9",
        "user": "rrg",
        "platform_id": "base_station",
        "online": True,
    },
}

TOPO = {
    "fleet_defaults": {
        "spot_platforms": ["smaug", "topaz"],
        "sessions": {
            "mapping": {"spot": "spot-default", "phoenix": "phoenix-default"},
            "relocalize": {"spot": "spot-relocalize", "phoenix": "phoenix-reloc"},
        },
        "base_station_session": "base_station-default",
    }
}


class FakeSsh:
    """Scripted ssh: ``rules`` maps a substring of the command to (rc, out, err)."""

    def __init__(self, rules=None):
        self.rules = rules or {}
        self.calls = []

    def __call__(self, user, ip, cmd, timeout=5):
        self.calls.append((ip, cmd))
        for needle, result in self.rules.items():
            if needle in cmd and (result[3] if len(result) > 3 else ip) == ip:
                return result[:3]
        return 0 if "run-adt4" in cmd else 1, "", ""

    def cmds_for(self, ip):
        return [c for i, c in self.calls if i == ip]


class Recorder:
    def __init__(self, answers=None, zenohd=False):
        self.lines = []
        self.requests = []
        self.answers = list(answers or [])
        self.zenohd = zenohd
        self.stopped = False
        self.pct = {}

    def log(self, level, text):
        self.lines.append((level, text))

    def confirm(self, request):
        self.requests.append(request)
        return self.answers.pop(0) if self.answers else None

    def progress(self, machine, pct):
        self.pct[machine] = pct

    def local_zenohd_running(self):
        return self.zenohd

    def stop_local_zenohd(self):
        self.stopped = True

    def text(self):
        return "\n".join(t for _, t in self.lines)


def make_deps(ssh, rsync=None):
    return LaunchDeps(
        ssh=ssh,
        rsync=rsync or (lambda *a, **k: (True, "direct", "ok")),
        deploy_zenoh=lambda user, ip, eps: (True, "deployed"),
        zenoh_endpoints=lambda topo, net, name, robot_filter=None: ["tcp/x:7447"],
        rviz=lambda name: f"/tmp/dcist_{name}.rviz",
        parallel=lambda fn, items: [(i, fn(i)) for i in items],
        sleep=lambda s: None,
    )


def run(plan, ssh, rec, rsync=None, preflight=False):
    plan.preflight = preflight  # the preflight phase has its own tests (test_preflight.py)
    return run_launch(
        plan,
        topo=TOPO,
        active_network="silvus",
        runtime_config=RUNTIME,
        output_root=OUT,
        callbacks=rec,
        deps=make_deps(ssh, rsync),
    )


# --- command construction -----------------------------------------------------


def test_robot_command_golden():
    plan = LaunchPlan(robots=["smaug"], session="spot-default", experiment="e1")
    [(m, cmd)] = build_robot_commands(plan, RUNTIME, OUT)
    assert m["name"] == "smaug"
    assert cmd == (
        "cd ~/dcist_ws && source install/setup.zsh && "
        "src/awesome_dcist_t4/dcist_launch_system/bin/run-adt4 spot-default "
        "-n smaug -c smaug -o ~/adt4_output/e1 -y "
        '--tmuxp-args "-d"'
    )


def test_robot_command_flags_and_prior_paths():
    plan = LaunchPlan(
        robots=["smaug", "topaz"],
        session="spot-relocalize",
        experiment="e2",
        mode="relocalize",
        prior_path="~/adt4_output/bldg45",
        prior_source="smaug",
        record=True,
    )
    cmds = dict((m["name"], c) for m, c in build_robot_commands(plan, RUNTIME, OUT))
    # source machine keeps its original path, others get prior_<name>
    assert "bldg45" in cmds["smaug"] and "prior_bldg45" not in cmds["smaug"]
    assert "prior_bldg45" in cmds["topaz"]
    assert all(" -r " in c for c in cmds.values())


@pytest.mark.parametrize(
    "kwargs, msg",
    [
        (dict(robots=[], session="s", experiment="e"), "No robots"),
        (dict(robots=["smaug"], session="", experiment="e"), "No session"),
        (dict(robots=["smaug"], session="s", experiment=""), "No experiment"),
        (
            dict(robots=["smaug"], session="s", experiment="e", mode="relocalize"),
            "requires a prior",
        ),
        (dict(robots=["ghost"], session="s", experiment="e"), "Unknown robot"),
    ],
)
def test_plan_validation(kwargs, msg):
    with pytest.raises(LaunchError, match=msg):
        build_robot_commands(LaunchPlan(**kwargs), RUNTIME, OUT)


def test_base_station_command_is_backgrounded_with_rviz():
    cmd = build_base_station_command(
        RUNTIME["willow"], "base_station-default", OUT, "e1", "/tmp/r.rviz"
    )
    assert "nohup" in cmd and cmd.endswith("&")
    assert "-c base_station" in cmd and "--rviz-config /tmp/r.rviz" in cmd


def test_dry_run_mentions_prior_transfer():
    plan = LaunchPlan(
        robots=["smaug", "topaz"],
        session="s",
        experiment="e",
        prior_path="~/adt4_output/p",
        prior_source="smaug",
    )
    lines = dry_run_lines(plan, RUNTIME, OUT)
    assert any("Transfer prior map: smaug -> topaz" in line for line in lines)
    assert sum(line.startswith("ssh ") for line in lines) == 2


def test_default_selection_mapping_and_relocalize():
    sel = default_selection(TOPO, RUNTIME, "mapping", None, {"10.0.0.9"})
    assert sel["robots"] == ["smaug", "topaz"]
    assert sel["session"] == "spot-default"
    assert sel["base_station"] == "willow"
    assert sel["bs_session"] == "base_station-default"
    sel = default_selection(TOPO, RUNTIME, "relocalize", ["topaz"], set())
    assert sel["session"] == "spot-relocalize" and sel["base_station"] is None


# --- full sequence ------------------------------------------------------------


def test_happy_path_launches_robot_and_base_station():
    # the post-launch check finds the base station's session; nothing ran before
    ssh = FakeSsh({"&& echo OK": (0, "OK", "", "10.0.0.9")})
    rec = Recorder(zenohd=True)
    plan = LaunchPlan(
        robots=["smaug"],
        session="spot-default",
        experiment="e",
        base_station="willow",
        bs_session="base_station-default",
    )
    assert run(plan, ssh, rec) is True
    assert rec.requests == []
    assert rec.stopped  # fleet zenohd handed over to the base station
    assert any("run-adt4 spot-default" in c for c in ssh.cmds_for("10.0.0.1"))
    assert any("nohup" in c for c in ssh.cmds_for("10.0.0.9"))
    assert "base station launched" in rec.text()


def test_session_launches_rviz_from_the_tmuxp_files():
    assert session_launches_rviz("base_station_heracles-default") is True
    assert session_launches_rviz("base_station_web_repair-default") is False
    assert session_launches_rviz("base_station_web-default") is False
    assert session_launches_rviz("base_station_console-default") is False
    assert session_launches_rviz("no-such-session") is True  # unknown: keep the config
    assert session_launches_rviz(None) is True


@pytest.mark.parametrize("bs_session,with_rviz", [("base_station-default", True), ("base_station_web_repair-default", False)])
def test_rviz_config_only_for_sessions_with_rviz(bs_session, with_rviz):
    ssh = FakeSsh({"&& echo OK": (0, "OK", "", "10.0.0.9")})
    rec = Recorder()
    made = []
    deps = make_deps(ssh)
    deps.rviz = lambda name: made.append(name) or f"/tmp/dcist_{name}.rviz"
    plan = LaunchPlan(robots=["smaug"], session="spot-default", experiment="e", base_station="willow", bs_session=bs_session, preflight=False)
    assert run_launch(plan, topo=TOPO, active_network="silvus", runtime_config=RUNTIME, output_root=OUT, callbacks=rec, deps=deps)
    (bs_cmd,) = [c for c in ssh.cmds_for("10.0.0.9") if "nohup" in c]
    assert ("--rviz-config" in bs_cmd) is with_rviz
    assert bool(made) is with_rviz


def test_base_station_command_passes_the_prior():
    cmd = build_base_station_command(
        RUNTIME["willow"], "base_station-default", OUT, "e1", prior_path="~/adt4_output/p"
    )
    assert " -p ~/adt4_output/p" in cmd and "ADT4_NEO4J_SEED=true " in cmd
    plain = build_base_station_command(RUNTIME["willow"], "b", OUT, "e1")
    assert " -p " not in plain and "ADT4_NEO4J_SEED" not in plain


def _reloc_plan(source):
    return LaunchPlan(
        robots=["smaug"],
        session="spot-relocalize",
        experiment="e",
        mode="relocalize",
        prior_path="~/adt4_output/p",
        prior_source=source,
        base_station="willow",
        bs_session="base_station-default",
    )


def _bs_launch(ssh):
    return next(c for c in ssh.cmds_for("10.0.0.9") if "nohup" in c)


def test_relocalize_base_station_gets_the_prior():
    # prior on the base station: it loads its own copy
    ssh = FakeSsh({"&& echo OK": (0, "OK", "", "10.0.0.9")})
    assert run(_reloc_plan("willow"), ssh, Recorder()) is True
    assert " -p ~/adt4_output/p" in _bs_launch(ssh)
    # prior on a robot: the base station gets prior_<name> like the other machines
    ssh = FakeSsh({"&& echo OK": (0, "OK", "", "10.0.0.9")})
    calls = []
    rec = Recorder()
    assert run(_reloc_plan("smaug"), ssh, rec, rsync=lambda *a, **k: calls.append(a) or (True, "direct", "ok")) is True
    assert " -p ~/adt4_output/prior_p" in _bs_launch(ssh)
    assert rec.pct.get("willow") == 100


def test_running_base_station_session_is_replaced():
    # a session left from the mapping run must not keep running with no prior
    ssh = FakeSsh({"2>/dev/null": (0, "ADT4_OUTPUT_DIR=/home/rrg/adt4_output/map_1715\nOK", "", "10.0.0.9")})
    rec = Recorder(answers=[True])
    assert run(_reloc_plan("willow"), ssh, rec) is True
    [req] = rec.requests
    assert req.detail["machines"] == ["willow"]
    # the prompt says what would be killed
    assert "willow (running map_1715)" in req.message
    assert req.detail["experiments"] == {"willow": "map_1715"}
    cmds = ssh.cmds_for("10.0.0.9")
    kill = next(i for i, c in enumerate(cmds) if "kill-session" in c)
    launch = next(i for i, c in enumerate(cmds) if "nohup" in c)
    assert kill < launch


def test_base_station_session_hosting_the_launcher_is_left_alone(monkeypatch):
    import dcist_launch_system.fleet.launch_workflow as lw

    monkeypatch.setattr(lw, "_hosts_this_process", lambda m, deps: True)
    ssh = FakeSsh({"2>/dev/null": (0, "OK", "", "10.0.0.9")})
    rec = Recorder(answers=[True])
    run(_reloc_plan("willow"), ssh, rec)
    assert rec.requests == []
    assert not any("kill-session" in c or "nohup" in c for c in ssh.cmds_for("10.0.0.9"))
    assert "restart the base station from a terminal" in rec.text()


def test_session_check_uses_real_tmux_session_name():
    ssh = FakeSsh({"tmux has-session": (0, "", "", "10.0.0.1")})
    rec = Recorder(answers=[False])
    plan = LaunchPlan(robots=["smaug"], session="spot-default", experiment="e")
    assert run(plan, ssh, rec) is False
    [req] = rec.requests
    assert req.kind == "kill_sessions" and req.detail["machines"] == ["smaug"]
    checks = [c for c in ssh.cmds_for("10.0.0.1") if "has-session" in c]
    assert checks and all(TMUX_SESSION_NAME in c for c in checks)
    assert not any("run-adt4" in c for c in ssh.cmds_for("10.0.0.1"))


def test_session_conflict_accepted_kills_then_launches():
    ssh = FakeSsh({"tmux has-session": (0, "", "", "10.0.0.1")})
    rec = Recorder(answers=[True])
    plan = LaunchPlan(robots=["smaug"], session="spot-default", experiment="e")
    assert run(plan, ssh, rec) is True
    cmds = ssh.cmds_for("10.0.0.1")
    kill = next(i for i, c in enumerate(cmds) if "kill-session" in c)
    launch = next(i for i, c in enumerate(cmds) if "run-adt4" in c)
    assert kill < launch


@pytest.mark.parametrize("size", [1024, 3 * 1024**3])
def test_existing_output_is_kept_and_launch_proceeds(size):
    # run-adt4 adds runs/<run_id> under an existing experiment dir: no prompt, no rm -rf
    ssh = FakeSsh(
        {
            "echo EXISTS": (0, "EXISTS", "", "10.0.0.1"),
            "du -sb": (0, str(size), "", "10.0.0.1"),
        }
    )
    rec = Recorder()
    plan = LaunchPlan(robots=["smaug"], session="spot-default", experiment="e")
    assert run(plan, ssh, rec) is True
    assert rec.requests == []
    cmds = ssh.cmds_for("10.0.0.1")
    assert not any(c.startswith("rm -rf") for c in cmds)
    assert any("run-adt4" in c for c in cmds)
    assert "earlier runs are kept" in rec.text()


def test_prior_transfer_skipped_when_present_and_aborts_on_failure():
    plan = LaunchPlan(
        robots=["smaug", "topaz"],
        session="spot-relocalize",
        experiment="e",
        prior_path="~/adt4_output/p",
        prior_source="smaug",
    )
    # topaz already has it → no rsync
    ssh = FakeSsh({"echo OK": (0, "OK", "", "10.0.0.2")})
    calls = []
    rec = Recorder()
    assert run(plan, ssh, rec, rsync=lambda *a, **k: calls.append(a)) is True
    assert calls == [] and rec.pct["topaz"] == 100

    # topaz lacks it and rsync fails → abort before launching anything
    ssh = FakeSsh()
    rec = Recorder()
    assert run(plan, ssh, rec, rsync=lambda *a, **k: (False, "", "boom")) is False
    assert "transfer FAILED: boom" in rec.text()
    assert not any("run-adt4" in c for _, c in ssh.calls)


def test_failed_robot_launch_returns_false():
    ssh = FakeSsh({"run-adt4": (2, "", "nope", "10.0.0.2")})
    rec = Recorder()
    plan = LaunchPlan(robots=["smaug", "topaz"], session="s", experiment="e")
    assert run(plan, ssh, rec) is False
    assert "smaug: launched" in rec.text() and "topaz: FAILED (rc=2) nope" in rec.text()


# --- per-robot platform sessions ---------------------------------------------

MIXED_RUNTIME = {
    "hamilton": {
        "name": "hamilton", "role": "robot", "ip": "10.0.0.3", "user": "swarm",
        "platform_id": "topaz", "online": True,
    },
    "gauss": {
        "name": "gauss", "role": "robot", "ip": "10.0.0.4", "user": "swarm",
        "platform_id": "samoyed", "online": True,
    },
}  # fmt: skip
MIXED_TOPO = {
    **TOPO,
    "machines": {
        "hamilton": {"role": "robot", "platform_id": "topaz"},  # no platform_type: spot
        "gauss": {"role": "robot", "platform_id": "samoyed", "platform_type": "husky"},
    },
}


def _sessions(plan, topo=MIXED_TOPO):
    return {
        m["name"]: c.split("run-adt4 ", 1)[1].split()[0]
        for m, c in build_robot_commands(plan, MIXED_RUNTIME, OUT, topo)
    }


def test_mixed_fleet_launch_runs_each_robots_platform_preset():
    plan = LaunchPlan(robots=["hamilton", "gauss"], session="spot-default", experiment="e", mode="mapping")
    assert _sessions(plan) == {"hamilton": "spot-default", "gauss": "phoenix-default"}
    # whichever preset the UI showed (the first robot's), and without a mode
    plan.session, plan.mode = "phoenix-default", None
    assert _sessions(plan) == {"hamilton": "spot-default", "gauss": "phoenix-default"}
    plan.session, plan.mode, plan.prior_path = "spot-relocalize", "relocalize", "~/p"
    assert _sessions(plan) == {"hamilton": "spot-relocalize", "gauss": "phoenix-reloc"}


def test_hand_picked_session_runs_on_every_robot():
    plan = LaunchPlan(robots=["hamilton", "gauss"], session="spot_bag-default", experiment="e")
    assert _sessions(plan) == {"hamilton": "spot_bag-default", "gauss": "spot_bag-default"}
    # no topology (the routes' up-front validation): the plan's session as given
    plan.session = "spot-default"
    cmds = build_robot_commands(plan, MIXED_RUNTIME, OUT)
    assert all("run-adt4 spot-default " in c for _, c in cmds)


def test_default_selection_per_robot_platform():
    sel = default_selection(MIXED_TOPO, MIXED_RUNTIME, "mapping", ["gauss", "hamilton"], set())
    assert sel["session"] == "phoenix-default"  # the first robot's
    assert sel["sessions"] == {"gauss": "phoenix-default", "hamilton": "spot-default"}
    # platform_type wins over fleet_defaults.spot_platforms (platform ids)
    topo = {**MIXED_TOPO, "fleet_defaults": {**TOPO["fleet_defaults"], "spot_platforms": ["samoyed"]}}
    sel = default_selection(topo, MIXED_RUNTIME, "mapping", ["gauss"], set())
    assert sel["session"] == "phoenix-default"
    # a robot missing from the topology's machines: its runtime entry, else spot
    sel = default_selection(TOPO, RUNTIME, "mapping", None, set())
    assert sel["sessions"] == {"smaug": "spot-default", "topaz": "spot-default"}


# --- base station only / per-platform sessions -----------------------------------

PP_RUNTIME = {
    **RUNTIME,
    "gauss": {
        "name": "gauss",
        "role": "robot",
        "ip": "10.0.0.3",
        "user": "swarm",
        "platform_id": "gauss",
        "online": True,
    },
}
PP_TOPO = {
    **TOPO,
    "machines": {
        "smaug": {"platform_type": "spot"},
        "topaz": {"platform_type": "spot"},
        "gauss": {"platform_type": "husky"},
    },
}


def _base_only_plan(**kw):
    return LaunchPlan(
        robots=[], session="", experiment="e", base_station="willow",
        bs_session="base_station_web-default", **kw,
    )  # fmt: skip


def test_base_only_plan_validates_and_dry_runs_only_the_base_command():
    from dcist_launch_system.fleet.launch_workflow import is_base_only, validate_plan

    plan = _base_only_plan()
    assert is_base_only(plan)
    assert validate_plan(plan, RUNTIME, OUT) == []
    lines = dry_run_lines(plan, RUNTIME, OUT)
    ssh_lines = [line for line in lines if line.startswith("ssh ")]
    assert len(ssh_lines) == 1
    assert ssh_lines[0].startswith('ssh rrg@10.0.0.9 "')
    assert "run-adt4 base_station_web-default -n willow" in ssh_lines[0]
    # no base station: still "No robots"
    with pytest.raises(LaunchError, match="No robots"):
        validate_plan(LaunchPlan(robots=[], session="", experiment="e"), RUNTIME, OUT)
    with pytest.raises(LaunchError, match="requires a prior"):
        validate_plan(_base_only_plan(mode="relocalize"), RUNTIME, OUT)


def test_base_only_dry_run_with_a_robot_prior_transfers_it_to_the_base():
    plan = _base_only_plan(mode="relocalize", prior_path="~/adt4_output/p", prior_source="smaug")
    lines = dry_run_lines(plan, RUNTIME, OUT)
    assert "Transfer prior map: smaug -> willow" in lines
    assert " -p ~/adt4_output/prior_p" in lines[-1]


def test_base_only_launch_touches_only_the_base_station():
    # an adt4_system session runs on the base: confirm, kill it, relaunch it
    ssh = FakeSsh({"2>/dev/null": (0, "ADT4_OUTPUT_DIR=/x/old\nOK", "", "10.0.0.9")})
    rec = Recorder(answers=[True])
    seen = {}
    deps = make_deps(ssh)
    deps.zenoh_endpoints = lambda topo, net, name, robot_filter=None: seen.setdefault("f", robot_filter) or []
    plan = _base_only_plan(preflight=False)
    assert run_launch(plan, topo=TOPO, active_network="silvus", runtime_config=RUNTIME, output_root=OUT, callbacks=rec, deps=deps)
    assert ssh.cmds_for("10.0.0.1") == [] and ssh.cmds_for("10.0.0.2") == []
    [req] = rec.requests
    assert req.detail["machines"] == ["willow"]
    cmds = ssh.cmds_for("10.0.0.9")
    assert any("kill-session" in c for c in cmds)
    assert sum("nohup" in c for c in cmds) == 1
    # the restarted base still links to every robot
    assert sorted(seen["f"]) == ["smaug", "topaz"]


def test_base_only_preflight_checks_only_the_base(monkeypatch):
    from dcist_launch_system.fleet import preflight as pf

    checked = []
    monkeypatch.setattr(pf, "run_checks", lambda machines, *a, **k: checked.extend(m["name"] for m in machines) or [])
    ssh = FakeSsh({"&& echo OK": (0, "OK", "", "10.0.0.9")})
    plan = _base_only_plan(preflight=True)
    assert run_launch(plan, topo=TOPO, active_network="silvus", runtime_config=RUNTIME, output_root=OUT, callbacks=Recorder(), deps=make_deps(ssh))
    assert checked == ["willow"]


def test_mixed_platforms_get_their_own_sessions():
    plan = LaunchPlan(
        robots=["smaug", "gauss"],
        session="",
        sessions={"spot": "spot_relocalize-relocalize", "phoenix": "phoenix_relocalize-relocalize"},
        experiment="e",
        mode="relocalize",
        prior_path="~/adt4_output/p",
    )
    cmds = {m["name"]: c for m, c in build_robot_commands(plan, PP_RUNTIME, OUT, PP_TOPO)}
    assert "run-adt4 spot_relocalize-relocalize -n smaug" in cmds["smaug"]
    assert "run-adt4 phoenix_relocalize-relocalize -n gauss" in cmds["gauss"]
    assert all(" -p ~/adt4_output/p" in c for c in cmds.values())
    # a platform with no session: named in the error
    plan.sessions = {"spot": "spot-default"}
    with pytest.raises(LaunchError, match="phoenix"):
        build_robot_commands(plan, PP_RUNTIME, OUT, PP_TOPO)
    # ...unless ``session`` covers it
    plan.session = "phoenix-default"
    cmds = {m["name"]: c for m, c in build_robot_commands(plan, PP_RUNTIME, OUT, PP_TOPO)}
    assert "run-adt4 spot-default" in cmds["smaug"] and "run-adt4 phoenix-default" in cmds["gauss"]


def test_single_session_still_runs_on_every_robot():
    plan = LaunchPlan(robots=["smaug", "gauss"], session="my-custom", experiment="e")
    cmds = [c for _, c in build_robot_commands(plan, PP_RUNTIME, OUT, PP_TOPO)]
    assert all("run-adt4 my-custom " in c for c in cmds)
    # a per-platform preset still maps to each robot's platform (old behaviour)
    plan.session = "spot-default"
    cmds = {m["name"]: c for m, c in build_robot_commands(plan, PP_RUNTIME, OUT, PP_TOPO)}
    assert "run-adt4 phoenix-default" in cmds["gauss"]


def test_base_station_command_drops_leaked_tmux_session_vars():
    cmd = build_base_station_command(RUNTIME["willow"], "b", OUT, "e1")
    assert "env -u config -u logging_key -u sim_time " in cmd
    assert cmd.index("env -u") < cmd.index("nohup")


def test_robots_get_the_prior_without_its_image_store_the_base_with_it():
    # prior made on willow (the base): smaug (robot) copies it without image_store/
    plan = LaunchPlan(
        robots=["smaug"],
        session="spot-relocalize",
        experiment="e",
        mode="relocalize",
        prior_path="~/adt4_output/p",
        prior_source="willow",
        base_station="willow",
        bs_session="base_station-default",
    )
    excludes = []
    rsync = lambda *a, **k: excludes.append(k.get("exclude")) or (True, "direct", "ok")
    assert run(plan, FakeSsh({"&& echo OK": (0, "OK", "", "10.0.0.9")}), Recorder(), rsync=rsync) is True
    assert excludes and all("image_store/" in e for e in excludes)
    # prior made on smaug: the base station copies it with its images
    excludes.clear()
    run(_reloc_plan("smaug"), FakeSsh({}), Recorder(), rsync=rsync)
    assert excludes and all("image_store/" not in e for e in excludes)
