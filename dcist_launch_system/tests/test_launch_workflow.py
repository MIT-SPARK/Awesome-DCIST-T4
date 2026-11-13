"""Tests for the UI-independent launch workflow — all side effects are faked."""

import re

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


def test_small_existing_output_asks_plain_confirm_then_removes():
    ssh = FakeSsh(
        {
            "echo EXISTS": (0, "EXISTS", "", "10.0.0.1"),
            "du -sb": (0, "1024\n", "", "10.0.0.1"),
        }
    )
    rec = Recorder(answers=[True])
    plan = LaunchPlan(robots=["smaug"], session="spot-default", experiment="e")
    assert run(plan, ssh, rec) is True
    [req] = rec.requests
    assert req.kind == "overwrite_output" and req.required_text is None
    assert any(c.startswith("rm -rf") for c in ssh.cmds_for("10.0.0.1"))


def test_large_existing_output_requires_typed_confirm():
    ssh = FakeSsh(
        {
            "echo EXISTS": (0, "EXISTS", "", "10.0.0.1"),
            "du -sb": (0, str(3 * 1024**3), "", "10.0.0.1"),
        }
    )
    rec = Recorder(answers=[None])  # timed out / closed
    plan = LaunchPlan(robots=["smaug"], session="spot-default", experiment="e")
    assert run(plan, ssh, rec) is False
    [req] = rec.requests
    assert req.required_text == "confirm" and req.detail["bytes"] == 3 * 1024**3
    assert not any(c.startswith("rm -rf") for c in ssh.cmds_for("10.0.0.1"))
    assert "timed out" in rec.text()


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


def test_hand_picked_session_runs_on_robots_of_its_platform_and_maps_on_the_others():
    plan = LaunchPlan(robots=["hamilton", "gauss"], session="spot_bag-default", experiment="e")
    # a Husky never runs a spot_* session: the phoenix equivalent instead
    assert _sessions(plan) == {"hamilton": "spot_bag-default", "gauss": "phoenix_bag-default"}
    # a session that belongs to no platform runs on every robot as given
    plan.session = "custom-thing"
    assert _sessions(plan) == {"hamilton": "custom-thing", "gauss": "custom-thing"}
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
    # ...unless ``session`` covers it (mapping sessions: a mapping launch)
    plan.session, plan.mode = "phoenix-default", "mapping"
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


# --- session vs launch mode ------------------------------------------------------


def test_session_mode_from_the_experiment_manifest():
    from dcist_launch_system.fleet.launch_workflow import session_mode, session_modes

    for s in ("spot_relocalize-relocalize", "phoenix_relocalize-relocalize_awcd",
              "spot_bag_relocalize-relocalize", "phoenix_bag_relocalize-relocalize"):
        assert session_mode(s) == "relocalize", s
    for s in ("spot-default", "spot-indoor_image_logging", "phoenix-default",
              "spot_bag-indoor", "phoenix_bag-default", "spot_regions-indoor_regions"):
        assert session_mode(s) == "mapping", s
    # base station / planning sessions, made-up names, configs an experiment lacks
    for s in ("base_station_web_repair-default", "spot_prior_dsg-prior_dsg", "my-custom",
              "spot-relocalize", "nodash", "", None):
        assert session_mode(s) is None, s
    assert session_modes(["spot-default", "base_station-default", "spot_relocalize-relocalize"]) == {
        "spot-default": "mapping",
        "spot_relocalize-relocalize": "relocalize",
    }


def test_relocalize_mode_refuses_a_mapping_session():
    # the 2026-11-11 field run: relocalize + a remembered mapping session
    plan = LaunchPlan(
        robots=["smaug", "topaz"], session="spot-indoor_image_logging", experiment="e",
        mode="relocalize", prior_path="~/adt4_output/p", prior_source="willow",
    )  # fmt: skip
    with pytest.raises(LaunchError, match=r"relocalize.*smaug would run spot-indoor_image_logging \(a mapping session\)"):
        build_robot_commands(plan, RUNTIME, OUT, TOPO)
    from dcist_launch_system.fleet.launch_workflow import validate_plan

    with pytest.raises(LaunchError, match="topaz"):
        validate_plan(plan, RUNTIME, OUT, TOPO)
    with pytest.raises(LaunchError, match="mapping session"):
        dry_run_lines(plan, RUNTIME, OUT, TOPO)
    rec = Recorder()
    ssh = FakeSsh()
    assert not run(plan, ssh, rec)
    assert "mapping session" in rec.text() and ssh.calls == []
    plan.session = "spot_relocalize-relocalize"
    assert len(build_robot_commands(plan, RUNTIME, OUT, TOPO)) == 2


def test_mapping_mode_refuses_a_relocalize_session():
    plan = LaunchPlan(robots=["smaug"], session="spot_relocalize-relocalize", experiment="e",
                      mode="mapping", prior_path="~/p")  # fmt: skip
    with pytest.raises(LaunchError, match=r"mapping but smaug would run spot_relocalize-relocalize \(a relocalize session\)"):
        build_robot_commands(plan, RUNTIME, OUT)
    # no mode (manual / old clients): not checked
    plan.mode = None
    assert build_robot_commands(plan, RUNTIME, OUT)
    # a session the manifest does not know: not checked
    plan.mode, plan.session = "mapping", "my-custom"
    assert build_robot_commands(plan, RUNTIME, OUT)


def test_per_platform_sessions_are_checked_per_robot():
    plan = LaunchPlan(
        robots=["smaug", "gauss"], session="",
        sessions={"spot": "spot_relocalize-relocalize", "phoenix": "phoenix-default"},
        experiment="e", mode="relocalize", prior_path="~/adt4_output/p",
    )  # fmt: skip
    with pytest.raises(LaunchError) as e:
        build_robot_commands(plan, PP_RUNTIME, OUT, PP_TOPO)
    assert "gauss would run phoenix-default" in str(e.value) and "smaug" not in str(e.value)
    plan.sessions["phoenix"] = "phoenix_relocalize-relocalize"
    assert len(build_robot_commands(plan, PP_RUNTIME, OUT, PP_TOPO)) == 2
    # relocalize still needs a prior
    plan.prior_path = ""
    with pytest.raises(LaunchError, match="requires a prior"):
        build_robot_commands(plan, PP_RUNTIME, OUT, PP_TOPO)


def test_legacy_single_mapping_preset_in_relocalize_mode_is_refused():
    # the mapping preset maps to each robot's platform: still a mapping session
    plan = LaunchPlan(robots=["smaug", "gauss"], session="spot-default", experiment="e",
                      mode="relocalize", prior_path="~/p")  # fmt: skip
    topo = {**PP_TOPO, "fleet_defaults": {**TOPO["fleet_defaults"], "sessions": {
        "mapping": {"spot": "spot-default", "phoenix": "phoenix-default"},
        "relocalize": {"spot": "spot_relocalize-relocalize", "phoenix": "phoenix_relocalize-relocalize"},
    }}}  # fmt: skip
    with pytest.raises(LaunchError, match="gauss would run phoenix-default"):
        build_robot_commands(plan, PP_RUNTIME, OUT, topo)
    plan.session = "spot_relocalize-relocalize"
    cmds = {m["name"]: c for m, c in build_robot_commands(plan, PP_RUNTIME, OUT, topo)}
    assert "run-adt4 phoenix_relocalize-relocalize" in cmds["gauss"]


# --- session platform / robots-only relaunch (field test 2026-11-11) -------------

NOV11_TOPO = {
    "fleet_defaults": {
        "sessions": {
            "mapping": {"spot": "spot-default", "phoenix": "phoenix-default"},
            "relocalize": {"spot": "spot_relocalize-relocalize", "phoenix": "phoenix_relocalize-relocalize"},
        },
        "base_station_session": "base_station_web_repair-default",
        "fleet_link": "fleetlink",
        "fleetlink_base": "willow",
    },
    "machines": {
        "euclid": {"role": "robot", "platform_id": "smaug", "platform_type": "spot"},
        "hamilton": {"role": "robot", "platform_id": "topaz", "platform_type": "spot"},
        "gauss": {"role": "robot", "platform_id": "phoenix", "platform_type": "phoenix"},
        "willow": {"role": "base_station"},
    },
}
NOV11_RUNTIME = {
    "euclid": {"name": "euclid", "role": "robot", "ip": "10.0.0.11", "user": "swarm", "platform_id": "smaug", "online": True},
    "hamilton": {"name": "hamilton", "role": "robot", "ip": "10.0.0.12", "user": "swarm", "platform_id": "topaz", "online": True},
    "gauss": {"name": "gauss", "role": "robot", "ip": "10.0.0.13", "user": "swarm", "platform_id": "phoenix", "online": True},
    "willow": {"name": "willow", "role": "base_station", "ip": "10.0.0.9", "user": "rrg", "platform_id": "base_station", "online": True},
}  # fmt: skip
# the console's launch memory when gauss was relaunched alone on 2026-11-11
NOV11_MEMORY = {
    "robots": ["gauss"],
    "session": "spot_relocalize-relocalize_awcd",
    "sessions": {},
    "bs_session": "base_station_web_repair-default",
    "base_only": False,
    "base_station": "willow",
    "experiment": "reloc_awcd",
    "mode": "relocalize",
    "prior_path": "/home/rrg/adt4_output/fused_awcd",
    "prior_source": "willow",
}


def _nov11_plan(**kw):
    fields = {k: v for k, v in NOV11_MEMORY.items() if k != "base_only"}
    return LaunchPlan(**{**fields, **kw})


def test_session_platform_by_experiment_name():
    from dcist_launch_system.fleet.launch_workflow import platform_equivalent, session_platform

    assert session_platform("spot_relocalize-relocalize_awcd") == "spot"
    assert session_platform("phoenix-default") == "phoenix"
    assert session_platform("base_station_web-default") is None
    assert platform_equivalent("spot_relocalize-relocalize_awcd", "phoenix") == "phoenix_relocalize-relocalize_awcd"
    assert platform_equivalent("phoenix_relocalize-relocalize", "spot") == "spot_relocalize-relocalize"
    assert platform_equivalent("spot-default", "spot") == "spot-default"
    # no phoenix twin: the platform's preset for the session's mode
    assert platform_equivalent("spot-indoor", "phoenix", NOV11_TOPO) == "phoenix-default"
    assert platform_equivalent("spot_prior_dsg-prior_dsg", "phoenix", {}) is None


def test_nov11_launch_memory_resolves_gauss_to_its_phoenix_session():
    from dcist_launch_system.fleet.launch_workflow import robot_session

    plan = _nov11_plan()
    assert robot_session(plan, NOV11_RUNTIME["gauss"], NOV11_TOPO) == "phoenix_relocalize-relocalize_awcd"
    [(m, cmd)] = build_robot_commands(plan, NOV11_RUNTIME, OUT, NOV11_TOPO)
    assert m["name"] == "gauss" and "run-adt4 phoenix_relocalize-relocalize_awcd -n gauss -c phoenix" in cmd
    # the Spots keep the spot session
    plan.robots = ["euclid", "gauss"]
    cmds = {m["name"]: c for m, c in build_robot_commands(plan, NOV11_RUNTIME, OUT, NOV11_TOPO)}
    assert "run-adt4 spot_relocalize-relocalize_awcd -n euclid" in cmds["euclid"]


def test_session_of_another_platform_is_refused_dry_run_included():
    plan = _nov11_plan(sessions={"phoenix": "spot_relocalize-relocalize_awcd"})
    msg = "gauss (platform phoenix) would run spot_relocalize-relocalize_awcd (a spot session)"
    with pytest.raises(LaunchError, match=re.escape(msg)):
        build_robot_commands(plan, NOV11_RUNTIME, OUT, NOV11_TOPO)
    with pytest.raises(LaunchError, match=re.escape(msg)):
        dry_run_lines(plan, NOV11_RUNTIME, OUT, NOV11_TOPO)
    # a spot session with no phoenix equivalent, picked by hand
    plan = _nov11_plan(session="spot_prior_dsg-prior_dsg", mode=None, sessions={})
    with pytest.raises(LaunchError, match="gauss .platform phoenix. would run spot_prior_dsg-prior_dsg"):
        build_robot_commands(plan, NOV11_RUNTIME, OUT, {"machines": NOV11_TOPO["machines"]})
    plan = _nov11_plan(robots=["hamilton"], sessions={"spot": "phoenix-default"}, mode="mapping", prior_path="")
    with pytest.raises(LaunchError, match="hamilton .platform spot. would run phoenix-default"):
        build_robot_commands(plan, NOV11_RUNTIME, OUT, NOV11_TOPO)


def _robots_only_run(plan, ssh, rec, topo=NOV11_TOPO):
    plan.preflight = False
    return run_launch(
        plan, topo=topo, active_network="silvus", runtime_config=NOV11_RUNTIME,
        output_root=OUT, callbacks=rec, deps=make_deps(ssh),
    )  # fmt: skip


def test_robots_only_relaunch_never_touches_the_base_station():
    gauss, willow = "10.0.0.13", "10.0.0.9"
    ssh = FakeSsh({
        # gauss runs a session; willow's base session runs
        "show-environment": (0, "ADT4_OUTPUT_DIR=/home/swarm/adt4_output/reloc_awcd", "", gauss),
        "send-keys": (0, "", "", gauss),
        "pane_current_command": (0, "zsh\nzsh\n", "", gauss),
        "kill-session": (0, "", "", gauss),
        "_prev-": (0, "MOVED /home/swarm/adt4_output/reloc_awcd_prev-20261111-2200\n", "", gauss),
        "&& echo OK": (0, "OK", "", willow),
        "test -d": (0, "OK", "", gauss),
    })  # fmt: skip
    rec = Recorder(answers=[True])
    seen = []
    plan = _nov11_plan(robots_only=True, preflight=False)
    deps = make_deps(ssh)
    deps.deploy_zenoh = lambda *a: seen.append("zenoh") or (True, "x")
    assert run_launch(plan, topo=NOV11_TOPO, active_network="silvus", runtime_config=NOV11_RUNTIME,
                      output_root=OUT, callbacks=rec, deps=deps), rec.text()  # fmt: skip
    assert seen == []
    # willow: only read-only commands (is the base session running, the prior's identity)
    wcmds = ssh.cmds_for(willow)
    assert wcmds[0] == f"tmux has-session -t {TMUX_SESSION_NAME} 2>/dev/null && echo OK"
    assert not any(k in c for c in wcmds for k in ("kill-session", "send-keys", "nohup", "run-adt4", "rm ", "mv "))
    assert not any(ip not in (gauss, willow) for ip, _ in ssh.calls)
    cmds = ssh.cmds_for(gauss)
    i_stop = next(i for i, c in enumerate(cmds) if "send-keys" in c and "C-c" in c)
    i_kill = next(i for i, c in enumerate(cmds) if "kill-session" in c)
    i_move = next(i for i, c in enumerate(cmds) if "_prev-" in c)
    i_run = next(i for i, c in enumerate(cmds) if "run-adt4" in c)
    assert i_stop < i_kill < i_move < i_run
    assert not any("rm -rf" in c for c in cmds)
    run_cmd = cmds[i_run]
    assert "run-adt4 phoenix_relocalize-relocalize_awcd -n gauss" in run_cmd and " -f" not in run_cmd.split("--tmuxp")[0]
    [req] = rec.requests
    assert req.kind == "kill_sessions" and req.detail["machines"] == ["gauss"]
    assert "reloc_awcd_prev-20261111-2200" in rec.text()


def test_robots_only_warns_without_a_running_base_but_launches():
    ssh = FakeSsh({"&& echo OK": (1, "", "", "10.0.0.9"), "_prev-": (0, "", "", "10.0.0.13")})
    rec = Recorder()
    plan = _nov11_plan(robots_only=True, prior_path="", mode="mapping", session="phoenix-default")
    assert _robots_only_run(plan, ssh, rec), rec.text()
    assert "need the base station for planning" in rec.text()
    assert rec.requests == []  # nothing running on gauss: no confirm


def test_robots_only_prior_inside_the_moved_dir_follows_it():
    # gauss relocalizes on its own previous run, which is the output dir
    gauss = "10.0.0.13"
    ssh = FakeSsh({"_prev-": (0, "MOVED /h/adt4_output/e_prev-1\n", "", gauss), "&& echo OK": (0, "OK", "", "10.0.0.9")})
    rec = Recorder()
    plan = _nov11_plan(robots_only=True, experiment="e", prior_path=f"{OUT}/e", prior_source="gauss",
                       session="phoenix_relocalize-relocalize")  # fmt: skip
    assert _robots_only_run(plan, ssh, rec), rec.text()
    [run_cmd] = [c for c in ssh.cmds_for(gauss) if "run-adt4" in c]
    assert " -p /h/adt4_output/e_prev-1 " in run_cmd


def test_robots_only_dry_run_and_validation():
    from dcist_launch_system.fleet.launch_workflow import is_base_only, move_aside_command

    plan = _nov11_plan(robots_only=True)
    lines = dry_run_lines(plan, NOV11_RUNTIME, OUT, NOV11_TOPO)
    ssh_lines = [line for line in lines if line.startswith("ssh ")]
    assert len(ssh_lines) == 1 and ssh_lines[0].startswith('ssh swarm@10.0.0.13 "')
    assert any("Robots only: willow's session" in line for line in lines)
    assert not any("nohup" in line for line in lines)
    # robots only with no robots is not a base-only launch
    empty = _nov11_plan(robots_only=True, robots=[])
    assert not is_base_only(empty)
    with pytest.raises(LaunchError, match="No robots"):
        dry_run_lines(empty, NOV11_RUNTIME, OUT, NOV11_TOPO)
    cmd = move_aside_command("~/adt4_output/e")
    assert cmd.startswith("d=~/'adt4_output/e'; ") or cmd.startswith("d=~/adt4_output/e; ")
    assert '[ ! -e "$d/runs" ]' in cmd and "rm " not in cmd


def test_robots_only_move_aside_runs_in_a_shell(tmp_path):
    import subprocess

    from dcist_launch_system.fleet.launch_workflow import move_aside_command

    d = tmp_path / "e"
    (d / "hydra").mkdir(parents=True)
    out = subprocess.run(["bash", "-c", move_aside_command(str(d))], capture_output=True, text=True).stdout
    assert out.startswith("MOVED ") and not d.exists()
    moved = out.split(" ", 1)[1].strip()
    assert (tmp_path / moved.rsplit("/", 1)[1] / "hydra").is_dir()
    # per-run layout: left for run-adt4 to add a run
    (d / "runs").mkdir(parents=True)
    out = subprocess.run(["bash", "-c", move_aside_command(str(d))], capture_output=True, text=True).stdout
    assert out == "" and (d / "runs").is_dir()


# --- platform bring-up (machines.<robot>.bringup) -------------------------------

BRINGUP = {
    "session": "husky",
    "command": "ROBOT_NAME=smaug tmuxp load -d ~/husky.yaml",
    "ready_topic": "/smaug/dlio/odom_node/odom",
    "ready_desc": "DLIO odometry",
    "timeout_s": 30,
}


def _bringup_topo(entry=BRINGUP):
    topo = dict(TOPO)
    topo["machines"] = {"smaug": {"role": "robot", "bringup": dict(entry)}}
    return topo


class FakeClock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def sleep(self, s):
        self.t += s


def run_bringup(plan, ssh, rec, entry=BRINGUP, clock=None):
    clock = clock or FakeClock()
    deps = make_deps(ssh)
    deps.clock = clock
    deps.sleep = clock.sleep
    plan.preflight = False
    return run_launch(
        plan,
        topo=_bringup_topo(entry),
        active_network="silvus",
        runtime_config=RUNTIME,
        output_root=OUT,
        callbacks=rec,
        deps=deps,
    )


class ReadyAfter(FakeSsh):
    """The husky session is not running; the ready topic publishes after ``n`` probes."""

    def __init__(self, n, session_running=False, start_rc=0, positions=None):
        super().__init__()
        self.n = n
        self.positions = list(positions or [])
        self.started = session_running
        self.start_rc = start_rc

    def __call__(self, user, ip, cmd, timeout=5):
        self.calls.append((ip, cmd))
        if "tmux has-session -t husky" in cmd:
            return (0 if self.started else 1), "", ""
        if "tmuxp load -d" in cmd:
            self.started = self.start_rc == 0
            return self.start_rc, "", "boom" if self.start_rc else ""
        if "--field pose.pose.position" in cmd:
            p = self.positions.pop(0) if self.positions else (0.0, 0.0, 0.0)
            if p is None:
                return 1, "", ""
            return 0, "x: %f\ny: %f\nz: %f\n---" % p, ""
        if "ros2 topic echo --once" in cmd:
            self.n -= 1
            return (0 if self.n < 0 else 1), "", ""
        if "_prev-" in cmd:  # robots-only move-aside: nothing to move
            return 0, "", ""
        return 0 if "run-adt4" in cmd else 1, "", ""


def _order(ssh, *needles):
    idx = []
    for needle in needles:
        idx.append(next(i for i, (_, c) in enumerate(ssh.calls) if needle in c))
    return idx


def test_bringup_starts_stack_and_waits_for_ready_topic_before_run_adt4():
    ssh = ReadyAfter(3)
    rec = Recorder()
    plan = LaunchPlan(robots=["smaug"], session="spot-default", experiment="e1")
    assert run_bringup(plan, ssh, rec)
    start, first_probe, launch = _order(ssh, "tmuxp load -d", "ros2 topic echo --once", "run-adt4")
    assert start < first_probe < launch
    assert sum("ros2 topic echo --once" in c for _, c in ssh.calls) == 4
    assert "smaug: DLIO odometry is up" in rec.text()
    assert not rec.requests


def test_bringup_leaves_running_session_alone():
    ssh = ReadyAfter(0, session_running=True)
    rec = Recorder()
    plan = LaunchPlan(robots=["smaug"], session="spot-default", experiment="e1")
    assert run_bringup(plan, ssh, rec)
    assert not any("tmuxp load -d" in c for _, c in ssh.calls)
    assert "'husky' already running, left alone" in rec.text()


def test_bringup_timeout_asks_and_skips_robot_on_no():
    ssh = ReadyAfter(10**6)
    rec = Recorder(answers=[False])
    plan = LaunchPlan(robots=["smaug"], session="spot-default", experiment="e1")
    assert not run_bringup(plan, ssh, rec)
    assert [r.kind for r in rec.requests] == ["bringup_not_ready"]
    assert "DLIO odometry did not come up" in rec.requests[0].message
    assert not any("run-adt4" in c for _, c in ssh.calls)


def test_bringup_timeout_launch_anyway():
    ssh = ReadyAfter(10**6)
    rec = Recorder(answers=[True])
    plan = LaunchPlan(robots=["smaug"], session="spot-default", experiment="e1")
    assert run_bringup(plan, ssh, rec)
    assert any("run-adt4" in c for _, c in ssh.calls)


def test_bringup_start_failure_asks():
    ssh = ReadyAfter(0, start_rc=1)
    rec = Recorder(answers=[None])
    plan = LaunchPlan(robots=["smaug"], session="spot-default", experiment="e1")
    assert not run_bringup(plan, ssh, rec)
    assert "could not be started" in rec.requests[0].message
    assert not any("ros2 topic echo" in c for _, c in ssh.calls)
    assert not any("run-adt4" in c for _, c in ssh.calls)


def test_bringup_only_for_robots_with_an_entry():
    ssh = ReadyAfter(0)
    rec = Recorder()
    plan = LaunchPlan(robots=["topaz"], session="spot-default", experiment="e1")
    assert run_bringup(plan, ssh, rec)
    assert not any("tmuxp load" in c or "topic echo" in c for _, c in ssh.calls)


def test_bringup_in_robots_only_relaunch():
    ssh = ReadyAfter(1)
    rec = Recorder()
    plan = LaunchPlan(
        robots=["smaug"], session="spot-default", experiment="e1",
        base_station="willow", robots_only=True,
    )  # fmt: skip
    assert run_bringup(plan, ssh, rec)
    start, launch = _order(ssh, "tmuxp load -d", "run-adt4")
    assert start < launch


def test_bringup_dry_run_lines():
    plan = LaunchPlan(robots=["smaug"], session="spot-default", experiment="e1")
    lines = dry_run_lines(plan, RUNTIME, OUT, _bringup_topo())
    text = "\n".join(lines)
    assert "smaug: start husky unless running: ROBOT_NAME=smaug tmuxp load -d" in text
    assert "wait up to 30 s for /smaug/dlio/odom_node/odom before run-adt4" in text
    assert text.index("tmuxp load -d") < text.index("run-adt4")


def test_gauss_bringup_in_topology():
    import pathlib

    import yaml

    topo = yaml.safe_load(
        (pathlib.Path(__file__).resolve().parents[2] / "network_diagnostics/config/topology.yaml").read_text()
    )
    b = topo["machines"]["gauss"]["bringup"]
    assert b["session"] == "husky"
    assert "tmuxp load -d" in b["command"] and "ROBOT_NAME=gauss" in b["command"]
    assert b["ready_topic"] == "/gauss/dlio/odom_node/odom"


SETTLE = dict(BRINGUP, settle_s=15, settle_max_move_m=0.3, settle_max_z_m=1.0)


def test_bringup_settle_waits_and_checks_the_estimate_held():
    ssh = ReadyAfter(0, positions=[(0.0, 0.0, 0.0), (0.05, 0.02, 0.01)])
    rec = Recorder()
    clock = FakeClock()
    plan = LaunchPlan(robots=["smaug"], session="spot-default", experiment="e1")
    assert run_bringup(plan, ssh, rec, entry=SETTLE, clock=clock)
    assert clock.t >= 15
    probes = [i for i, (_, c) in enumerate(ssh.calls) if "--field pose.pose.position" in c]
    launch = next(i for i, (_, c) in enumerate(ssh.calls) if "run-adt4" in c)
    assert len(probes) == 2 and probes[-1] < launch
    assert "DLIO odometry stable (moved 0.05 m in 15 s" in rec.text()
    assert not rec.requests


def test_bringup_settle_drift_asks():
    ssh = ReadyAfter(0, positions=[(0.0, 0.0, 0.0), (0.4, 0.0, -1.2)])
    rec = Recorder(answers=[False])
    plan = LaunchPlan(robots=["smaug"], session="spot-default", experiment="e1")
    assert not run_bringup(plan, ssh, rec, entry=SETTLE)
    assert [r.kind for r in rec.requests] == ["bringup_not_ready"]
    assert "not stable while still" in rec.requests[0].message
    assert not any("run-adt4" in c for _, c in ssh.calls)


def test_bringup_settle_z_limit_alone_asks():
    ssh = ReadyAfter(0, positions=[(0.0, 0.0, -1.5), (0.0, 0.0, -1.5)])
    rec = Recorder(answers=[True])
    plan = LaunchPlan(robots=["smaug"], session="spot-default", experiment="e1")
    assert run_bringup(plan, ssh, rec, entry=SETTLE)
    assert "z -1.50" in rec.requests[0].message
    assert any("run-adt4" in c for _, c in ssh.calls)


def test_bringup_settle_unreadable_asks():
    ssh = ReadyAfter(0, positions=[None, None])
    rec = Recorder(answers=[None])
    plan = LaunchPlan(robots=["smaug"], session="spot-default", experiment="e1")
    assert not run_bringup(plan, ssh, rec, entry=SETTLE)
    assert "could not be read" in rec.requests[0].message


def test_bringup_no_settle_when_stack_already_running():
    ssh = ReadyAfter(0, session_running=True)
    rec = Recorder()
    clock = FakeClock()
    plan = LaunchPlan(robots=["smaug"], session="spot-default", experiment="e1")
    assert run_bringup(plan, ssh, rec, entry=SETTLE, clock=clock)
    assert clock.t < 15
    assert not any("--field pose.pose.position" in c for _, c in ssh.calls)
    assert "no settle wait" in rec.text()


def test_parse_position():
    from dcist_launch_system.fleet.launch_workflow import parse_position

    assert parse_position("x: 1.5\ny: -2.0\nz: 0.25\n---\n") == (1.5, -2.0, 0.25)
    assert parse_position("") is None
    assert parse_position("x: 1\ny: nan-ish\nz: 0") is None


def test_gauss_settle_in_topology():
    import pathlib

    import yaml

    topo = yaml.safe_load(
        (pathlib.Path(__file__).resolve().parents[2] / "network_diagnostics/config/topology.yaml").read_text()
    )
    b = topo["machines"]["gauss"]["bringup"]
    assert b["settle_s"] == 15
