from dcist_launch_system.tui.context import TuiContext


def test_construction(sample_topo):
    ctx = TuiContext(
        topo=sample_topo,
        active_network="mit_wifi",
        topology_path=None,
        output_root="~/adt4_output",
    )
    assert ctx.active_network == "mit_wifi"
    assert ctx.runtime_config == {}
    assert ctx._deleted_machines == set()
    assert ctx._fleet_zenoh == {"proc": None, "config": None}


def test_fuzzy_match_basic():
    assert TuiContext.fuzzy_match("abc", "aXbYcZ") is True
    assert TuiContext.fuzzy_match("abc", "aXbY") is False
    assert TuiContext.fuzzy_match("", "anything") is True
    assert TuiContext.fuzzy_match("ABC", "aXbYcZ") is True  # case-insensitive


def test_fuzzy_match_exact():
    assert TuiContext.fuzzy_match("euclid", "euclid") is True
    assert TuiContext.fuzzy_match("euc", "euclid") is True
    assert TuiContext.fuzzy_match("xyz", "euclid") is False


def test_runtime_config_mutation(ctx):
    ctx.runtime_config["alpha"]["online"] = True
    assert ctx.runtime_config["alpha"]["online"] is True


def test_deleted_machines_mutation(ctx):
    ctx._deleted_machines.add("alpha")
    assert "alpha" in ctx._deleted_machines


def test_fleet_zenoh_default(ctx):
    assert ctx._fleet_zenoh["proc"] is None
    assert ctx._fleet_zenoh["config"] is None
