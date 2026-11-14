"""Tests for the map-verification and Silvus link-matrix helpers."""

from dcist_launch_system.fleet.maps import compare_hashes, is_prior_map
from dcist_launch_system.fleet.silvus import link_matrix, routing_summary


def test_is_prior_map():
    assert is_prior_map({"subdirs": ["hydra", "roman", "agents"]})
    assert not is_prior_map({"subdirs": ["hydra"]})
    assert not is_prior_map({})


def test_compare_hashes():
    rows = compare_hashes(
        {
            "a": {"dsg.json": "aaaa1111bbbb", "mesh.ply": "m1", "only_a.csv": "x"},
            "b": {"dsg.json": "aaaa1111bbbb", "mesh.ply": "m2xxxxxxxxx"},
        }
    )
    by_path = {r.path: r for r in rows}
    assert by_path["dsg.json"].consistent and by_path["dsg.json"].cell("b") == "OK"
    assert not by_path["mesh.ply"].consistent
    assert by_path["mesh.ply"].cell("b") == "m2xxxxxx"
    assert not by_path["only_a.csv"].consistent
    assert by_path["only_a.csv"].cell("b") == "MISSING"


DETAILS = {
    "r1": {
        "node_id": 101,
        "link_stats": {"102": -55.0, "103": -85},
        "routing_tree": [102, 103],
        "battery": 80.0,
    },
    "r2": {"node_id": 102, "link_stats": {}, "routing_tree": [101]},
    "r3": {"node_id": 103, "error": "timeout"},
}


def test_link_matrix():
    m = link_matrix(DETAILS)
    assert m["r3"] == "timeout"
    assert m["r1"]["r2"].quality == "good" and m["r1"]["r2"].rssi_db == -55.0
    assert m["r1"]["r3"].quality == "poor"
    assert m["r2"]["r1"].quality == "linked"
    assert m["r2"]["r3"].quality == "unknown"


def test_link_matrix_two_radio_rssi_fallback():
    m = link_matrix(
        {
            "a": {"node_id": 1, "rssi": [-130]},
            "b": {"node_id": 2, "rssi": [-100]},
        }
    )
    assert m["a"]["b"].rssi_db == -65.0 and m["a"]["b"].quality == "fair"


def test_routing_summary_resolves_names():
    s = routing_summary(DETAILS)
    assert set(s) == {"r1", "r2"}
    assert s["r1"]["routes"] == ["r2", "r3"] and s["r1"]["battery"] == 80.0


# --- StreamScape 5 radios (no rssi/link_stats/snr_stats; network_status instead) ---

from dcist_launch_system import fleet_helpers as fh  # noqa: E402


def test_parse_network_status():
    assert fh.parse_network_status(["65631", "310308", "51"]) == {310308: 51.0}
    assert fh.parse_network_status(["1", "2", "40", "3", "8"]) == {2: 40.0, 3: 8.0}
    assert fh.parse_network_status(None) == {} and fh.parse_network_status(["1"]) == {}


def test_link_matrix_uses_network_status_when_there_is_no_rssi():
    m = link_matrix(
        {
            "a": {"node_id": 1, "routing_tree": [2, 1], "link_quality": {2: 36.0}},
            "b": {"node_id": 2, "routing_tree": [1, 2], "link_quality": {1: 9.0}},
        }
    )
    assert m["a"]["b"].link_db == 36.0 and m["a"]["b"].quality == "good"
    assert m["b"]["a"].quality == "poor" and m["b"]["a"].rssi_db is None


def _radios_topo():
    return {"silvus_radios": {"radios": {
        "radio_a": {"mgmt_ip": "192.0.2.95", "node_id": 65631},
        "radio_b": {"mgmt_ip": "192.0.2.36", "node_id": 310308},
    }}}  # fmt: skip


def _answer(nid, nbr):
    return [{"id": 1, "result": [str(nid)]}, {"id": 2, "result": [nbr, nid]}, {"id": 3, "result": ["97.5"]},
            {"id": 4, "error": {"code": -33000}}, {"id": 7, "result": [str(nid), str(nbr), "36"]}]  # fmt: skip


def test_radios_unreachable_by_mgmt_ip_are_asked_through_the_dongle(monkeypatch):
    calls = []
    lls = {"fe80::1": 65631, "fe80::2": 310308}

    def rpc(host, payload, timeout=3.0, iface=None):
        calls.append((host, iface))
        if iface is None:
            raise OSError("timed out")  # no route to the mgmt subnet
        nid = lls[host]
        if isinstance(payload, dict):  # node id probe during discovery
            return {"result": [str(nid)]}
        return _answer(nid, 310308 if nid == 65631 else 65631)

    monkeypatch.setattr(fh, "silvus_rpc", rpc)
    monkeypatch.setattr(fh, "silvus_interface", lambda topo: "enx0")
    monkeypatch.setattr(fh, "silvus_link_locals", lambda iface: list(lls))
    monkeypatch.setattr(
        fh, "_SILVUS_LL_CACHE", {"t": 0.0, "iface": None, "by_node": {}}
    )
    d = fh.get_silvus_radio_details(_radios_topo())
    assert d["radio_a"]["error"] is None and d["radio_a"]["via"] == "fe80::1%enx0"
    assert (
        d["radio_a"]["link_quality"] == {310308: 36.0}
        and d["radio_a"]["battery"] == 97.5
    )
    assert d["radio_b"]["via"] == "fe80::2%enx0"
    # next refresh goes straight to the dongle (no mgmt-IP timeouts)
    calls.clear()
    fh.get_silvus_radio_details(_radios_topo())
    assert all(iface == "enx0" for _, iface in calls)


def test_radio_missing_everywhere_reports_why(monkeypatch):
    monkeypatch.setattr(
        fh, "silvus_rpc", lambda *a, **k: (_ for _ in ()).throw(OSError("timed out"))
    )
    monkeypatch.setattr(fh, "silvus_interface", lambda topo: None)
    monkeypatch.setattr(
        fh, "_SILVUS_LL_CACHE", {"t": 0.0, "iface": None, "by_node": {}}
    )
    d = fh.get_silvus_radio_details(_radios_topo())
    assert "192.0.2.95: timed out" in d["radio_a"]["error"]
    assert "radio dongle unplugged" in d["radio_a"]["error"]


def test_mesh_view_names_hosts_and_pairs_links():
    from dcist_launch_system.fleet.silvus import mesh_view

    details = {
        "r_w": {"node_id": 1, "routing_tree": [2], "link_quality": {2: 40.0},
                "clients": [{"mac": "30:68", "ip": "203.0.113.100"}], "battery": 95.0, "via": "fe80::1%e"},
        "r_h": {"node_id": 2, "routing_tree": [1], "link_quality": {1: 38.0},
                "clients": [{"mac": "20:23", "ip": "203.0.113.2"}]},
        "r_x": {"node_id": 3, "error": "timed out"},
    }  # fmt: skip
    topo = {"machines": {"willow": {"addresses": {"silvus": "203.0.113.100"}},
                         "hamilton": {"addresses": {"silvus": "203.0.113.2"}}}}  # fmt: skip
    v = mesh_view(details, topo, local_ips=["203.0.113.100", "198.51.100.250"])
    radios = {r["name"]: r for r in v["radios"]}
    assert (
        radios["r_w"]["hosts"] == [{"ip": "203.0.113.100", "machine": "willow"}]
        and radios["r_w"]["local"]
    )
    assert (
        radios["r_h"]["hosts"][0]["machine"] == "hamilton"
        and not radios["r_h"]["local"]
    )
    assert radios["r_x"]["error"] == "timed out"
    assert v["links"] == [
        {
            "a": "r_h",
            "b": "r_w",
            "ab": 38.0,
            "ba": 40.0,
            "unit": "dB",
            "quality": "good",
        }
    ]
