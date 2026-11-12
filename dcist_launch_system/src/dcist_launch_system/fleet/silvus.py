"""Silvus radio link-quality matrix and mesh routing, from ``get_silvus_radio_details``."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class LinkCell:
    rssi_db: float | None = None  # measured RSSI (older firmware: rssi / link_stats)
    linked: bool = False  # in the routing tree
    # StreamScape 5 network_status value for this neighbour (SNR-like, dB); the
    # firmware has no rssi / link_stats / snr_stats
    link_db: float | None = None

    @property
    def quality(self) -> str:
        """good | fair | poor | linked | unknown"""
        if self.rssi_db is not None:
            if self.rssi_db > -60:
                return "good"
            return "fair" if self.rssi_db > -80 else "poor"
        if self.link_db is not None:
            if self.link_db >= 25:
                return "good"
            return "fair" if self.link_db >= 12 else "poor"
        return "linked" if self.linked else "unknown"


def node_names(details: dict) -> dict[int, str]:
    """node_id -> radio name."""
    return {
        int(info["node_id"]): name
        for name, info in details.items()
        if info.get("node_id")
    }


def link_matrix(details: dict) -> dict[str, dict[str, LinkCell] | str]:
    """``{src: {dst: LinkCell}}``; an unreachable ``src`` maps to its error string."""
    names = sorted(details)
    matrix = {}
    for src in names:
        info = details[src]
        if info.get("error"):
            matrix[src] = info["error"]
            continue
        link_stats = info.get("link_stats")
        rssi_raw = info.get("rssi")
        link_quality = info.get("link_quality") or {}
        routing_tree = [int(x) for x in (info.get("routing_tree") or [])]
        row = {}
        for dst in names:
            if dst == src:
                continue
            dst_nid = details[dst].get("node_id")
            val = None
            # link_stats is keyed by the neighbour's node_id string
            if isinstance(link_stats, dict) and dst_nid is not None:
                try:
                    val = float(
                        link_stats.get(str(dst_nid))
                        or link_stats.get(str(int(dst_nid)))
                    )
                except (TypeError, ValueError):
                    pass
            # Fallback: rssi field (single-neighbour case), in half-dB units
            if (
                val is None
                and isinstance(rssi_raw, list)
                and rssi_raw
                and len(names) == 2
            ):
                try:
                    val = float(rssi_raw[0]) / 2.0
                except (TypeError, ValueError):
                    pass
            linked = dst_nid is not None and int(dst_nid) in routing_tree
            link_db = link_quality.get(int(dst_nid)) if dst_nid is not None else None
            row[dst] = LinkCell(val, linked, link_db)
        matrix[src] = row
    return matrix


def routing_summary(details: dict) -> dict[str, dict]:
    """Per reachable radio: node id, battery and routing tree resolved to names."""
    nid_to_name = node_names(details)
    return {
        name: {
            "node_id": info.get("node_id"),
            "via": info.get("via"),
            "battery": info.get("battery"),
            "routes": [
                nid_to_name.get(int(n), str(n)) for n in info.get("routing_tree") or []
            ],
        }
        for name, info in sorted(details.items())
        if not info.get("error")
    }


def mesh_view(details: dict, topology: dict | None = None, local_ips=()) -> dict:
    """Radios and links for drawing the mesh.

    ``radios``: name, node_id, battery, via, error, ``hosts`` (the computers on
    its ethernet port, named from topology addresses) and ``local`` (one of them
    is this machine). ``links``: one per radio pair either side reports, with
    both directions' values and the best quality.
    """
    by_ip = {}
    for mname, m in ((topology or {}).get("machines") or {}).items():
        for addr in ((m or {}).get("addresses") or {}).values():
            by_ip.setdefault(str(addr), mname)
    local = {str(ip) for ip in local_ips}
    radios = []
    for name, info in sorted(details.items()):
        hosts = [
            {"ip": c.get("ip"), "machine": by_ip.get(str(c.get("ip")))}
            for c in info.get("clients") or []
        ]
        radios.append({
            "name": name,
            "node_id": info.get("node_id"),
            "battery": info.get("battery"),
            "via": info.get("via"),
            "error": info.get("error"),
            "hosts": hosts,
            "local": any(h["ip"] in local for h in hosts),
        })  # fmt: skip
    matrix = link_matrix(details)
    rank = {"good": 4, "fair": 3, "poor": 2, "linked": 1, "unknown": 0}
    links, names = [], sorted(details)
    for i, a in enumerate(names):
        for b in names[i + 1 :]:
            ab = matrix[a].get(b) if isinstance(matrix.get(a), dict) else None
            ba = matrix[b].get(a) if isinstance(matrix.get(b), dict) else None
            cells = [c for c in (ab, ba) if c is not None]
            if not any(c.linked or c.link_db is not None or c.rssi_db is not None for c in cells):
                continue
            best = max(cells, key=lambda c: rank[c.quality])
            def val(c):
                return None if c is None else (c.rssi_db if c.rssi_db is not None else c.link_db)
            links.append({
                "a": a, "b": b,
                "ab": val(ab), "ba": val(ba),
                "unit": "dBm" if any(c.rssi_db is not None for c in cells) else "dB",
                "quality": best.quality,
            })  # fmt: skip
    return {"radios": radios, "links": links}
