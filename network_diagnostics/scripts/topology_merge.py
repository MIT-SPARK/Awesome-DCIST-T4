#!/usr/bin/env python3
"""Merge the public fleet topology with the private fleet-address overlay.

``network_diagnostics/config/topology.yaml`` is public: it names the machines,
their roles and platforms, the networks (in detection-priority order), ports
and fleet defaults.  The fleet's real addresses live in a private overlay file
with the same layout, which this module deep-merges over the public file.
The overlay holds only these keys (see ``PRIVATE_KEYS``)::

    networks.<net>.subnet
    machines.<machine>.addresses
    silvus_radios.mgmt_subnet
    silvus_radios.radios.<radio>.{mgmt_ip,node_id}

The overlay is found, in order: the ``addresses_path`` argument, then
``$ADT4_FLEET_ADDRESSES``, then ``~/.config/adt4/fleet_addresses/fleet_addresses.yaml``
if it exists.  An explicitly named overlay (argument or env var) that does not
exist is an error; no overlay at all is not, and the public topology is
returned as is (callers should warn: every address lookup will come up empty).

Standard library + PyYAML only, so shell scripts can call the CLI::

    topology_merge.py [--topology P] [--addresses P] --out FILE   # write merged YAML (0600), print FILE
    topology_merge.py [--topology P] [--addresses P] --check      # exit 1 if no overlay is found
    topology_merge.py [--topology P] [--addresses P]              # merged YAML to stdout
"""

from __future__ import annotations

import argparse
import copy
import os
import pathlib
import sys
import tempfile

import yaml

ENV_VAR = "ADT4_FLEET_ADDRESSES"

DEFAULT_TOPOLOGY_PATH = (
    pathlib.Path(__file__).resolve().parent.parent / "config" / "topology.yaml"
)

DEFAULT_ADDRESSES_PATH = pathlib.Path(
    "~/.config/adt4/fleet_addresses/fleet_addresses.yaml"
)

# Private keys, as (section, per-entry?, keys).  A per-entry section is a map of
# named entries (machines, networks, radios) whose listed keys are private.
_PRIVATE_ENTRY_KEYS = {
    ("networks",): {"subnet"},
    ("machines",): {"addresses"},
    ("silvus_radios", "radios"): {"mgmt_ip", "node_id"},
}
_PRIVATE_SCALAR_KEYS = {
    ("silvus_radios",): {"mgmt_subnet"},
}

PRIVATE_KEYS = (
    "networks.<net>.subnet",
    "machines.<machine>.addresses",
    "silvus_radios.mgmt_subnet",
    "silvus_radios.radios.<radio>.mgmt_ip",
    "silvus_radios.radios.<radio>.node_id",
)

NO_OVERLAY_MESSAGE = (
    "No fleet address overlay found: set $"
    + ENV_VAR
    + " or clone robustrobotics/adt4_fleet_addresses to "
    "~/.config/adt4/fleet_addresses. Machine addresses and network subnets "
    "are not in the public topology.yaml."
)


def deep_merge(base, overlay):
    """Return ``base`` deep-merged with ``overlay``; overlay values win.

    Dicts merge recursively (base key order first, then keys only the overlay
    has); anything else in the overlay replaces the base value.  Inputs are not
    modified.
    """
    if not isinstance(base, dict) or not isinstance(overlay, dict):
        return copy.deepcopy(overlay)
    merged = copy.deepcopy(base)
    for key, value in overlay.items():
        if key in merged and isinstance(merged[key], dict) and isinstance(value, dict):
            merged[key] = deep_merge(merged[key], value)
        else:
            merged[key] = copy.deepcopy(value)
    return merged


def find_addresses_path(addresses_path=None):
    """Return the overlay path to use, or None if there is none.

    Raises FileNotFoundError if the path was named explicitly (argument or
    ``$ADT4_FLEET_ADDRESSES``) and does not exist.
    """
    if addresses_path:
        path = pathlib.Path(addresses_path).expanduser()
        source = "addresses path"
    elif os.environ.get(ENV_VAR):
        path = pathlib.Path(os.environ[ENV_VAR]).expanduser()
        source = "$" + ENV_VAR
    else:
        path = DEFAULT_ADDRESSES_PATH.expanduser()
        return path if path.is_file() else None
    if not path.is_file():
        raise FileNotFoundError(f"Fleet address overlay not found ({source}): {path}")
    return path


def _load_yaml(path):
    with open(path) as f:
        data = yaml.safe_load(f)
    return data if data is not None else {}


def load_merged_topology(topology_path=None, addresses_path=None):
    """Load topology.yaml and deep-merge the private address overlay over it.

    Returns the public topology unchanged if no overlay is found (see
    ``find_addresses_path``).
    """
    topo = _load_yaml(
        pathlib.Path(topology_path) if topology_path else DEFAULT_TOPOLOGY_PATH
    )
    overlay_path = find_addresses_path(addresses_path)
    if overlay_path is None:
        return topo
    return deep_merge(topo, _load_yaml(overlay_path))


def _get(d, keys):
    for k in keys:
        if not isinstance(d, dict) or not isinstance(d.get(k), dict):
            return None
        d = d[k]
    return d


def _set(d, keys, value):
    for k in keys[:-1]:
        d = d.setdefault(k, {})
    d[keys[-1]] = value


def split_private(topo):
    """Split a merged topology into ``(public, private)`` dicts.

    ``private`` holds only the keys in ``PRIVATE_KEYS``; ``public`` is
    everything else.  Radio entries left empty in ``public`` are dropped (their
    names carry node ids), and so is ``radios`` if that empties it.
    ``deep_merge(public, private) == topo``.
    """
    public = copy.deepcopy(topo) if topo else {}
    private = {}

    for section, keys in _PRIVATE_SCALAR_KEYS.items():
        container = _get(public, section)
        if container is None:
            continue
        for key in keys:
            if key in container:
                _set(private, list(section) + [key], container.pop(key))

    for section, keys in _PRIVATE_ENTRY_KEYS.items():
        entries = _get(public, section)
        if entries is None:
            continue
        is_radios = section == ("silvus_radios", "radios")
        for name in list(entries):
            entry = entries[name]
            if not isinstance(entry, dict):
                continue
            moved = {k: entry.pop(k) for k in list(entry) if k in keys}
            if moved:
                _set(private, list(section) + [name], moved)
                if is_radios and not entry:
                    del entries[name]
        if is_radios and not entries and _get(private, section):
            del public["silvus_radios"]["radios"]

    return public, private


def _write_yaml_private(data, out_path):
    """Write YAML atomically with mode 0600 (replacing, never following, a link)."""
    out_path = pathlib.Path(out_path).expanduser()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=out_path.parent, prefix=f".{out_path.name}.")
    try:
        os.fchmod(fd, 0o600)
        with os.fdopen(fd, "w") as f:
            yaml.safe_dump(data, f, default_flow_style=False, sort_keys=False)
        os.replace(tmp, out_path)
    except BaseException:
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise
    return out_path


def save_split(topo, topology_path, addresses_path=None):
    """Write ``topo``'s public part to ``topology_path`` and its private part to
    ``addresses_path`` (0600).  With no ``addresses_path`` the private part is
    not written anywhere.  Returns ``(public, private)``.
    """
    public, private = split_private(topo)
    with open(topology_path, "w") as f:
        yaml.safe_dump(public, f, default_flow_style=False, sort_keys=False)
    if addresses_path:
        _write_yaml_private(private, addresses_path)
    return public, private


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Merge topology.yaml with the private fleet address overlay."
    )
    parser.add_argument(
        "--topology",
        default=None,
        help=f"public topology (default: {DEFAULT_TOPOLOGY_PATH})",
    )
    parser.add_argument(
        "--addresses",
        default=None,
        help=f"address overlay (default: ${ENV_VAR}, then {DEFAULT_ADDRESSES_PATH})",
    )
    parser.add_argument(
        "--out",
        default=None,
        help="write the merged YAML here (mode 0600) and print its path",
    )
    parser.add_argument(
        "--check", action="store_true", help="exit 1 if no address overlay is found"
    )
    args = parser.parse_args(argv)

    try:
        overlay_path = find_addresses_path(args.addresses)
        if overlay_path is None:
            if args.check:
                print(f"ERROR: {NO_OVERLAY_MESSAGE}", file=sys.stderr)
                return 1
            print(f"WARNING: {NO_OVERLAY_MESSAGE}", file=sys.stderr)
        merged = load_merged_topology(args.topology, overlay_path)
    except (OSError, yaml.YAMLError) as e:
        print(f"ERROR: {e}", file=sys.stderr)
        return 1

    if args.out:
        print(_write_yaml_private(merged, args.out))
    elif args.check:
        print(overlay_path)
    else:
        yaml.safe_dump(merged, sys.stdout, default_flow_style=False, sort_keys=False)
    return 0


if __name__ == "__main__":
    sys.exit(main())
