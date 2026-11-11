"""Fleet link environment for one machine, derived from topology.yaml.

    python -m dcist_launch_system.fleet_env <machine_name> [--network NAME]
        [--mode router|fleetlink] [--topology PATH] [--check]

prints shell ``export`` lines for:

``ADT4_FLEETLINK_MODE``
    ``router`` (rmw_zenohd router-to-router across the radio) or ``fleetlink``
    (only fleetlink crosses the radio), from ``fleet_defaults.fleet_link``.
``ADT4_FLEETLINK_CONNECT``
    where this machine's fleetlink processes connect (comma-separated zenoh
    endpoints). A robot in fleetlink mode gets the fleetlink base's address on each
    network of ``zenoh.fleetlink_networks``, in failover order (``--network`` keeps
    only that network's). A base station in fleetlink mode gets its local fleetlink
    router; router mode gets the local rmw_zenohd router.
``ADT4_FLEETLINK_PORT``
    ``zenoh.fleetlink_port`` (the fleetlink router's listen port, default 7450).

It never fails: any problem (no topology, unknown machine, base unreachable on every
fleetlink network) prints a warning on stderr and the router-mode values, so a launch
always goes ahead the way it did before fleetlink mode existed.

``--check`` instead lists the topology's problems (fleet_helpers.validate_topology) and
exits 1 when there are any.

``ADT4_FLEET_LINK`` is the old name of ``ADT4_FLEETLINK_MODE``; :func:`mode_from_env`
still reads it when the new name is unset.
"""

import argparse
import os
import shlex
import sys

from dcist_launch_system import fleet_helpers as fh

ROUTER_MODE_PORT = 7447
MODE_VAR = "ADT4_FLEETLINK_MODE"
LEGACY_MODE_VAR = "ADT4_FLEET_LINK"
FLEET_VARS = (MODE_VAR, "ADT4_FLEETLINK_CONNECT", "ADT4_FLEETLINK_PORT")


def mode_from_env(environ=None):
    """The fleet link mode set in the environment (``ADT4_FLEETLINK_MODE``, else the
    legacy ``ADT4_FLEET_LINK``), or None when neither is set."""
    environ = os.environ if environ is None else environ
    return environ.get(MODE_VAR) or environ.get(LEGACY_MODE_VAR) or None


def router_env(topology=None):
    port = ((topology or {}).get("zenoh") or {}).get("port", ROUTER_MODE_PORT)
    return {
        MODE_VAR: "router",
        "ADT4_FLEETLINK_CONNECT": f"tcp/127.0.0.1:{port}",
        "ADT4_FLEETLINK_PORT": str(fh.fleetlink_port(topology)),
    }


def fleet_env(machine_name, topology=None, network=None, warn=None, mode=None):
    """Return ``{ADT4_FLEETLINK_MODE, ADT4_FLEETLINK_CONNECT, ADT4_FLEETLINK_PORT}``.

    ``mode`` ("router" | "fleetlink") overrides ``fleet_defaults.fleet_link`` so a
    forced mode still gets matching endpoints. ``warn(msg)`` receives the reason
    whenever router mode is used as a fallback.
    """
    warn = warn or (lambda msg: None)
    try:
        if topology is None:
            topology = fh.load_topology()
        env = router_env(topology)
        if mode not in (None, "", *fh.FLEET_LINK_MODES):
            warn(f"unknown fleet link mode {mode!r}; using the topology's")
            mode = None
        if (mode or fh.fleet_link_mode(topology)) != "fleetlink":
            return env
        machines = topology.get("machines") or {}
        if machine_name not in machines:
            warn(f"{machine_name!r} is not in topology.yaml; using router mode")
            return env
        port = fh.fleetlink_port(topology)
        if (machines[machine_name] or {}).get("role") == "base_station":
            return {
                **env,
                MODE_VAR: "fleetlink",
                "ADT4_FLEETLINK_CONNECT": f"tcp/127.0.0.1:{port}",
            }
        endpoints = fh.generate_fleetlink_endpoints(topology)
        if network:
            only = fh.generate_fleetlink_endpoints(topology, networks=[network])
            if only:
                endpoints = only
            else:
                warn(
                    f"fleetlink base has no address on {network!r}; "
                    "using every fleetlink network"
                )
        if not endpoints:
            warn(
                f"fleetlink base {fh.fleetlink_base(topology)!r} has no address on "
                "zenoh.fleetlink_networks; using router mode"
            )
            return env
        return {
            **env,
            MODE_VAR: "fleetlink",
            "ADT4_FLEETLINK_CONNECT": ",".join(endpoints),
        }
    except Exception as e:  # noqa: BLE001 — never fail a launch over this
        warn(f"fleet link settings unavailable ({e}); using router mode")
        return router_env(topology if isinstance(topology, dict) else None)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("machine_name", nargs="?")
    parser.add_argument("--network", help="keep only this network's endpoint")
    parser.add_argument("--topology", help="topology.yaml (default: repo copy)")
    parser.add_argument(
        "--mode", choices=fh.FLEET_LINK_MODES, help="override fleet_defaults.fleet_link"
    )
    parser.add_argument(
        "--check", action="store_true", help="validate the topology and exit"
    )
    args = parser.parse_args(argv)
    if args.check:
        problems = fh.validate_topology(fh.load_topology(args.topology))
        for p in problems:
            print(f"topology: {p}")
        return 1 if problems else 0
    if not args.machine_name:
        parser.error("machine_name is required (except with --check)")

    def warn(msg):
        print(f"fleet_env: {msg}", file=sys.stderr)

    topology = None
    if args.topology:
        try:
            topology = fh.load_topology(args.topology)
        except Exception as e:  # noqa: BLE001
            warn(f"cannot read {args.topology} ({e}); using router mode")
            topology = {}
    env = fleet_env(args.machine_name, topology, args.network, warn, args.mode)
    for key, value in env.items():
        print(f"export {key}={shlex.quote(value)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
