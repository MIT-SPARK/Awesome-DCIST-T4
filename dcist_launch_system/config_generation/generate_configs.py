#!/usr/bin/env python3

import argparse
import itertools
import logging
import pathlib
import pprint
import re
import shlex
import shutil
import subprocess
import sys
import traceback

import yaml

logger = logging.getLogger(__name__)


def _dump_str(obj):
    return yaml.dump(obj, default_flow_style=False, width=800)


def _show_paths(paths, **kwargs):
    return pprint.pformat([str(x) for x in paths], **kwargs)


def str_to_bool(string):
    if string.lower() == "true":
        return True
    elif string.lower() == "false":
        return False
    return None


# --- pane shells -------------------------------------------------------------
# tmuxp types `shell_command_before` and each pane command with `tmux send-keys`
# right after creating the pane, before the pane's shell has finished loading
# ~/.zshrc. Whether those keystrokes survive depends on what the rc files do with
# pending terminal input; with heavy rc files (oh-my-zsh: update check, compinit,
# terminal queries) they were lost or typed into another pane: on willow the
# prior_dsg heracles publisher pane sat at an empty prompt, play_rosbag lost its
# command offline. Every generated pane therefore runs as its own process instead,
# through tmuxp's pane-level `shell` key, so there is nothing to type and nothing
# to lose:
#
#     zsh -ic 'trap : INT; <shell_command_before> && <cmd>; <exec an interactive zsh>'
#
# `trap : INT` keeps Ctrl-C on the command from also ending the pane: without it,
# an interactive `zsh -c` abandons the rest of the line when its job dies of
# SIGINT, and the pane closes. (Children still get the default SIGINT action.)
#
# The pane inherits the tmux session environment, but `zsh -ic` sources ~/.zshrc
# before the command, as the typed commands did. run-adt4 expands `$VAR`s from its
# own environment into the pane scripts before loading (expand_pane_shell), as
# tmuxp did for typed commands. When the command exits, the pane becomes an
# interactive zsh whose history holds the command (up-arrow restarts it). A command
# ending in a backslash is only prepared, not run: it is put on the prompt instead.
PANE_ZDOTDIR = (
    "${ADT4_WS}/src/awesome_dcist_t4/dcist_launch_system/config_generation/pane_shell"
)


# Ctrl-C on the pane's command must not end the pane (see above)
PANE_PREAMBLE = "trap : INT; "


def _cmd_strings(value):
    """tmuxp shell_command / shell_command_before (str, list, dicts) -> [str]."""
    if value is None:
        return []
    if isinstance(value, dict):
        value = value.get("shell_command", value.get("cmd", []))
    if isinstance(value, str):
        value = [value]
    cmds = []
    for item in value:
        if isinstance(item, dict):
            item = item.get("cmd", "")
        if item is None:
            continue
        item = str(item).strip()
        if item and item != "blank":
            cmds.append(item)
    return cmds


def interactive_shell_cmd(cmd=None, prefill=False, zdotdir=PANE_ZDOTDIR):
    """Shell code that replaces the pane process with an interactive zsh.

    ``cmd`` goes into that shell's history (or onto its prompt with ``prefill``).
    Falls back to a plain ``exec zsh`` if the pane_shell ZDOTDIR is missing.
    """
    env = []
    if cmd:
        env.append(f"ADT4_PANE_CMD={shlex.quote(cmd)}")
        if prefill:
            env.append("ADT4_PANE_PREFILL=1")
    return (
        f'_adt4_zd="{zdotdir}"; [[ -r $_adt4_zd/.zshrc ]] && '
        'export _ADT4_ZDOTDIR="${ZDOTDIR-}" ZDOTDIR=$_adt4_zd; '
        + " ".join(env + ["exec zsh"])
    )


def pane_script(before, cmds):
    """The shell script a pane runs: setup && command(s), then an interactive zsh."""
    before = _cmd_strings(before)
    cmds = _cmd_strings(cmds)
    prefill = None
    if cmds and cmds[-1].endswith("\\"):
        # a prepared command (typed but not run in the send-keys days)
        prefill = cmds.pop().rstrip("\\").rstrip()
    steps = before + cmds
    script = PANE_PREAMBLE + (" && ".join(steps) + "; " if steps else "")
    history = prefill if prefill is not None else " && ".join(cmds)
    return script + interactive_shell_cmd(history, prefill=prefill is not None)


def pane_shell(before, cmds):
    """tmuxp pane-level ``shell`` value for ``pane_script(before, cmds)``."""
    return f"zsh -ic {shlex.quote(pane_script(before, cmds))}"


def pane_command(pane):
    """The command a converted pane runs (its history entry); "" for none."""
    shell = pane.get("shell", "") if isinstance(pane, dict) else str(pane or "")
    argv = shlex.split(shell)
    if argv[:2] != ["zsh", "-ic"] or len(argv) != 3:
        return shell
    for token in shlex.split(argv[2]):
        if token.startswith("ADT4_PANE_CMD="):
            return token[len("ADT4_PANE_CMD=") :]
    return ""


_VAR_RE = re.compile(r"\$(\w+|\{[^}]*\})", re.ASCII)  # os.path.expandvars'
# variables of the pane script itself (interactive_shell_cmd), never expanded
_PANE_VARS = {"_adt4_zd"}


def expand_vars(text, env):
    """``os.path.expandvars`` against ``env``: set ``$VAR``/``${VAR}`` only."""

    def _sub(m):
        name = m.group(1)
        if name.startswith("{"):
            name = name[1:-1]
        if name in _PANE_VARS or name not in env:
            return m.group(0)
        return env[name]

    return _VAR_RE.sub(_sub, text)


def expand_pane_shell(shell, env):
    """Expand ``$VAR``s in a pane-shell script from ``env``, as tmuxp did when it typed
    the commands (``shell_command`` is expanded from the loading process's environment,
    a pane ``shell`` is not). Without it, a session started on a tmux server that
    another session started (e.g. start-adt4-console's adt4_console) would read the
    first session's values. The tail that starts the interactive shell is rebuilt
    from the expanded history entry, so the entry stays correctly quoted."""
    argv = shlex.split(shell)
    if argv[:2] != ["zsh", "-ic"] or len(argv) != 3:
        return shell
    script = argv[2]
    tail = script.rfind('_adt4_zd="')
    if tail < 0:
        return f"zsh -ic {shlex.quote(expand_vars(script, env))}"
    tokens = shlex.split(script[tail:])
    history = ""
    for token in tokens:
        if token.startswith("ADT4_PANE_CMD="):
            history = token[len("ADT4_PANE_CMD=") :]
    interactive = interactive_shell_cmd(
        expand_vars(history, env),
        prefill="ADT4_PANE_PREFILL=1" in tokens,
        zdotdir=expand_vars(PANE_ZDOTDIR, env),
    )
    return f"zsh -ic {shlex.quote(expand_vars(script[:tail], env) + interactive)}"


def unresolved_vars(text, env):
    """Names ``$VAR``/``${VAR}`` in ``text`` refers to that ``env`` does not set."""
    names = []
    for m in _VAR_RE.finditer(str(text)):
        name = m.group(1)
        if name.startswith("{"):
            name = name[1:-1]
        if re.fullmatch(r"\w+", name, re.ASCII) and name not in env:
            names.append(name)
    return names


def add_pane_exports(shell, exports):
    """Export ``exports`` ({name: value}) at the start of a pane script, after the
    pane's ~/.zshrc: a value's ``$VAR``s then expand in the pane, from what the rc
    files export, instead of reaching the pane as the literal text."""
    argv = shlex.split(shell)
    if not exports or argv[:2] != ["zsh", "-ic"] or len(argv) != 3:
        return shell
    script = argv[2]
    preamble = PANE_PREAMBLE if script.startswith(PANE_PREAMBLE) else ""
    lines = []
    for name, value in exports.items():
        quoted = re.sub(r'(["\\`])', r"\\\1", str(value))
        lines.append(f'export {name}="{quoted}"; ')
    script = preamble + "".join(lines) + script[len(preamble) :]
    return f"zsh -ic {shlex.quote(script)}"


def convert_to_pane_shells(contents):
    """Rewrite a tmuxp session so no pane relies on send-keys (see above)."""
    session_before = _cmd_strings(contents.pop("shell_command_before", None))
    for window in contents.get("windows") or []:
        before = session_before + _cmd_strings(window.pop("shell_command_before", None))
        panes = window.get("panes") or [None]
        new_panes = []
        for pane in panes:
            pane = dict(pane) if isinstance(pane, dict) else {"shell_command": pane}
            if "shell" in pane:  # already a process pane
                new_panes.append(pane)
                continue
            cmds = pane.pop("shell_command", None)
            pane_before = before + _cmd_strings(pane.pop("shell_command_before", None))
            pane["shell"] = pane_shell(pane_before, cmds)
            new_panes.append(pane)
        window["panes"] = new_panes
    return contents


def log_error(msg):
    logger.error(f"\033[1;33m{msg}\033[1;0m")


def log_warn(msg):
    logger.warning(f"\033[1;31m{msg}\033[1;0m")


def log_info(msg):
    logger.info(msg)


def log_debug(msg):
    logger.debug(msg)


def enforce_unique_filenames(paths):
    fns = [p.name for p in paths]
    counts = compute_list_counts(fns)

    for k, v in counts.items():
        if v > 1:
            raise Exception(f"Error, file {k} is specified {v} times!")


def list_files(directory):
    directory = pathlib.Path(directory)
    r = []
    for root, dirs, files in directory.walk():
        for name in files:
            r.append(root / name)

    return r


def chop(path, delim, n_start, n_end):
    if n_end > 0:
        toks = path.split(delim)[n_start:-n_end]
    else:
        toks = path.split(delim)[n_start:]

    return delim.join(toks)


def compute_list_counts(lst):
    counts = {}
    for e in lst:
        if e not in counts:
            counts[e] = 0
        counts[e] += 1

    return counts


def resolve_unique_dirs(manifest, keys, leaves=None):
    dirs = resolve_override_dirs(manifest, keys, leaves=leaves)

    # A given override directory might be (transitively) specified multiple
    # times. This isn't great (since it means that the included override groups
    # might not be orthoganol), so we'll warn about it (and also return only
    # the unique set of override directories)
    override_counts = compute_list_counts(dirs)

    for k, v in override_counts.items():
        if v > 1:
            log_warn(f"Warning, override directory {k} specified {v} times!")

    return list(override_counts.keys())


def resolve_override_dirs(manifest, keys, leaves=None):
    override_dirs = []
    for key in keys:
        children = manifest.get(key)
        if children is None and leaves:
            children = [] if key in leaves else None

        if children is None:
            valid_keys = list(manifest.keys()) + (leaves or [])
            log_error(
                f"Requested key {key} is not defined in the manifest! Valid keys: {valid_keys}"
            )
            sys.exit(f"invalid manifest key {key}")

        if len(children) == 0:
            override_dirs += [key]
            continue

        for c in children:
            transitive_children = resolve_override_dirs(manifest, [c], leaves=leaves)
            override_dirs += transitive_children

    return override_dirs


def get_experiment_manifest(root_path):
    root_path = pathlib.Path(root_path)
    manifest_path = root_path / "experiment_manifest.yaml"
    if not manifest_path.exists():
        log_error(f"Cannot find experiment manifest at {manifest_path}. Aborting!")
        exit(1)
        return

    with manifest_path.open("r") as fo:
        experiment_manifest = yaml.safe_load(fo)

    return experiment_manifest


def generate_config_name_yaml(root_path, experiment_manifest):
    root_path = pathlib.Path(root_path)
    for name, children in experiment_manifest["configs"].items():
        param_dir = root_path / "experiment_overrides" / name
        param_dir.mkdir(exist_ok=True, parents=True)


def find_concrete_configs(experiment_manifest):
    """Get all of the configs that appear in an experiment definition"""
    concrete_configs = []
    for _, v in experiment_manifest["experiments"].items():
        groupings = v["config"]
        for g in groupings:
            if g not in concrete_configs:
                concrete_configs.append(g)

    return concrete_configs


def render_config(root_path, manifest, base_yamls, concrete_configs, output_dir):
    root_path = pathlib.Path(root_path).expanduser().resolve()
    output_dir = pathlib.Path(output_dir).expanduser().resolve()
    for experiment_key in manifest["configs"].keys():
        # Configs only used as building blocks for other configs aren't rendered
        if experiment_key not in concrete_configs:
            continue

        log_debug(f"Generating config for {experiment_key}")
        base_param_dir = root_path / "base_params"
        rendered_config_dir = output_dir / experiment_key
        shutil.copytree(base_param_dir, rendered_config_dir, dirs_exist_ok=True)

        try:
            override_dirs = resolve_unique_dirs(manifest["configs"], [experiment_key])
            if experiment_key not in override_dirs:
                if (root_path / "experiment_overrides" / experiment_key).exists():
                    override_dirs.append(experiment_key)
        except KeyError:
            log_error(
                f"Error when processing {experiment_key}. Could not resolve config"
            )
            raise

        override_files = []
        for od in override_dirs:
            override_files += list_files(root_path / "experiment_overrides" / od)

        override_yamls = [f for f in override_files if f.match("*.yaml")]
        override_other = [f for f in override_files if not f.match("*.yaml")]

        enforce_unique_filenames(override_other)

        # Non-yaml files in the override directory are copied directly to the output config
        for path in override_other:
            copy_dst = rendered_config_dir / path.name
            log_debug(f"copy '{path}' to '{copy_dst}'")
            shutil.copy2(path, copy_dst)

        # There can be both an "args" and an "overlay" file, which we want to process together.
        # We build a map from "base" yaml files to override yaml files, and apply any applicable
        rendering_map = {}
        for fn in override_yamls:
            base_config_name = chop(fn.name, "_", 0, 1) + ".yaml"
            if base_config_name in base_yamls:
                if base_config_name not in rendering_map:
                    rendering_map[base_config_name] = []

                rendering_map[base_config_name].append(fn)

        log_debug(f"rendering_map: {rendering_map}")
        for by in base_yamls:
            full_base_name = base_param_dir / by
            cmd = ["composite-configs", "-d", "-f", str(full_base_name)]
            if by in rendering_map:
                for fn in rendering_map[by]:
                    log_debug(f"  Override {by} with {fn}")
                    cmd += ["-f", str(fn)]

            if experiment_key in concrete_configs:
                cmd += ["-v", f"config_type={experiment_key}"]
                dest_path = output_dir / experiment_key / by

                log_debug(f"  Calling: {' '.join(cmd)} and outputting to {dest_path}")
                with dest_path.open("w") as fout:
                    subprocess.run(cmd, stdout=fout)


def render_tmux(root_path, manifest, tmux_output_dir):
    root_path = pathlib.Path(root_path)
    tmux_output_dir = pathlib.Path(tmux_output_dir)
    tmux_output_dir.mkdir(exist_ok=True, parents=True)

    base_launch_file = root_path / "base_launch" / "base_launch.yaml"

    # Generate the tmux launch files
    all_configs = manifest["launch_configs"]
    components = [x.stem for x in (root_path / "launch_components").glob("*.yaml")]
    important_variants = set(manifest.get("important_variants", []))
    generated_variants = []
    for exp_key, exp in manifest["experiments"].items():
        log_info(f"Generating launch files for experiment {exp_key}")

        keys = exp["launch_config"]
        keys = keys if isinstance(keys, list) else [keys]
        launch_configs = resolve_unique_dirs(all_configs, keys, components)
        log_debug(f"Building launch config with windows: {launch_configs}")

        base_cmd = ["composite-configs", "-d", "-f", base_launch_file]
        for lc in launch_configs:
            base_cmd += ["-f", root_path / "launch_components" / f"{lc}.yaml"]

        special_keys = ["launch_config"]
        extra_keys = [k for k in exp.keys() if k not in special_keys]
        log_debug(f"{exp_key} extra keys: {extra_keys}")

        cartesian_kv = list(
            itertools.product(
                *[
                    [(k, vi) for vi in v]
                    for k, v in exp.items()
                    if k not in special_keys
                ]
            )
        )
        for tmux in cartesian_kv:
            logging_key = exp_key
            for k, v in tmux:
                bool_val = str_to_bool(v)
                if bool_val is None:
                    s = v
                elif bool_val:
                    s = k
                else:
                    s = "no_" + k

                logging_key += f"-{s}"

            extras = {
                "environment": {
                    "logging_key": logging_key,
                    "ADT4_ROBOT_NAME": "$ADT4_ROBOT_NAME",
                }
            }
            extras["environment"].update(tmux)

            cmd = [str(x) for x in base_cmd]
            base_launch_fn = tmux_output_dir / f"{logging_key}.yaml"
            log_debug(f"    Calling: {' '.join(cmd)} > {base_launch_fn}")

            ret = subprocess.run(cmd, stdout=subprocess.PIPE)
            if ret.returncode != 0:
                log_error(f"Failed to composite tmux for {logging_key}!")
                continue

            contents = yaml.safe_load(ret.stdout)
            generated_variants.append(logging_key)
            if "nodes_to_monitor" in contents:
                monitor_str = yaml.dump(
                    {"nodes_to_track": contents["nodes_to_monitor"]},
                    default_flow_style=True,
                    width=800,
                ).strip("\n")
                extras["environment"]["ADT4_MONITOR_CONFIG"] = monitor_str

            cmd += ["-c", _dump_str(extras)]
            ret = subprocess.run(cmd, stdout=subprocess.PIPE)
            if ret.returncode != 0:
                log_error(f"Failed to composite tmux for {logging_key}!")
                continue

            session = convert_to_pane_shells(yaml.safe_load(ret.stdout))
            with base_launch_fn.open("w") as fout:
                yaml.safe_dump(session, fout, sort_keys=False, width=100000)

    unmatched = important_variants - set(generated_variants)
    if unmatched:
        log_warn(f"Some important variants were not generated: {unmatched}")


def render_manifest(root_path, conf_output_dir, tmux_output_dir):
    root_path = pathlib.Path(root_path)
    conf_output_dir = pathlib.Path(conf_output_dir)
    tmux_output_dir = pathlib.Path(tmux_output_dir)
    if not conf_output_dir.exists():
        log_info(f"Creating {conf_output_dir}")
        conf_output_dir.mkdir(parents=True)

    if not tmux_output_dir.exists():
        log_info(f"Creating {tmux_output_dir}")
        tmux_output_dir.mkdir(parents=True)

    # Load the definition of what configs, node topologies, and launch files we need to generate
    experiment_manifest = get_experiment_manifest(root_path)

    base_param_path = root_path / "base_params"
    base_yamls = [x.name for x in base_param_path.glob("*.yaml")]
    log_info(f"base_yamls:\n{_show_paths(base_yamls, indent=2)}")

    # Identify the sets of parameters that need to be generated because they are loaded by a launch file
    concrete_configs = find_concrete_configs(experiment_manifest)

    log_info(f"Rendering concrete configs: {concrete_configs}")

    # Render all of the defined configs
    try:
        render_config(
            root_path,
            experiment_manifest,
            base_yamls,
            concrete_configs,
            conf_output_dir,
        )
    except KeyError:
        log_error("Could not render parameters!")
        raise

    # Render all of the tmux files
    render_tmux(root_path, experiment_manifest, tmux_output_dir)


def main():
    parser = argparse.ArgumentParser("Config Generation Tool")
    parser.add_argument("base_dir")
    parser.add_argument("conf_output_dir")
    parser.add_argument("tmux_output_dir")
    parser.add_argument("--verbose", "-v", action="store_true", default=False)
    parser.add_argument("--quiet", "-q", action="store_true", default=False)

    args = parser.parse_args()
    if args.verbose:
        logging.basicConfig(level=logging.DEBUG)
    elif args.quiet:
        logging.basicConfig(level=logging.WARNING)
    else:
        logging.basicConfig(level=logging.INFO)

    try:
        render_manifest(args.base_dir, args.conf_output_dir, args.tmux_output_dir)
    except Exception:
        tb = traceback.format_exc()
        log_info(tb)
        log_error("Rendering unsuccessful!")
        exit(1)


if __name__ == "__main__":
    main()
