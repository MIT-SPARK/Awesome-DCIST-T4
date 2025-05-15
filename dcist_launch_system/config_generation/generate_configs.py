#!/usr/bin/env python3

import argparse
import io
import itertools
import logging
import pathlib
import pprint
import shutil
import subprocess
import traceback

import ruamel.yaml

yaml = ruamel.yaml.YAML()

logger = logging.getLogger(__name__)


def _show_paths(paths, **kwargs):
    return pprint.pformat([str(x) for x in paths], **kwargs)

def str_to_bool(string):
    if string.lower() == "true":
        return True
    elif string.lower() == "false":
        return False
    return None


def log_error(msg):
    logger.error(f"\033[1;33m{msg}\033[1;0m")


def log_warn(msg):
    logger.warning(f"\033[1;31m{msg}\033[1;0m")


def log_info(msg):
    logger.info(msg)


def log_debug(msg):
    logger.debug(msg)


def enforce_unique_filenames(paths):
    fns = [p.split("/")[-1] for p in paths]
    counts = compute_list_counts(fns)

    for k, v in counts.items():
        if v > 1:
            raise Exception(f"Error, file {k} is specified {v} times!")


def list_files(directory):
    directory = pathlib.Path(directory)
    r = []
    for root, dirs, files in directory.walk():
        for name in files:
            r.append(root /  name)

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


def resolve_unique_dirs(manifest, key):
    dirs = resolve_override_dirs(manifest, key)

    # A given override directory might be (transitively) specified multiple
    # times. This isn't great (since it means that the included override groups
    # might not be orthoganol), so we'll warn about it (and also return only
    # the unique set of override directories)
    override_counts = compute_list_counts(dirs)

    for k, v in override_counts.items():
        if v > 1:
            log_warn(f"Warning, override directory {k} specified {v} times!")

    return list(override_counts.keys())


def resolve_override_dirs(manifest, key):
    override_dirs = []
    try:
        children = manifest[key]
    except KeyError:
        log_error(
            f"Requested key {key} is not a defined config in your manifest! Valid keys are {list(manifest.keys())}"
        )
        raise

    if len(children) == 0:
        override_dirs = [key]
        return override_dirs
    for c in children:
        transitive_children = resolve_override_dirs(manifest, c)
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
        experiment_manifest = yaml.load(fo)

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


def render_config(
    root_path, experiment_manifest, base_yamls, concrete_configs, output_dir
):
    root_path = pathlib.Path(root_path).expanduser().resolve()
    output_dir = pathlib.Path(output_dir).expanduser().resolve()
    for experiment_key in experiment_manifest["configs"].keys():
        log_debug(f"Generating config for {experiment_key}")
        base_param_dir = root_path / "base_params"
        rendered_config_dir = output_dir / experiment_key
        shutil.copytree(base_param_dir, rendered_config_dir, dirs_exist_ok=True)

        try:
            override_dirs = resolve_unique_dirs(
                experiment_manifest["configs"], experiment_key
            )
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
            cmd = ["composite-configs", "-f", str(full_base_name)]
            if by in rendering_map:
                for fn in rendering_map[by]:
                    log_debug(f"  Override {by} with {fn}")
                    cmd += ["-f", str(fn)]

            if experiment_key in concrete_configs:
                cmd += ["-v", f"config_name={experiment_key}"]
                dest_path = output_dir / experiment_key / by

                log_debug(f"  Calling: {' '.join(cmd)} and outputting to {dest_path}")
                with dest_path.open("w") as fout:
                    subprocess.run(cmd, stdout=fout)


def render_tmux(root_path, experiment_manifest, tmux_output_dir):
    root_path = pathlib.Path(root_path)
    tmux_output_dir = pathlib.Path(tmux_output_dir)
    tmux_output_dir.mkdir(exist_ok=True, parents=True)

    base_launch_file = root_path / "base_launch" / "base_launch.yaml"

    # Generate the tmux launch files
    for exp_key, exp in experiment_manifest["experiments"].items():
        log_info(f"Generating launch files for experiment {exp_key}")

        launch_configs = resolve_unique_dirs(
            experiment_manifest["launch_configs"], exp["launch_config"]
        )
        log_debug(f"Building launch config with windows: {launch_configs}")

        cmd = ["composite-configs", "-f", base_launch_file]
        for lc in launch_configs:
            cmd += ["-f", root_path / "launch_components" / f"{lc}.yaml"]

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

            cmd = [str(x) for x in cmd]
            extras = {"environment": {"logging_key": logging_key}}
            extras["environment"].update(tmux)
            extras_str = io.StringIO()
            yaml.dump(extras, extras_str)
            cmd += ["-c", extras_str.getvalue()]
            extras_str.close()

            base_launch_fn = tmux_output_dir / f"{logging_key}.yaml"
            log_debug(f"    Calling: {" ".join(cmd)} > {base_launch_fn}")
            with base_launch_fn.open("w") as fout:
                subprocess.run(cmd, stdout=fout)


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
