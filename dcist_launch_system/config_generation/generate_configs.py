#!/usr/bin/env python3

import argparse
import logging
import os
import shutil
import traceback

import yaml

logger = logging.getLogger(__name__)


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
    r = []
    for root, dirs, files in os.walk(directory):
        for name in files:
            r.append(os.path.join(root, name))
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
            f"Requested key {key} is not a defined parameter grouping in your manifest! Valid keys are {list(manifest.keys())}"
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
    manifest_path = os.path.join(root_path, "experiment_manifest.yaml")
    if not os.path.exists(manifest_path):
        log_error(f"Cannot find experiment manifest at {manifest_path}. Aborting!")
        exit(1)
        return
    with open(manifest_path, "r") as fo:
        experiment_manifest = yaml.safe_load(fo)
    return experiment_manifest


def generate_config_name_yaml(root_path, experiment_manifest):
    for name, children in experiment_manifest["parameter_groupings"].items():
        param_dir = os.path.join(root_path, "experiment_overrides", name)
        if not os.path.exists(param_dir):
            os.mkdir(param_dir)
        config_name_path = os.path.join(param_dir, "config_name.yaml")
        with open(config_name_path, "w") as fo:
            fo.write(
                f"#@data/values-schema\n---\n#@overlay/match missing_ok=True\nconfig_type: {name}\n"
            )


def find_concrete_param_groupings(experiment_manifest):
    """Get all of the parameter groupings that appear in an experiment definition"""
    concrete_param_groupings = []
    for _, v in experiment_manifest["experiments"].items():
        groupings = v["parameter_groupings"]
        for g in groupings:
            if g not in concrete_param_groupings:
                concrete_param_groupings.append(g)
    return concrete_param_groupings


def render_param_grouping(
    root_path, experiment_manifest, base_yamls, concrete_param_groupings, output_dir
):
    for experiment_key in experiment_manifest["parameter_groupings"].keys():
        log_debug(f"Generating config for {experiment_key}")
        base_param_dir = os.path.join(root_path, "base_params")
        rendered_config_dir = os.path.join(output_dir, experiment_key)
        shutil.copytree(base_param_dir, rendered_config_dir, dirs_exist_ok=True)

        try:
            override_dirs = resolve_unique_dirs(
                experiment_manifest["parameter_groupings"], experiment_key
            )
        except KeyError:
            log_error(
                f"Error when processing {experiment_key}. Could not resolve parameter grouping"
            )
            raise

        override_files = []
        for od in override_dirs:
            override_files += list_files(
                os.path.join(root_path, "experiment_overrides", od)
            )

        override_yamls = [f for f in override_files if f.endswith("yaml")]
        override_other = [f for f in override_files if not f.endswith("yaml")]

        enforce_unique_filenames(override_other)

        # Non-yaml files in the override directory are copied directly to the output config
        for path in override_other:
            fn = path.split("/")[-1]
            copy_dst = os.path.join(rendered_config_dir, fn)
            log_debug(f"copy {fn} to {copy_dst}")
            shutil.copy(path, copy_dst)

        # There can be both an "args" and an "overlay" file, which we want to process together.
        # We build a map from "base" yaml files to override yaml files, and apply any applicable
        rendering_map = {}
        for fn in override_yamls:
            base_config_name = chop(fn.split("/")[-1], "_", 0, 1) + ".yaml"

            if base_config_name in base_yamls:
                if base_config_name not in rendering_map:
                    rendering_map[base_config_name] = []
                rendering_map[base_config_name].append(fn)

        log_debug(f"rendering_map: {rendering_map}")
        for by in base_yamls:
            full_base_name = os.path.join(base_param_dir, by)
            cmd = f"ytt -f {full_base_name}"
            if by in rendering_map:
                for fn in rendering_map[by]:
                    log_debug(f"  Override {by} with {fn}")
                    cmd += f" -f {fn}"

            if experiment_key in concrete_param_groupings:
                cmd += f" -f {root_path}/experiment_overrides/{experiment_key}/config_name.yaml"
                dest_path = os.path.join(output_dir, experiment_key, by)

                cmd += f" > {dest_path}"

                log_debug(f"  Calling: {cmd}")
                os.system(cmd)


def render_tmux(root_path, experiment_manifest, tmux_output_dir):
    base_launch_file = os.path.join(root_path, "base_launch", "base_launch.yaml")
    if not os.path.exists(tmux_output_dir):
        os.mkdir(tmux_output_dir)

    # Generate the tmux launch files
    for exp_key, exp in experiment_manifest["experiments"].items():
        log_info(f"Generating launch files for experiment {exp_key}")

        launch_configs = resolve_unique_dirs(
            experiment_manifest["launch_configs"], exp["launch_config"]
        )
        log_debug(f"Building launch config with windows: {launch_configs}")

        cmd = f"ytt -f {base_launch_file}"
        for lc in launch_configs:
            component_yaml = os.path.join(root_path, "launch_components", lc) + ".yaml"
            cmd += f" -f {component_yaml}"

        # TODO: generalize this to automatically add any keys as environment variables to the tmux
        for odom in exp["odometry_types"]:
            log_debug(f"  Generating launch file for odom type {odom}")
            if "real_robot" in exp.keys():
                real_robot_vals = exp["real_robot"]
            else:
                real_robot_vals = ["false"]
            for real_robot in real_robot_vals:
                cmd_w_odom = cmd + f" -v odometry_type={odom}"
                for pg in exp["parameter_groupings"]:
                    log_debug(f"    Generating launch file for parameter grouping {pg}")
                    logging_key = f"{exp_key}-{odom}-{pg}"
                    cmd_w_odom_w_params = (
                        cmd_w_odom + f" -v config={pg} -v logging_key={logging_key}"
                    )
                    cmd_w_odom_w_params += f" -v real_robot={real_robot}"
                    launch_fn = f"{tmux_output_dir}/{exp_key}-{odom}-{pg}.yaml"
                    cmd_final = cmd_w_odom_w_params + " > " + launch_fn
                    log_debug(f"    Calling: {cmd_final}")
                    os.system(cmd_final)


def render_manifest(root_path, conf_output_dir, tmux_output_dir):
    if not os.path.exists(conf_output_dir):
        log_info(f"Creating {conf_output_dir}")
        os.makedirs(conf_output_dir)

    if not os.path.exists(tmux_output_dir):
        log_info(f"Creating {tmux_output_dir}")
        os.makedirs(tmux_output_dir)

    # Load the definition of what configs, node topologies, and launch files we need to generate
    experiment_manifest = get_experiment_manifest(root_path)

    # Generate an extra config file that's necessary to optionally fill in the
    # config name as a parameter in the base templates
    generate_config_name_yaml(root_path, experiment_manifest)

    base_param_path = os.path.join(root_path, "base_params")
    base_yamls = [f for f in os.listdir(base_param_path) if f.endswith("yaml")]
    log_info(f"base_yamls: {base_yamls}")

    # Identify the sets of parameters that need to be generated because they are loaded by a launch file
    concrete_param_groupings = find_concrete_param_groupings(experiment_manifest)

    log_info(f"Rendering concrete configs: {concrete_param_groupings}")

    # Render all of the defined parameter groupings.
    try:
        render_param_grouping(
            root_path,
            experiment_manifest,
            base_yamls,
            concrete_param_groupings,
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
