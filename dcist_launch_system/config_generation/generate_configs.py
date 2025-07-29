#!/usr/bin/env python3

import itertools
import logging
import os
import pathlib
import pprint
import shutil
import subprocess
import traceback
from dataclasses import dataclass, field
from typing import Annotated, TypeVar, Union

import click
import yaml
from pydantic import (
    AfterValidator,
    BaseModel,
    ValidationError,
    ValidationInfo,
    field_validator,
    model_validator,
)

T = TypeVar("T")
PathT = Union[str, bytes, os.PathLike]
TreeMap = dict[str, list[str]]

logger = logging.getLogger(__name__)
CMD = "composite-configs"


def _normalize_path(path: PathT) -> pathlib.Path:
    return pathlib.Path(path).expanduser().absolute()


def _show_conflicts(conflicts):
    def _render(x, p1, p2):
        return f"{x} included by {p1} and {p2}"

    return "[" + ", ".join([_render(*x) for x in conflicts]) + "]"


def _dump_str(obj, default_flow_style=True, width=800, **kwargs):
    return yaml.dump(obj, default_flow_style=default_flow_style, width=width, **kwargs)


def _show_paths(paths, **kwargs):
    return pprint.pformat([str(x) for x in paths], **kwargs)


def _get_tree_leaves(values: TreeMap, key: str, return_self: bool) -> list[str]:
    children = values[key]
    if not children:
        return [key] if return_self else []

    leaves = [_get_tree_leaves(values, c, True) for c in children]
    return [x for child_list in leaves for x in child_list]


def _get_nodes_to_monitor(cmd):
    ret = subprocess.run(cmd, stdout=subprocess.PIPE)
    if ret.returncode != 0:
        logger.error("'" + " ".join(cmd) + "' failed!")
        return None

    contents = yaml.safe_load(ret.stdout)
    if "nodes_to_monitor" not in contents:
        return _dump_str(contents), None

    track_str = _dump_str({"nodes_to_track": contents["nodes_to_monitor"]}).strip("\n")
    del contents["nodes_to_monitor"]
    return _dump_str(contents, sort_keys=False).strip("\n"), track_str


def nonempty(value: list[T]) -> list[T]:
    """Enforce that a list is non-empty."""
    if len(value) == 0:
        raise ValueError("value must be non-empty!")

    return value


def no_conflicts(value: TreeMap, key: str, parents: dict[str, str]):
    """Enforce that there are no keys with multiple parents."""
    children = value.get(key)
    if not children:
        return []

    invalid = []
    for child in children:
        if child in parents:
            invalid += [(child, key, parents[child])]
            continue

        parents[child] = key
        invalid += no_conflicts(value, child, parents)

    return invalid


def tree_valid(value: TreeMap, path: pathlib.Path, formatter=lambda x: x) -> TreeMap:
    missing = set([])
    for key, children in value.items():
        if not children:
            if not (path / formatter(key)).exists():
                missing.add(key)

            continue

        for child in children:
            if child not in value:
                missing.add(child)

    if missing:
        raise ValueError(f"Found missing names: '{missing}'")

    for key in value:
        conflicts = no_conflicts(value, key, {})
        if conflicts:
            conflict_str = _show_conflicts(conflicts)
            raise ValueError(f"Found conflicts for '{key}': {conflict_str}")

    return value


@dataclass
class ExperimentVariant:
    launch_mode: str
    config: str
    extras: dict[str, str] = field(default_factory=dict)

    @property
    def name(self):
        """Resolve unique name for experiment key and variant."""

        def _get_key_value_name(key, value):
            if value.lower() == "true":
                return key
            elif value.lower() == "false":
                return f"no_{key}"
            else:
                return value

        names = [self.config]
        names += [_get_key_value_name(k, v) for k, v in self.extras]
        return "-".join(names)


class Experiment(BaseModel):
    launch_mode: str
    robot_models: list[str]
    config: Annotated[list[str], AfterValidator(nonempty)]
    extras: dict[str, list[str]] = {}

    @property
    def variants(self):
        opts = [(k, v) for k, values in self.extras.items() for v in values]

        def _extras_product():
            if not opts:
                yield {}

            for combo in itertools.product(*opts):
                yield {k: v for k, v in combo}

        for config in self.config:
            for extras in _extras_product():
                yield ExperimentVariant(
                    launch_mode=self.launch_mode,
                    config=config,
                    extras=extras,
                )


class Manifest(BaseModel):
    """Experiment manifest model."""

    launch_modes: TreeMap = {}
    configs: TreeMap = {}
    experiments: dict[str, Experiment] = {}

    @field_validator("configs", mode="after")
    @classmethod
    def configs_present(cls, value: TreeMap, info: ValidationInfo) -> TreeMap:
        root_path = (info.context or {}).get("root_path")
        if not root_path:
            logger.warning(
                "Missing 'root_path' from context, unable to validate configs"
            )
            return value

        root_path = _normalize_path(root_path)
        return tree_valid(value, root_path / "experiment_overrides")

    @field_validator("launch_modes", mode="after")
    @classmethod
    def components_present(cls, value: TreeMap, info: ValidationInfo) -> TreeMap:
        root_path = (info.context or {}).get("root_path")
        if not root_path:
            logger.warning(
                "Missing 'root_path' from context, unable to validate launch modes"
            )
            return value

        root_path = _normalize_path(root_path)
        return tree_valid(value, root_path / "launch_components", lambda x: f"{x}.yaml")

    @model_validator(mode="after")
    def experiments_consistent(self):
        for key, info in self.experiments.items():
            if info.launch_mode not in self.launch_modes:
                available = f" (available: {[x for x in self.launch_modes]})"
                raise ValueError(
                    f"Missing launch mode '{info.launch_mode}' used by '{key}'{available}"
                )

        return self

    @classmethod
    def from_path(cls, root_path):
        root_path = _normalize_path(root_path)
        manifest_path = root_path / "experiment_manifest.yaml"
        if not manifest_path.exists():
            logger.error(f"Invalid manifest file: '{manifest_path}'!")
            return None

        with manifest_path.open("r") as fin:
            contents = yaml.safe_load(fin)

        try:
            return cls.model_validate(contents, context={"root_path": root_path})
        except ValidationError as e:
            logger.error(f"Could not parse manifest: {e}")
            return None

    @property
    def used_configs(self):
        """Return a list of all used configs by all experiments."""
        return set([c for _, exp in self.experiments.items() for c in exp.config])

    def get_config_leaves(self, key):
        """Get all base override keys for a specific config key."""
        if key not in self.configs:
            logger.warning(f"Missing '{key}' from configs!")
            return []

        return _get_tree_leaves(self.configs, key, False)

    def get_launch_leaves(self, key):
        """Get all base override keys for a specific launch mode key."""
        if key not in self.launch_modes:
            logger.warning(f"Missing '{key}' from launch modes!")
            return []

        return _get_tree_leaves(self.launch_modes, key, False)


def render_config(root_path: PathT, key: str, overrides: list[str], output: PathT):
    """
    Render a single config with overrides.

    Args:
        root_path: Path containing config information
        key: Name of rendered config
        overrides: List of overrides to apply (in order)
        output: Directory to output the rendered configs to
    """
    root_path = _normalize_path(root_path)
    output = _normalize_path(output)
    base_path = root_path / "base_params"
    overrides_path = root_path / "experiment_overrides"

    output.mkdir(parents=True, exist_ok=True)
    shutil.copytree(base_path, output, dirs_exist_ok=True)
    for override in overrides:
        curr_override_path = overrides_path / override
        for dirpath, _, files in curr_override_path.walk():
            for filename in files:
                filepath = dirpath / filename
                if filepath.match("*.yaml"):
                    continue

                destpath = output / (filepath.relative_to(curr_override_path))
                destpath.parent.mkdir(exist_ok=True, parents=True)
                shutil.copy2(filepath, destpath)

    base_yamls = [x for x in (root_path / "base_params").glob("*.yaml")]
    for base_file in base_yamls:
        cmd = [CMD, "-d", "-f", str(base_file), "-v", f"config_type={key}"]
        for override in overrides:
            override_file = overrides_path / override / base_file.name
            if override_file.exists():
                logger.debug(f"Override '{base_file}' with '{override_file}'")
                cmd += ["-f", str(override_file)]
            else:
                logger.debug(f"Override '{override_file}' for '{base_file}' not found")

        output_file = output / base_file.name
        logger.debug(f"Calling: {' '.join(cmd)} and outputting to {output_file}")
        with output_file.open("w") as fout:
            subprocess.run(cmd, stdout=fout)


def render_configs(manifest: Manifest, root_path, output):
    """
    Render all active configs to a output directory.

    Args:
        manifest: Manifest to render configs from
        root_path: Path containing all config information
        output: Directory to render all configs to
    """
    root_path = _normalize_path(root_path)
    output = _normalize_path(output)
    output.mkdir(exist_ok=True, parents=True)

    logging.debug("Rendering 'default' config!")
    render_config(root_path, "default", [], output / "default")

    valid_configs = [x for x in manifest.used_configs if x != "default"]
    logger.info(f"Rendering configs: {valid_configs}")
    for key in valid_configs:
        overrides = manifest.get_config_leaves(key)
        logging.debug(f"Rendering config '{key}' with overrides {overrides}")
        render_config(root_path, key, [key] + overrides, output / key)


def render_tmux(manifest, root_path, output):
    root_path = _normalize_path(root_path)
    output = _normalize_path(output)
    output.mkdir(exist_ok=True, parents=True)

    # Generate the tmux launch files
    base_launch_file = root_path / "base_launch" / "base_launch.yaml"
    for experiment, info in manifest.experiments.items():
        for robot in info.robot_models or [None]:
            key = f"{robot}-{experiment}" if robot else experiment
            launch_components = manifest.get_launch_leaves(experiment)

            logger.info(f"Generating launch files for experiment '{key}'")
            logger.debug(f"Building tmuxp file with windows: {launch_components}")

            cmd = [CMD, "-d", "--force-block-style", "-f", base_launch_file]
            for filepath in launch_components:
                cmd += ["-f", root_path / "launch_components" / f"{filepath}.yaml"]

            if robot:
                cmd += ["-f", root_path / "robots" / f"{robot}.yaml"]

            cmd = [str(x) for x in cmd]
            content_str, monitor_str = _get_nodes_to_monitor(cmd)
            cmd = [CMD, "-d", "--force-block-style", "-c", content_str]

            for variant in info.variants:
                name = f"{key}-{variant.name}"
                extras = {
                    "ADT4_LAUNCH_CONFIG": variant.config,
                    "ADT4_LAUNCH_ROBOT_MODEL": robot or "none",
                    "ADT4_LAUNCH_LOGGING_KEY": name,
                }

                if monitor_str:
                    extras["ADT4_MONITOR_CONFIG"] = monitor_str

                extras.update(variant.extras)

                output_file = output / f"{name}.yaml"
                cmd += ["-c", _dump_str({"environment": extras}, sort_keys=False)]
                logger.debug(f"Calling: {' '.join(cmd)} > {output_file}")
                with output_file.open("w") as fout:
                    subprocess.run(cmd, stdout=fout)


@click.command()
@click.argument("root_path", type=click.Path(exists=True))
@click.argument("conf_output_dir", type=click.Path())
@click.argument("tmux_output_dir", type=click.Path())
@click.option("--clear", "-c", is_flag=True)
@click.option("--verbose", "-v", is_flag=True)
@click.option("--quiet", "-q", is_flag=True)
def main(root_path, conf_output_dir, tmux_output_dir, clear, verbose, quiet):
    """Generate rendered configs and tmuxp files from a provided manifest."""
    if verbose:
        logging.basicConfig(level=logging.DEBUG)
    elif quiet:
        logging.basicConfig(level=logging.WARNING)
    else:
        logging.basicConfig(level=logging.INFO)

    root_path = _normalize_path(root_path)
    conf_output_dir = _normalize_path(conf_output_dir)
    if conf_output_dir.exists() and clear:
        logger.info(f"Clearing {conf_output_dir}")
        shutil.rmtree(conf_output_dir)

    tmux_output_dir = _normalize_path(tmux_output_dir)
    if tmux_output_dir.exists() and clear:
        logger.info(f"Clearing {tmux_output_dir}")
        shutil.rmtree(tmux_output_dir)

    # Load the definition of what configs, node topologies, and launch files we need to generate
    manifest = Manifest.from_path(root_path)
    if not manifest:
        exit(1)

    base_param_path = root_path / "base_params"
    base_yamls = [x.name for x in base_param_path.glob("*.yaml")]
    logger.info(f"base_yamls:\n{_show_paths(base_yamls, indent=2)}")

    try:
        render_configs(manifest, root_path, conf_output_dir)
        render_tmux(manifest, root_path, tmux_output_dir)
    except Exception:
        tb = traceback.format_exc()
        logger.info(tb)
        logger.error("Rendering unsuccessful!")
        exit(1)


if __name__ == "__main__":
    main()
