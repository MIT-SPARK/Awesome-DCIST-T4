#!/usr/bin/env python3
"""Parse robot files and add to single json file."""

import json
import pathlib
import shutil
import subprocess
from typing import List

import click
from roman.align.params import SubmapAlignInputOutput, SubmapAlignParams
from roman.align.submap_align import submap_align


def _normalize_path(path):
    return pathlib.Path(path).expanduser().absolute()


def _convert_time(contents, idx):
    stamp_ns = int(contents["seconds"][idx] * 1.0e9)
    return stamp_ns + int(contents["nanoseconds"][idx])


def _convert_rot(quaternion_record, order):
    return {k: v for k, v in zip(order, quaternion_record)}


def _get_index(order, name):
    return order.index(name)


def _default_config():
    pkg_path = pathlib.Path(__file__).absolute().parent.parent
    config_path = pkg_path / "config" / "perception" / "hydra_multi" / "offline.yaml"
    return config_path


def _convert_file(contents, order, body_frame):
    loop_closures = []
    for x in contents:
        # convention is names[1]_T_names[0]
        to_R_from = _convert_rot(x["rotation"], x["rotation_convention"])
        to_p_from = x["translation"]
        robot_from = x["names"][1]
        robot_to = x["names"][0]
        loop_closures.append(
            {
                "robot_from": _get_index(order, robot_from),
                "time_from": _convert_time(x, 1),
                "robot_to": _get_index(order, robot_to),
                "time_to": _convert_time(x, 0),
                "to_p_from": to_p_from,
                "to_R_from": to_R_from,
                "in_body_frame": body_frame,
            }
        )

    return loop_closures


def lcd(
    robot_directories: List[pathlib.Path],
    root_output_dir: pathlib.Path,
    submap_max_size: int = 40,
    lc_association_thresh: int = 4,
):
    """
    Create loop closures files.

    Path directory will be:
    root_output_dir:
    - lcd/
      - robot0_robot0.json
      - robot0_robot1.json
      ...
      - robot0.sm.json
      - robot1.sm.json
      ...
    """
    lcd_output_dir = root_output_dir / "lcd"
    if lcd_output_dir.exists():
        click.secho("LCD output exists! skipping", fg="yellow")
        return

    lcd_output_dir.mkdir(exist_ok=True)

    # iterate through every pair of robots
    for i in range(len(robot_directories)):
        for j in range(i, len(robot_directories)):
            input_files = [
                str(robot_directories[i] / "roman_map.pkl"),
                str(robot_directories[j] / "roman_map.pkl"),
            ]
            sm_io = SubmapAlignInputOutput(
                inputs=input_files,
                output_dir=str(lcd_output_dir),
                run_name=f"{robot_directories[i].name}_{robot_directories[j].name}",
                robot_names=[robot_directories[i].name, robot_directories[j].name],
                # lc_association_thresh=4,
            )
            sm_params = SubmapAlignParams()
            sm_params.method = "pcavolgrav"
            sm_params.submap_max_size = submap_max_size
            sm_params.single_robot_lc = i == j
            sm_params.force_rm_upside_down = True
            sm_params.use_object_bottom_middle = True
            sm_io.lc_association_thresh = lc_association_thresh

            # run loop closure detection
            submap_align(sm_params=sm_params, sm_io=sm_io)


def collate_loopclosures(output, order, remaps, body_frame):
    """Load and convert robot loop closures."""
    loop_closures_path = output / "lcd"
    loop_closures = [x for x in loop_closures_path.glob("*.json")]
    loop_closures = [x for x in loop_closures if ".sm.json" not in x.name]

    roman_order = [remaps.get(x, x) for x in order]
    all_loopclosures = []
    for lcd_file in loop_closures:
        with lcd_file.open("r") as fin:
            contents = json.load(fin)

        click.secho(f"Loaded {len(contents)} loop closures from {lcd_file}", fg="green")
        all_loopclosures += _convert_file(contents, roman_order, body_frame)

    out_path = output / "loop_closures.json"
    click.secho(
        f"Writing {len(all_loopclosures)} loop closures to {out_path}!", fg="green"
    )
    with out_path.open("w") as fout:
        json.dump(all_loopclosures, fout, indent=2)

    return out_path


@click.command()
@click.argument("robot_directories", nargs=-1, type=click.Path(exists=True))
@click.option("-o", "--output", default=None, type=click.Path(), help="output file")
@click.option("-r", "--remap", multiple=True, type=str, help="robot name remap")
@click.option("-c", "--clean", is_flag=True, help="remove previous output if it exists")
@click.option("-f", "--force", is_flag=True, help="proceed without prompting")
@click.option("-v", "--verbosity", type=int, default=0, help="glog verbosity")
@click.option(
    "-m", "--submap-max-size", type=int, default=40, help="roman submap max size"
)
@click.option(
    "-t",
    "--lc-association-threshold",
    type=int,
    default=4,
    help="roman loop closure association threshold",
)
@click.option(
    "--config", default=None, type=click.Path(exists=True), help="config path"
)
@click.option(
    "--body-frame/--no-body-frame", default=True, help="loop closures in body frame"
)
@click.option("--gnc-alpha", default=None, type=float, help="gnc alpha to use")
def main(
    robot_directories,
    output,
    remap,
    clean,
    force,
    verbosity,
    submap_max_size,
    lc_association_threshold,
    config,
    body_frame,
    gnc_alpha,
):
    """Run intermission code."""
    # TODO(nathan) fix output path
    output = _normalize_path("fused") if output is None else _normalize_path(output)
    click.secho(f"Writing results to {output}!", fg="green")
    if output.exists():
        if clean:
            if not force:
                click.confirm(f"clean existing {output}?", abort=True, default=False)

            shutil.rmtree(output)
        else:
            click.secho(f"[WARNING] {output} exists already!", fg="yellow")
            if not force:
                click.confirm("continue?", abort=True, default=False)

    robot_directories = [_normalize_path(x) for x in robot_directories]
    output.mkdir(parents=True, exist_ok=True)
    lcd(
        robot_directories,
        output,
        submap_max_size=submap_max_size,
        lc_association_thresh=lc_association_threshold,
    )

    remaps = [x.split(":")[:2] for x in remap]
    remaps = {k: v for k, v in remaps}
    click.secho(f"Using remaps: {remaps}", fg="green")
    order = [_normalize_path(x).stem for x in robot_directories]
    click.secho(f"Robot order: {order}", fg="green")
    lcd_file = collate_loopclosures(output, order, remaps, body_frame)

    if config is None:
        config = _default_config()
        click.secho(f"[WARNING] Using default config {config}", fg="yellow")

    config = _normalize_path(config)
    command = [
        "offline_fusion",
        "--config",
        config,
        f"-v={verbosity}",
        "--output",
        str(output),
        "--loopclosures",
        str(lcd_file),
    ] + [str(_normalize_path(x)) for x in robot_directories]
    if gnc_alpha is not None:
        command += ["--gnc_alpha", str(gnc_alpha)]

    ret = subprocess.run(command)
    if ret.returncode != 0:
        click.secho("[ERROR] Fusion failed!", fg="red")


if __name__ == "__main__":
    main()
