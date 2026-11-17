#!/usr/bin/env python3
import os
import subprocess

import click


@click.command
@click.argument("output", type=click.Path())
@click.argument("topic_file", type=click.Path(exists=True))
def main(output, topic_file):
    """Wrap rosbag record with topics from file"""
    cmd = ["ros2", "bag", "record", "-o", output, "--topics"]
    # a ${ADT4_FLEET_ROBOT} line is one topic per robot of $ADT4_FLEET_ROBOTS
    # (comma separated); a file with such a line is refused when it is unset
    robots = [
        r.strip()
        for r in os.environ.get("ADT4_FLEET_ROBOTS", "").split(",")
        if r.strip()
    ]
    topics = []
    with open(topic_file, "r") as fin:
        for line in fin:
            line = line.strip()
            if "${ADT4_FLEET_ROBOT}" in line:
                if not robots:
                    raise click.ClickException(
                        f"{topic_file} has per-robot topics ({line}): set "
                        "ADT4_FLEET_ROBOTS to the fleet's robots, comma separated "
                        "(e.g. ADT4_FLEET_ROBOTS=euclid,hamilton,gauss)"
                    )
                topics += [
                    os.path.expandvars(line.replace("${ADT4_FLEET_ROBOT}", r))
                    for r in robots
                ]
            else:
                topics.append(os.path.expandvars(line))

    cmd += topics
    print(f"Running: '{' '.join(cmd)}'")
    subprocess.run(cmd)


if __name__ == "__main__":
    main()
