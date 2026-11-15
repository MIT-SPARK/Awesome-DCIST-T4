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
    # (comma separated; the three field robots when unset)
    robots = os.environ.get("ADT4_FLEET_ROBOTS", "euclid,hamilton,gauss").split(",")
    topics = []
    with open(topic_file, "r") as fin:
        for line in fin:
            line = line.strip()
            if "${ADT4_FLEET_ROBOT}" in line:
                topics += [
                    os.path.expandvars(line.replace("${ADT4_FLEET_ROBOT}", r.strip()))
                    for r in robots
                    if r.strip()
                ]
            else:
                topics.append(os.path.expandvars(line))

    cmd += topics
    print(f"Running: '{' '.join(cmd)}'")
    subprocess.run(cmd)


if __name__ == "__main__":
    main()
