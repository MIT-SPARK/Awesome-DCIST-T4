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
    with open(topic_file, "r") as fin:
        topics = [os.path.expandvars(x.strip()) for x in fin]

    cmd += topics
    print(f"Running: '{' '.join(cmd)}'")
    subprocess.run(cmd)


if __name__ == "__main__":
    main()
