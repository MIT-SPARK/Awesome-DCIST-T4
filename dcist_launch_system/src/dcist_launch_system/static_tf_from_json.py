import argparse
import json
import logging
import shutil
import subprocess


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("filename")
    args, unknown_args = parser.parse_known_args()
    with open(args.filename, "r") as fin:
        contents = json.loads(fin.read())

    cmd = [shutil.which("ros2"), "run", "tf2_ros", "static_transform_publisher"]
    for key in ["x", "y", "z", "roll", "pitch", "yaw"]:
        cmd += [f"--{key}", str(contents.get(key, 0.0))]

    cmd += unknown_args
    logging.debug(f"Running {' '.join(cmd)}!")
    subprocess.run(cmd)


if __name__ == "__main__":
    main()
