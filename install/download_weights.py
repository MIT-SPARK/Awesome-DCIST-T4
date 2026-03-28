#!/usr/bin/env python3
import argparse
import pathlib

import ultralytics


def main():
    parser = argparse.ArgumentParser(
        description="Utility for downloading ultralytics weights"
    )
    parser.add_argument("models", nargs="*")
    parser.add_argument("-d", "--directory", default=None)
    args = parser.parse_args()

    target_path = pathlib.Path(args.directory or ".").expanduser().absolute()
    target_path.mkdir(exist_ok=True, parents=True)
    for model in args.models:
        output_path = target_path / f"{model}"
        if output_path.exists():
            continue

        print(f"Downloading {model} to {output_path}")
        ultralytics.utils.downloads.attempt_download_asset(str(output_path))


if __name__ == "__main__":
    main()
