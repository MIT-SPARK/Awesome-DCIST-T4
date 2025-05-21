import pathlib

import click
import rclpy.logging
import rosbag2_py


def _get_first_odom_stamp(odom_file):
    with odom_file.open("r") as fin:
        have_first = False
        for line in fin:
            if not have_first:
                have_first = True
                continue

            return int(line.split(",")[0])


def _get_first_lidar_stamp(bag_file):
    reader = rosbag2_py.SequentialReader()
    reader.open_uri(str(bag_file))

    reader.set_filter(rosbag2_py.StorageFilter(regex=".*lidar_points"))
    assert reader.has_next()
    info = reader.read_next()
    return info[2]


def _get_bag_start(bag_file):
    reader = rosbag2_py.SequentialReader()
    reader.open_uri(str(bag_file))
    return reader.get_metadata().starting_time.nanoseconds


@click.command()
@click.argument("path", type=click.Path(exists=True))
def main(path):
    rclpy.logging.get_logger("rosbag2_storage").set_level(
        rclpy.logging.get_logging_severity_from_string("WARN")
    )
    path = pathlib.Path(path).expanduser().resolve()
    odom_files = sorted(list(path.rglob("*gt_odom.csv")))
    for odom_file in odom_files:
        bag_file = odom_file.parent / odom_file.stem[:-8]

        odom_ns = _get_first_odom_stamp(odom_file)
        bag_ns = _get_first_lidar_stamp(bag_file)
        # bag_ns = _get_bag_start(bag_file)
        diff_s = 1.0e-9 * (odom_ns - bag_ns)
        print(
            f"{bag_file.stem} -> odom: {odom_ns} [ns], bag: {bag_ns} [ns], diff: {diff_s} [s]"
        )


if __name__ == "__main__":
    main()
