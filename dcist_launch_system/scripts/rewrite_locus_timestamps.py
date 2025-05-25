import pathlib

import click
import rclpy.time
import rclpy.logging
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import shutil

import rclpy.time
from rclpy.serialization import serialize_message


def _load_odom(bag_file, topic_regex=".*odometry"):
    reader = rosbag2_py.SequentialReader()
    reader.open_uri(str(bag_file))

    topic_types = reader.get_all_topics_and_types()

    def _deserialize(topic_name, data):
        for topic_type in topic_types:
            if topic_type.name == topic_name:
                return deserialize_message(data, get_message(topic_type.type))

        return None

    msgs = []
    reader.set_filter(rosbag2_py.StorageFilter(regex=topic_regex))
    while reader.has_next():
        topic, data, t = reader.read_next()
        msg = _deserialize(topic, data)
        msgs.append(msg)

    return msgs


@click.command()
@click.argument("path", type=click.Path(exists=True))
@click.option("--bags-path", type=click.Path(exists=True))
@click.option("--name", "-n", type=str, default="_locus.db3")
@click.option("--force", "-f", is_flag=True)
@click.option("--threshold", "-t", type=int, default=0)
@click.option("--topic-out", default="locus_odom")
def main(path, bags_path, name, force, threshold, topic_out):
    rclpy.logging.get_logger("rosbag2_storage").set_level(
        rclpy.logging.get_logging_severity_from_string("WARN")
    )
    path = pathlib.Path(path).expanduser().resolve()
    odom_bags = sorted(list(path.rglob(f"*{name}")))
    odom_bags = [x.parent for x in odom_bags]
    for odom_bag in odom_bags:
        if bags_path:
            bags_path = pathlib.Path(bags_path).expanduser().resolve()
            bags_parent = bags_path / (odom_bag.parent.relative_to(path))
        else:
            bags_parent = odom_bag.parent

        suffix = name.split(".")[0]
        bag_file = bags_parent / odom_bag.stem[: -len(suffix)]
        if not bag_file.exists():
            click.secho(f"Could not find original bag file: {bag_file}", fg="red")
            continue

        output_bag = bag_file.parent / f"{bag_file.stem}_odom"
        if output_bag.exists():
            if not force:
                click.confirm(f"Delete existing '{output_bag}'?", abort=True)

            shutil.rmtree(output_bag)

        click.secho(f"Reading odom from {odom_bag}", fg="green")
        click.secho(f"Reading time from {bag_file}", fg="green")
        click.secho(f"Writing to {output_bag}", fg="green")
        odom = _load_odom(odom_bag)
        reader = rosbag2_py.SequentialReader()
        reader.open_uri(str(bag_file))

        writer = rosbag2_py.SequentialWriter()
        writer.open(
            rosbag2_py.StorageOptions(uri=str(output_bag)),
            rosbag2_py.ConverterOptions(),
        )

        topic_types = reader.get_all_topics_and_types()

        def _deserialize(topic_name, data):
            for topic_type in topic_types:
                if topic_type.name == topic_name:
                    return deserialize_message(data, get_message(topic_type.type))

            return None

        odom_idx = 0
        created_topic = False
        reader.set_filter(rosbag2_py.StorageFilter(regex=".*lidar_points"))
        while reader.has_next():
            if odom_idx >= len(odom):
                click.secho("More pointclouds than odom!", fg="red")
                break

            topic, data, t = reader.read_next()
            robot_name = topic.split("/")[1]
            odom_topic = (
                "/"
                + robot_name
                + "/"
                + (topic_out if topic_out[0] != "/" else topic_out[1:])
            )

            if not created_topic:
                writer.create_topic(
                    rosbag2_py.TopicMetadata(
                        id=0,
                        name=odom_topic,
                        type="nav_msgs/msg/Odometry",
                        serialization_format="cdr",
                    )
                )

            msg = _deserialize(topic, data)
            msg_ns = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds

            while odom_idx < len(odom):
                odom_msg = odom[odom_idx]
                odom_ns = rclpy.time.Time.from_msg(odom_msg.header.stamp).nanoseconds
                diff_ns = odom_ns - msg_ns

                if diff_ns > threshold:
                    if odom_ns > msg_ns:
                        break

                    odom_idx += 1
                    continue

                writer.write(odom_topic, serialize_message(odom_msg), t)
                odom_idx += 1
                break

        click.secho(f"Wrote {odom_idx} of {len(odom)} messages!", fg="green")


if __name__ == "__main__":
    main()
