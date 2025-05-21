import pathlib
import shutil

import click
import nav_msgs.msg
import rclpy.logging
import rclpy.time
import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message


def _load_odom(odom_file):
    odom = []
    with odom_file.open("r") as fin:
        have_first = False
        for line in fin:
            if not have_first:
                have_first = True
                continue

            values = line.split(",")
            msg = nav_msgs.msg.Odometry()
            msg.header.stamp = rclpy.time.Time(nanoseconds=int(values[0])).to_msg()
            msg.pose.pose.position.x = float(values[1])
            msg.pose.pose.position.y = float(values[2])
            msg.pose.pose.position.z = float(values[3])
            msg.pose.pose.orientation.w = float(values[4])
            msg.pose.pose.orientation.x = float(values[5])
            msg.pose.pose.orientation.y = float(values[6])
            msg.pose.pose.orientation.z = float(values[7])
            odom.append(msg)

    return odom


@click.command()
@click.argument("path", type=click.Path(exists=True))
@click.option("--name", "-n", type=str, default="/gt_odom")
@click.option("--parent-frame", type=str, default="odom")
@click.option("--child-frame", type=str, default="base_link")
@click.option("--force", "-f", is_flag=True)
@click.option("--threshold", "-t", type=int, default=0)
@click.option("--verbose", "-v", is_flag=True)
def main(path, name, parent_frame, child_frame, force, threshold, verbose):
    rclpy.logging.get_logger("rosbag2_storage").set_level(
        rclpy.logging.get_logging_severity_from_string("WARN")
    )
    path = pathlib.Path(path).expanduser().resolve()
    odom_files = sorted(list(path.rglob("*gt_odom.csv")))
    for odom_file in odom_files:
        odom = _load_odom(odom_file)

        bag_file = odom_file.parent / odom_file.stem[:-8]
        reader = rosbag2_py.SequentialReader()
        reader.open_uri(str(bag_file))

        output_bag = bag_file.parent / f"{bag_file.stem}_odom"
        if output_bag.exists():
            if not force:
                click.confirm(f"Delete existing '{output_bag}'?", abort=True)

            shutil.rmtree(output_bag)

        click.secho(f"Writing {output_bag}", fg="green")
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
            odom_topic = "/" + robot_name + "/" + (name if name[0] != "/" else name[1:])

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

                if verbose:
                    click.secho(
                        f"Writing {msg_ns} [ns] (orig: {odom_ns} [ns]) @ {t} [ns]...",
                        fg="green",
                    )
                    click.secho(
                        f"Message to odom: {(msg_ns - odom_ns) * 1.0e-9} [ns]",
                        fg="green",
                    )
                    click.secho(
                        f"Message to bag:  {(msg_ns - t) * 1.0e-9} [ns]", fg="green"
                    )
                    click.secho(
                        f"Odom to bag:     {(odom_ns - t) * 1.0e-9} [ns]", fg="green"
                    )

                odom_msg.header.frame_id = parent_frame
                odom_msg.child_frame_id = child_frame
                odom_msg.header.stamp = msg.header.stamp
                writer.write(odom_topic, serialize_message(odom_msg), t)
                odom_idx += 1
                break

        click.secho(f"Wrote {odom_idx} of {len(odom)} messages!", fg="green")


if __name__ == "__main__":
    main()
