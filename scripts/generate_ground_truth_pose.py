import argparse

import numpy as np
import robotdatapy as rdp

GPS_POSITIONS = {
    "lewis": np.array([0.175, 0.19, 0.65]),
    "gauss": np.array([0.175, 0.17, 0.55]),
    "pascal": np.array([-0.125, -0.17, 0.35]),
    "euclid": np.array([-0.22, 0.0, 0.54]),
    "hamilton": np.array([-0.22, 0.0, 0.54]),
}


def generate_ground_truth_pose(
    input_bag, output_csv, robot_name, max_gps_sigma=1.0, center=False
):
    gps_topic = f"/{robot_name}/fix"
    odom_parent_frame = f"{robot_name}/odom"
    if robot_name in ["gauss", "lewis", "pascal"]:
        odom_child_frame = f"{robot_name}/base_link"
    elif robot_name in ["hamilton", "euclid"]:
        odom_child_frame = f"{robot_name}/body"
    else:
        raise ValueError(f"Unknown robot name: {robot_name}")

    gps_data = rdp.data.GPSData.from_bag(input_bag, gps_topic, time_tol=10.0)
    gps_data.rm_nans()
    odometry = rdp.data.PoseData.from_bag_tf(
        input_bag, odom_parent_frame, odom_child_frame, time_tol=10.0
    )

    fused = rdp.data_fusion.fuse_gps_and_local_pose_estimates(
        gps_data,
        odometry,
        max_gps_sigma=max_gps_sigma,
        gps_position=GPS_POSITIONS[robot_name],
    )

    if center:
        mean = np.mean([fused.position(t) for t in fused.times], axis=0)
        T_premultiply = np.eye(4)
        T_premultiply[:3, 3] = -mean
        fused.T_premultiply = T_premultiply

    fused.to_csv(output_csv)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Generate ground truth pose from GPS and odometry data."
    )
    parser.add_argument(
        "-i",
        "--input_bag",
        type=str,
        required=True,
        help="Path to the input ROS bag file.",
    )
    parser.add_argument(
        "-o",
        "--output_csv",
        type=str,
        required=True,
        help="Path to the output CSV file for ground truth pose.",
    )
    parser.add_argument(
        "-r",
        "--robot_name",
        type=str,
        required=True,
        help="Name of the robot (e.g., gauss, lewis, pascal, hamilton, euclid).",
    )
    parser.add_argument(
        "-s",
        "--max-gps-sigma",
        type=float,
        default=1.0,
        help="Maximum GPS sigma to consider for fusion.",
    )
    parser.add_argument(
        "-c",
        "--center",
        action="store_true",
        help="Whether to center output fused trajectory",
    )

    args = parser.parse_args()
    generate_ground_truth_pose(
        args.input_bag,
        args.output_csv,
        args.robot_name,
        args.max_gps_sigma,
        center=args.center,
    )
