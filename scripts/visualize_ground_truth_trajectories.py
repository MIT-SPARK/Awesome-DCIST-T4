import argparse
from typing import Dict

import matplotlib.pyplot as plt
import robotdatapy as rdp
from robotdatapy.data import PoseData


def visualize_ground_truth_trajectories(
    pose_data_dict: Dict[str, PoseData], output_file: str = None, plot_poses=False
):
    for name, pose_data in pose_data_dict.items():
        pose_data.plot2d(label=name)
        if plot_poses:
            pose_data.plot2d(trajectory=False, pose=True, dt=10.0, axis_len=3.0)
    plt.legend()
    if output_file:
        plt.savefig(output_file, dpi=300)
    else:
        plt.show()

    ax = None
    for name, pose_data in pose_data_dict.items():
        ax = pose_data.plot3d(ax=ax)
    plt.legend()
    if output_file:
        plt.savefig(output_file.replace(".", "_3d."), dpi=300)
    else:
        plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Visualize ground truth trajectories from PoseData files."
    )
    parser.add_argument(
        "-i",
        "--input",
        type=str,
        nargs="+",
        action="append",
        default=[],
        help="Input files in the form of <name> <path_to_pose_data_file>",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=str,
        default=None,
        help="Output file (w/ image extension) to save visualizations (optional).",
    )
    parser.add_argument(
        "-p",
        "--plot_poses",
        action="store_true",
        help="Whether to plot individual poses along the trajectory.",
    )
    args = parser.parse_args()

    pose_data_dict = {}
    for entry in args.input:
        assert len(entry) == 2, "Each input must contain a name and a file path."
        name, file_path = entry
        pose_data = PoseData.from_csv(
            file_path,
            csv_options=rdp.data.pose_data.KIMERA_MULTI_GT_CSV_OPTIONS,
            time_tol=10.0,
        )
        pose_data_dict[name] = pose_data

    visualize_ground_truth_trajectories(pose_data_dict, args.output, args.plot_poses)
