import pathlib
import json
import yaml

import tf_transformations

this_path = pathlib.Path(__file__).absolute().parent
calibrations = list(this_path.rglob("**/zed_calibration.json"))

for x in calibrations:
    with x.open("r") as fin:
        contents = json.load(fin)

    roll = contents["roll"]
    pitch = contents["pitch"]
    yaw = contents["yaw"]
    q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)

    new_contents = {}
    new_contents["frame_id"] = "$(var robot_name)/frontleft"
    new_contents["child_frame_id"] = "$(var robot_name)_zed_camera_link"
    new_contents["x"] = contents["x"]
    new_contents["y"] = contents["y"]
    new_contents["z"] = contents["z"]
    new_contents["qw"] = float(q[3])
    new_contents["qx"] = float(q[0])
    new_contents["qy"] = float(q[1])
    new_contents["qz"] = float(q[2])

    new_path = x.parent / "calibration.yaml"
    with new_path.open("w") as fout:
        fout.write(yaml.dump({"frames": [new_contents]}))
