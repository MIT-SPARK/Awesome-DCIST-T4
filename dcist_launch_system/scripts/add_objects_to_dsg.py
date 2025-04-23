#!/usr/bin/env python3
import argparse
import uuid
from dataclasses import dataclass
from typing import Optional

import numpy as np
import spark_dsg


@dataclass
class SpotObject:
    uuid: str
    position: np.ndarray
    bounding_box_dims: np.ndarray
    semantic_label: str
    semantic_color: np.ndarray
    b64image: Optional[str] = None


def load_spot_objects(fn):
    # TODO: load objects from file
    center = np.array([1, 2, 3.0])
    obj = SpotObject(
        str(uuid.uuid4()),
        center,
        np.array([10.0, 10, 10]),
        "wood",
        label_to_color["wood"],
        "b64ImageString",
    )
    return [obj]


def construct_object_attrs(o, label_to_id):
    bb = spark_dsg.BoundingBox(np.array([10.0, 10, 10]), o.position)

    attrs = spark_dsg.ObjectNodeAttributes()
    attrs.name = ""
    attrs.position = o.position
    attrs.semantic_label = label_to_id[o.semantic_label]
    attrs.color = o.semantic_color
    attrs.bounding_box = bb
    attrs.metadata.set({"image": o.b64image})
    return attrs


label_to_color = {"wood": np.array([0, 255, 0])}


def update_labelspace(G):
    id_to_label = G.get_labelspace(2, 0).labels_to_names
    label_to_id = G.get_labelspace(2, 0).names_to_labels
    max_semantic_id = max(id_to_label.keys())
    wood_id = max_semantic_id + 1
    id_to_label[wood_id] = "wood"
    label_to_id["wood"] = wood_id

    G.set_labelspace(
        spark_dsg.Labelspace(id_to_label),
        G.get_layer_id(spark_dsg.DsgLayers.OBJECTS).layer,
        0,
    )

    return id_to_label, label_to_id


def add_objects_to_dsg(G, objects):
    id_to_label, label_to_id = update_labelspace(G)

    max_gtsam_value = 0
    for n in G.get_layer(spark_dsg.DsgLayers.OBJECTS).nodes:
        max_gtsam_value = max(n.id.value, max_gtsam_value)
    print("Max existing gtsam value: ", max_gtsam_value)

    uuid_to_gtsam_value = {}
    last_gtsam_value = max_gtsam_value
    for o in objects:
        attrs = construct_object_attrs(o, label_to_id)
        gtsam_value = last_gtsam_value + 1
        last_gtsam_value += 1
        uuid_to_gtsam_value[o.uuid] = gtsam_value
        G.add_node(spark_dsg.DsgLayers.OBJECTS, gtsam_value, attrs)


def process_dsg_updates(dsg_fn, objects_fn):
    G = spark_dsg.DynamicSceneGraph.load(dsg_fn)
    objects = load_spot_objects(objects_fn)
    add_objects_to_dsg(G, objects)

    return G


def main():
    parser = argparse.ArgumentParser(description="Update DSG files.")
    parser.add_argument("dsg_path_in", type=str, help="Path to the input DSG JSON file")
    parser.add_argument(
        "objects_to_add_file", type=str, help="Path to a file with objects to add"
    )
    parser.add_argument(
        "--out",
        type=str,
        default="dsg_updated.json",
        help="Output path for the updated DSG file",
    )

    args = parser.parse_args()

    G = process_dsg_updates(args.dsg_path_in, args.objects_to_add_file)
    G.save(args.out, True)


if __name__ == "__main__":
    main()
