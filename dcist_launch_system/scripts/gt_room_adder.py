import os
from typing import Optional

import numpy as np
import spark_dsg
import yaml
from scipy.spatial.transform import Rotation as Rot


class RoomExtents:
    def __init__(self, path_to_yaml: Optional[str] = None):
        self.boxes: list[list[spark_dsg.BoundingBox]] = []

        if path_to_yaml is None or not os.path.exists(path_to_yaml):
            raise Exception(
                "Currently RoomExtents must be initialized from a valid yaml path"
            )

        with open(path_to_yaml, "r") as f:
            root = yaml.safe_load(f)

        for key in sorted(root.keys(), key=lambda k: int(k)):
            group = []
            for box_node in root[key]:
                center = np.array(box_node["center"], dtype=float)
                dimensions = np.array(box_node["extents"], dtype=float)
                rot_dict = box_node["rotation"]
                rotation = np.array(
                    [rot_dict["x"], rot_dict["y"], rot_dict["z"], rot_dict["w"]],
                    dtype=float,
                )

                rotmat = Rot.from_quat(rotation).as_matrix()
                group.append(
                    spark_dsg.BoundingBox(
                        spark_dsg.BoundingBoxType.AABB, dimensions, center, rotmat
                    )
                )

            self.boxes.append(group)


def add_gt_rooms(
    dsg: spark_dsg.DynamicSceneGraph,
    room_boxes: list[list[spark_dsg.BoundingBox]],
    semantic_labels: list[int],
):
    for idx, boxes in enumerate(room_boxes):
        box_centers = np.mean([b.world_P_center for b in boxes], axis=0)
        room_position = box_centers
        room_attrs = spark_dsg.RoomNodeAttributes()
        room_attrs.semantic_label = semantic_labels[idx]
        room_attrs.position = room_position
        dsg.add_node(
            spark_dsg.DsgLayers.ROOMS, spark_dsg.NodeSymbol("R", idx), room_attrs
        )

    for idx, boxes in enumerate(room_boxes):
        print(f"{idx}, {boxes}")
        for box in boxes:
            for node in dsg.get_layer(spark_dsg.DsgLayers.MESH_PLACES).nodes:
                pos = node.attributes.position
                if box.contains(pos):
                    print(f"R{idx} -> {node.id}")
                    dsg.insert_edge(spark_dsg.NodeSymbol("R", idx), node.id)


def add_manual_connections(dsg, connections):
    for edge in connections:
        s = edge[0]
        t = edge[1]
        dsg.insert_edge(spark_dsg.NodeSymbol("R", s), spark_dsg.NodeSymbol("R", t))


if __name__ == "__main__":
    dsg_path = "/home/aaron/adt4_output/wednesday_afternoon_1_processed/hydra/backend/dsg_with_mesh.json"
    G = spark_dsg.DynamicSceneGraph.load(dsg_path)

    manual_room_connections = [(0, 1), (1, 2), (2, 3)]
    bb_path = "gt_room_bounding_boxes.yaml"

    extents = RoomExtents(bb_path)

    semantic_labels = [2, 0, 1]

    add_gt_rooms(G, extents.boxes, semantic_labels)
    add_manual_connections(G, manual_room_connections)

    for node in G.get_layer(spark_dsg.DsgLayers.ROOMS).nodes:
        print(node.id)
        print(node.attributes)

    for edge in G.get_layer(spark_dsg.DsgLayers.ROOMS).edges:
        print(edge)

    G.save("test_dsg.json")
