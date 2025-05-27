from scipy.spatial.transform import Rotation as R
import numpy as np
import spark_dsg as dsg
import pathlib
import click


def _get_first_pose(G):
    agents = G.get_layer(2, "a")
    for node in agents.nodes:
        pos = node.attributes.position
        rot = node.attributes.world_R_body
        world_T_body = np.eye(4)
        world_T_body[:3, :3] = R.from_quat([rot.x, rot.y, rot.z, rot.w]).as_matrix()
        world_T_body[:3, 3] = pos
        return world_T_body


def _inverse(a_T_b):
    b_T_a = np.eye(4)
    b_T_a[:3, :3] = a_T_b[:3, :3].T
    b_T_a[:3, 3] = -b_T_a[:3, :3] @ a_T_b[:3, 3]
    return b_T_a


@click.command()
@click.argument("input_path", type=click.Path(exists=True))
@click.argument("aligned_path", type=click.Path(exists=True))
@click.argument("output_path", type=click.Path())
def main(input_path, aligned_path, output_path):
    input_path = pathlib.Path(input_path).expanduser().resolve()
    aligned_path = pathlib.Path(aligned_path).expanduser().resolve()
    output_path = pathlib.Path(output_path).expanduser().absolute()

    input_files = sorted(list(input_path.rglob("*dsg.json")))
    for input_graph in input_files:
        subpath = input_graph.relative_to(input_path)
        aligned_graph = aligned_path / subpath
        output_graph = output_path / subpath
        if not aligned_path.exists():
            click.secho(f"Error: missing '{aligned_graph}' for input '{input_graph}'", fg="red")
            continue

        G_input = dsg.DynamicSceneGraph.load(input_graph)
        G_aligned = dsg.DynamicSceneGraph.load(aligned_graph)

        input_T_body = _get_first_pose(G_input)
        aligned_T_body = _get_first_pose(G_aligned)
        aligned_T_unaligned = aligned_T_body @ _inverse(input_T_body)
        G_input.transform(aligned_T_unaligned)

        click.secho(f"Outputting to '{output_graph}'", fg="green")
        output_graph.parent.mkdir(exist_ok=True, parents=True)
        G_input.save(output_graph)


if __name__ == "__main__":
    main()
