import json
import pathlib

import click


def _get_id(name, order):
    return order.index(name)


@click.command()
@click.argument("input_path", type=click.Path(exists=True))
@click.argument("order", nargs=-1)
@click.option("--output", "-o", type=click.Path(), default=None)
def main(input_path, order, output):
    order = list(order)
    input_path = pathlib.Path(input_path).expanduser().resolve()
    if output is None:
        output = input_path.parent / f"{input_path.stem}_ordered.json"
    else:
        output = pathlib.Path(output).expanduser().absolute()

    with input_path.open("r") as fin:
        loop_closures = json.load(fin)

    for lc in loop_closures:
        lc["robot_from"] = _get_id(lc["robot_from"], order)
        lc["robot_to"] = _get_id(lc["robot_to"], order)

    with output.open("w") as fout:
        json.dump(loop_closures, fout)


if __name__ == "__main__":
    main()
