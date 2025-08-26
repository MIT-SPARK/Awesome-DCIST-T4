#!/usr/bin/env python3

import os

import rich
from rich.table import Table

ADT4_VARS = [
    "ADT4_ROBOT_NAME",
    "ADT4_PLATFORM_ID",
    "ADT4_OUTPUT_DIR",
    "ADT4_PRIOR_MAP",
    "ADT4_SIM_TIME",
]


def main():
    env = os.environ.copy()
    table = Table(title="Environment Variables")
    table.add_column("Environment Variable", justify="left", style="cyan")
    table.add_column("Value", justify="right")
    for name in ADT4_VARS:
        table.add_row(name, env.get(name) or "---")

    rich.print(table)


if __name__ == "__main__":
    main()
