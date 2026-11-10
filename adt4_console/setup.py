import pathlib

from setuptools import find_packages, setup

package_name = "adt4_console"
here = pathlib.Path(__file__).parent


def static_files():
    """Ship the built SPA (adt4_console/static/**) as package data, if present."""
    root = here / package_name / "static"
    return [
        str(p.relative_to(here / package_name)) for p in root.rglob("*") if p.is_file()
    ]


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["tests"]),
    package_data={package_name: static_files()},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="harel",
    maintainer_email="harelb@mit.edu",
    description="ADT4 web console: scene graph / map viewer",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["adt4-console = adt4_console.cli:main"]},
)
