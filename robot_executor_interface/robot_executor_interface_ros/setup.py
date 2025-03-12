from setuptools import find_packages, setup

package_name = "robot_executor_interface_ros"

setup(
    name=package_name,
    version="0.0.0",
    package_dir={"": "src"},
    packages=find_packages("src"),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="aaron",
    maintainer_email="aaronray@mit.edu",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
