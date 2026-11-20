from setuptools import setup
setup(
    name="phoenix_task_executor", version="0.1.0",
    packages=["phoenix_task_executor"],
    data_files=[("share/ament_index/resource_index/packages", ["resource/phoenix_task_executor"]),
                ("share/phoenix_task_executor", ["package.xml", "REVIEW.txt"]),
                ("share/phoenix_task_executor/launch", ["launch/gauss_sequential_tasks.launch.yaml", "launch/phoenix_task_executor.launch.yaml"])],
    install_requires=["setuptools"],
    entry_points={"console_scripts": ["phoenix_task_executor = phoenix_task_executor.node:main"]},
)
