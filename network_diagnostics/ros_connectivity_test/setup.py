from setuptools import find_packages, setup
from glob import glob

package_name = 'ros_connectivity_test'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='harel',
    maintainer_email='harel@mit.edu',
    description='ROS2 connectivity test nodes for ADT4 multi-robot system',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'connectivity_pub = ros_connectivity_test.pub_node:main',
            'connectivity_sub = ros_connectivity_test.sub_node:main',
            'bw_monitor = ros_connectivity_test.bw_monitor_node:main',
        ],
    },
)
