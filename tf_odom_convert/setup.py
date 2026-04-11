from setuptools import find_packages, setup

package_name = 'tf_odom_convert'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='todo',
    maintainer_email='todo@todo.com',
    description='Utilities for converting between TF transforms and odometry topics',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'tf_to_odom_node = tf_odom_convert.tf_to_odom_node:main',
            'odom_to_tf_node = tf_odom_convert.odom_to_tf_node:main',
        ],
    },
)
