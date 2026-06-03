from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'robot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cst',
    maintainer_email='1656189150@qq.com',
    description='Launch and interface bridge package for simulation and real robot bringup.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'interface_bridge = robot_bringup.interface_bridge:main',
        ],
    },
)
