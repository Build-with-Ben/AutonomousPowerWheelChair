from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'rnet_bridge'

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
    maintainer='ben-niemann',
    maintainer_email='benjamin.f.niemann@gmail.com',
    description=('Bridge between ROS 2 and the R-Net CAN bus: follows the joystick '
                 'module to inject commands and decodes power module status.'),
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'bridge_node = rnet_bridge.bridge_node:main',
        ],
    },
)
