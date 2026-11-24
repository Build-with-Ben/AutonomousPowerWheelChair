from setuptools import find_packages, setup

package_name = 'wheelchair_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ben-niemann',
    maintainer_email='benjamin.f.niemann@gmail.com',
    description='Manual control: gamepad or joystick to velocity command, with a deadman button.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'teleop_node = wheelchair_teleop.teleop_node:main',
        ],
    },
)
