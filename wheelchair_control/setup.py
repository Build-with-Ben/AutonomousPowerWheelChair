from setuptools import find_packages, setup

package_name = 'wheelchair_control'

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
    description='Speed and acceleration limits and mode gating for drive commands',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'control_node = wheelchair_control.control_node:main',
        ],
    },
)
