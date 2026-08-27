import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'g1_walk'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config', 'paths'),
            glob('config/paths/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nirav',
    maintainer_email='niravpanchalmerai@gmail.com',
    description='Open-loop fixed-path walking for the Unitree G1',
    license='MIT License',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'loco_bridge = g1_walk.loco_bridge:main',
            'path_follower = g1_walk.path_follower:main',
            'state_bridge = g1_walk.state_bridge:main',
            'loco_cli = g1_walk.loco_cli:main',
            'fake_loco_server = g1_walk.fake_loco_server:main',
            'check_path = g1_walk.check_path:main',
            'preflight = g1_walk.preflight:main',
            'odom_bridge = g1_walk.odom_bridge:main',
        ],
    },
)
