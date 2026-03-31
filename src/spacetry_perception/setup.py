from setuptools import setup
from pathlib import Path

package_name = 'spacetry_perception'

# Robustness: ensure the ament resource marker exists even if missing in the repo
Path('resource').mkdir(exist_ok=True)
(Path('resource') / package_name).touch(exist_ok=True)

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SpaceTry',
    maintainer_email='ricardo.caldas@gssi.it',
    description='Perception helpers for SpaceTry (LiDAR obstacle direction topics).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_direction_node = spacetry_perception.obstacle_direction_node:main',
            'odom_relay_node = spacetry_perception.odom_relay_node:main',
        ],
    },
)
