import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'duburi_sim_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fh1m',
    maintainer_email='fh1m@users.noreply.github.com',
    description='ros_gz bridge, underwater_fx, and camera dataset recorder.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'contract_check = duburi_sim_bridge.contract_check:main',
            'mavlink_check = duburi_sim_bridge.mavlink_check:main',
            'record_cameras = duburi_sim_bridge.record_cameras:main',
            'underwater_fx = duburi_sim_bridge.underwater_fx:main',
            'dvl_bridge = duburi_sim_bridge.dvl_bridge:main',
        ],
    },
)
