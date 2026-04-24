from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'duburi_manager'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'pymavlink'],
    zip_safe=True,
    maintainer='Mongla project',
    description='AUV manager node -- MAVLink connection, live logger, /duburi/move action server',
    license='MIT',
    entry_points={
        'console_scripts': [
            'start            = duburi_manager.auv_manager_node:main',
            'auv_manager      = duburi_manager.auv_manager_node:main',  # kept for compat
            'auv_manager_node = duburi_manager.auv_manager_node:main',  # kept for compat
            'bringup_check    = duburi_manager.bringup_check:main',
        ],
    },
)
