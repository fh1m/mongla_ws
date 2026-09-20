from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'mongla_manager'

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
    maintainer='Muhammad Fahim Faisal',
    description='AUV manager node -- MAVLink connection, live logger, /mongla/move action server',
    license='MIT',
    entry_points={
        'console_scripts': [
            'start            = mongla_manager.auv_manager_node:main',
            'auv_manager      = mongla_manager.auv_manager_node:main',  # kept for compat
            'auv_manager_node = mongla_manager.auv_manager_node:main',  # kept for compat
            'bringup_check    = mongla_manager.bringup_check:main',
            'connect          = mongla_manager.srot_connect:main',
            'flare_order      = mongla_manager.flare_order_send:main',
            'autotune         = mongla_manager.srot_autotune:main',
        ],
    },
)
