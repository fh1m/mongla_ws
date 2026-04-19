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
    ],
    install_requires=['setuptools', 'pymavlink'],
    zip_safe=True,
    maintainer='BRACU Duburi',
    description='AUV manager node — MAVLink connection, live logger, command dispatcher',
    license='MIT',
    entry_points={
        'console_scripts': [
            'auv_manager       = duburi_manager.auv_manager_node:main',
            'auv_manager_node  = duburi_manager.auv_manager_node:main',
            'test_runner       = duburi_manager.test_runner:main',
            'duburi            = duburi_manager.cli:main',
        ],
    },
)
