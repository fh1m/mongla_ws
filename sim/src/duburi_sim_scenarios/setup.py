import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'duburi_sim_scenarios'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fh1m',
    maintainer_email='fh1m@users.noreply.github.com',
    description='Runtime prop control for the Duburi AUV simulator.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'prop_manager = duburi_sim_scenarios.prop_manager:main',
            'props = duburi_sim_scenarios.cli:main',
            'gate_transit_check = '
            'duburi_sim_scenarios.gate_transit_check:main',
        ],
    },
)
