from glob import glob

from setuptools import find_packages, setup

package_name = 'mongla_batighor'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'LICHTBLICK.json']),
        ('share/' + package_name + '/layouts', glob('layouts/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Muhammad Fahim Faisal',
    description='Batighor, the lighthouse: Kutum serves the operators Lichtblick, '
                'connected to the vehicle and opened on our layouts.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'serve = mongla_batighor.serve:main',
            'fetch = mongla_batighor.fetch:main',
            'overlay = mongla_batighor.overlay:main',
        ],
    },
)
