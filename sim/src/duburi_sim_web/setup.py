import os
from glob import glob
from pathlib import Path

from setuptools import find_packages, setup

package_name = 'duburi_sim_web'


def _static_files():
    root = Path('static')
    if not root.is_dir():
        return []
    pairs = []
    for path in root.rglob('*'):
        if path.is_file():
            rel = path.relative_to(root).parent
            dest = os.path.join('share', package_name, 'static', str(rel))
            pairs.append((dest, [str(path)]))
    return pairs


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test', 'frontend']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.yaml')),
        *_static_files(),
    ],
    install_requires=['setuptools', 'fastapi', 'uvicorn', 'python-multipart'],
    zip_safe=True,
    maintainer='fh1m',
    maintainer_email='fh1m@users.noreply.github.com',
    description='Operator web lab for Duburi simulator dataset collection.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'lab_server = duburi_sim_web.server:main',
        ],
    },
)
