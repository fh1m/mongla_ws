from glob import glob
from setuptools import setup, find_packages

package_name = 'duburi_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name,              ['package.xml']),
        ('share/' + package_name + '/config',  glob('config/*.yaml')),
        ('share/' + package_name + '/launch',  glob('launch/*.launch.py')),
        ('share/' + package_name + '/models',  glob('models/*.yaml') + glob('models/*.pt')),
    ],
    install_requires=['setuptools', 'numpy', 'supervision', 'filterpy', 'trackers'],
    zip_safe=True,
    maintainer='Muhammad Fahim Faisal',
    maintainer_email='fahim.2002.faisal@gmail.com',
    description=(
        'Duburi perception: Camera factory + YOLO11 detector (yolov11n) + rich on-image '
        'visualizations. ByteTrack + Kalman tracking in v2/v3.'
    ),
    license='MIT',
    entry_points={
        'console_scripts': [
            'camera_node         = duburi_vision.camera_node:main',
            'detector_node       = duburi_vision.detector_node:main',
            'tracker_node        = duburi_vision.tracker_node:main',
            'vision_node         = duburi_vision.vision_node:main',
            'vision_check        = duburi_vision.utils.check_pipeline:main',
            'vision_thrust_check = duburi_vision.utils.check_thrust:main',
            'tracker_check       = duburi_vision.utils.check_tracker:main',
            'vision_display      = duburi_vision.utils.display_node:main',
            'export_engine       = duburi_vision.utils.export_engine:main',
            'depth_estimation_node = duburi_vision.depth.depth_estimation_node:main',
        ],
    },
)
