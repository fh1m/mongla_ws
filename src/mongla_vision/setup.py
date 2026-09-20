from glob import glob
from setuptools import setup, find_packages

package_name = 'mongla_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name,              ['package.xml']),
        ('share/' + package_name + '/config',  glob('config/*.yaml')),
        # The measured camera intrinsics were NOT installed, so even a launch
        # file that passed the share path would have found nothing there.
        ('share/' + package_name + '/config/calibration',
            glob('config/calibration/*.json')),
        ('share/' + package_name + '/launch',  glob('launch/*.launch.py')),
        ('share/' + package_name + '/models',  glob('models/*.yaml') + glob('models/*.pt') + glob('models/*.hef')),
        # Web console static assets (served by the mission_web node from its source
        # tree via __file__; this install keeps them alongside the installed pkg too).
        ('share/' + package_name + '/web/static', glob('mongla_vision/web/static/*')),
    ],
    install_requires=['setuptools', 'numpy', 'supervision', 'filterpy', 'trackers'],
    zip_safe=True,
    maintainer='Muhammad Fahim Faisal',
    maintainer_email='fh1m.dev@gmail.com',
    description=(
        'Mongla perception: Camera factory + YOLO11 detector (yolov11n) + rich on-image '
        'visualizations. ByteTrack + Kalman tracking in v2/v3.'
    ),
    license='MIT',
    entry_points={
        'console_scripts': [
            'camera_node         = mongla_vision.camera_node:main',
            'detector_node       = mongla_vision.detector_node:main',
            'detector_dual_node  = mongla_vision.detector_dual_node:main',
            'tracker_node        = mongla_vision.tracker_node:main',
            'lock_node           = mongla_vision.lock_node:main',
            'vision_node         = mongla_vision.vision_node:main',
            'vision_check        = mongla_vision.utils.check_pipeline:main',
            'water_check         = mongla_vision.utils.water_check:main',
            'vision_thrust_check = mongla_vision.utils.check_thrust:main',
            'tracker_check       = mongla_vision.utils.check_tracker:main',
            'vision_display      = mongla_vision.utils.display_node:main',
            'export_engine       = mongla_vision.utils.export_engine:main',
            'switch_camera       = mongla_vision.utils.switch_camera:main',
            # Calibration is a MISSION capability: on competition ground a
            # knocked lens or a swapped camera has to be recalibrated, and if
            # it cannot be, the uplink aims with the wrong focal length and
            # the DVL's velocity scale is wrong by the same factor.
            'calibrate           = mongla_vision.calibration.guide:main',
            'calibrate_solve     = mongla_vision.calibration.solver:main',
            'depth_estimation_node = mongla_vision.depth.depth_estimation_node:main',
            'distance_estimation_node = mongla_vision.flow.distance_estimation_node:main',
            'flow_node = mongla_vision.flow.flow_node:main',
            'mission_web         = mongla_vision.web.mission_web_node:main',
        ],
    },
)
