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
        # The measured camera intrinsics were NOT installed, so even a launch
        # file that passed the share path would have found nothing there.
        ('share/' + package_name + '/config/calibration',
            glob('config/calibration/*.json')),
        ('share/' + package_name + '/launch',  glob('launch/*.launch.py')),
        ('share/' + package_name + '/models',  glob('models/*.yaml') + glob('models/*.pt') + glob('models/*.hef')),
        # Web console static assets (served by the mission_web node from its source
        # tree via __file__; this install keeps them alongside the installed pkg too).
        ('share/' + package_name + '/web/static', glob('duburi_vision/web/static/*')),
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
            'detector_dual_node  = duburi_vision.detector_dual_node:main',
            'tracker_node        = duburi_vision.tracker_node:main',
            'lock_node           = duburi_vision.lock_node:main',
            'pnp_node            = duburi_vision.pnp_node:main',
            'vision_node         = duburi_vision.vision_node:main',
            'vision_check        = duburi_vision.utils.check_pipeline:main',
            'water_check         = duburi_vision.utils.water_check:main',
            'vision_thrust_check = duburi_vision.utils.check_thrust:main',
            'tracker_check       = duburi_vision.utils.check_tracker:main',
            'vision_display      = duburi_vision.utils.display_node:main',
            'export_engine       = duburi_vision.utils.export_engine:main',
            'switch_camera       = duburi_vision.utils.switch_camera:main',
            # Calibration is a MISSION capability: on competition ground a
            # knocked lens or a swapped camera has to be recalibrated, and if
            # it cannot be, the uplink aims with the wrong focal length and
            # the DVL's velocity scale is wrong by the same factor.
            'calibrate           = duburi_vision.calibration.guide:main',
            'calibrate_solve     = duburi_vision.calibration.solver:main',
            'depth_estimation_node = duburi_vision.depth.depth_estimation_node:main',
            'distance_estimation_node = duburi_vision.flow.distance_estimation_node:main',
            'flow_node = duburi_vision.flow.flow_node:main',
            'mission_web         = duburi_vision.web.mission_web_node:main',
        ],
    },
)
