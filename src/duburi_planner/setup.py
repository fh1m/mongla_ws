from setuptools import setup, find_packages

package_name = 'duburi_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mongla project',
    description=(
        'Mission planner: DuburiClient Python API + `duburi` CLI + '
        'mission scripts. Reserved space for YASMIN state machines.'
    ),
    license='MIT',
    entry_points={
        'console_scripts': [
            # Operator CLI: `ros2 run duburi_planner duburi <cmd> [...]`
            'duburi  = duburi_planner.cli:main',
            # Mission runner: `ros2 run duburi_planner mission <name>`
            'mission = duburi_planner.mission:main',
        ],
    },
)
