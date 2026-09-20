from setuptools import setup, find_packages

package_name = 'mongla_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    # The course priors travel WITH the package: a mission that loads one on
    # the vehicle must not depend on the source tree being present, and the
    # deck override (~/.mongla/courses) is searched before this copy anyway.
    package_data={package_name: ['courses/*.yaml']},
    include_package_data=True,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Muhammad Fahim Faisal',
    description=(
        'Mission planner: MonglaClient Python API + `mongla` CLI + '
        'mission scripts. Reserved space for YASMIN state machines.'
    ),
    license='MIT',
    entry_points={
        'console_scripts': [
            # Operator CLI: `ros2 run mongla_planner mongla <cmd> [...]`
            'mongla  = mongla_planner.cli:main',
            # Mission runner: `ros2 run mongla_planner mission <name>`
            'mission = mongla_planner.mission:main',
        ],
    },
)
