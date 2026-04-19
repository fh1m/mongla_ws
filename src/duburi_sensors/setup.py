from setuptools import setup, find_packages

package_name = 'duburi_sensors'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name,         ['package.xml']),
        ('share/' + package_name + '/config', ['config/sensors.yaml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='Muhammad Fahim Faisal',
    maintainer_email='fahim.2002.faisal@gmail.com',
    description='Mongla control stack: sensor sources + optional diagnostic node',
    license='MIT',
    entry_points={
        'console_scripts': [
            'sensors_node = duburi_sensors.sensors_node:main',
        ],
    },
)
