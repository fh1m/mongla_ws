from setuptools import setup, find_packages

package_name = 'mongla_runtime'

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
    maintainer='Muhammad Fahim Faisal',
    description='How a Mongla process runs: spin, parameters, launch defaults, restarts.',
    license='MIT',
)
