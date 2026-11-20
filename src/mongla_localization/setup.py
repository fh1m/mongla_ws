from setuptools import setup, find_packages

package_name = 'mongla_localization'

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
    description='Estimation: invariant filter, course priors, heading anchor, resection.',
    license='MIT',
    package_data={package_name: ['courses/*.yaml', 'frames.yaml']},
    include_package_data=True,
    entry_points={
        'console_scripts': [
            'localization_node = mongla_localization.localization_node:main',
            'course_survey     = mongla_localization.course_survey:main',
        ],
    },
)
