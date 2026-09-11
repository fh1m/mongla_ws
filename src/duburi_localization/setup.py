from setuptools import setup, find_packages

package_name = 'duburi_localization'

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
    description='Estimation: invariant filter, heading anchor, resection, pose fusion.',
    license='MIT',
    package_data={package_name: ['courses/*.yaml']},
    include_package_data=True,
    entry_points={
        'console_scripts': [
            # Both produce a POSE, which is why they live here and not beside
            # the detector. The launch files still start them; only the
            # package they are fetched from changed.
            'pnp_node       = duburi_localization.pnp_node:main',
            'pose_fuse_node = duburi_localization.pose_fuse_node:main',
        ],
    },
)
