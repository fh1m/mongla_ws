from glob import glob

from setuptools import find_packages, setup

package_name = 'mongla_batighor'
THEME_FONTS = ('zed-mono-extended', 'zed-mono-extendedbold',
               'zed-sans-extended', 'zed-sans-extendedbold')

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'LICHTBLICK.json']),
        ('share/' + package_name + '/layouts', glob('layouts/*.json')),
        # Every panel directory, so a new panel ships without an edit here,
        # and the helpers panels.py puts above each one.
        ('share/' + package_name + '/panels', glob('panels/*.js')),
        *[('share/' + package_name + '/' + d, glob(d + '/*'))
          for d in sorted(glob('panels/*')) if glob(d + '/extension.js')],
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/theme', glob('theme/*.css')),
        # The site's own font files, one copy in the repo: Kutum serves them,
        # because the deck has no internet at the pool.
        ('share/' + package_name + '/theme/fonts',
         [f'../../docs/assets/fonts/{f}.woff2' for f in THEME_FONTS]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Muhammad Fahim Faisal',
    description='Batighor, the lighthouse: Kutum serves the operators Lichtblick, '
                'connected to the vehicle and opened on our layouts.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'serve = mongla_batighor.serve:main',
            'fetch = mongla_batighor.fetch:main',
            'overlay = mongla_batighor.overlay:main',
            'helm = mongla_batighor.helm:main',
            'deck = mongla_batighor.deck:main',
            'views = mongla_batighor.views:main',
        ],
    },
)
