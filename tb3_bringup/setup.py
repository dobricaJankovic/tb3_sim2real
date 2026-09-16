import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'tb3_bringup'


def share(*parts):
    return os.path.join('share', package_name, *parts)


setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [os.path.join('resource', package_name)]),
        (share(), ['package.xml']),
        # launch/ is nested, so each level needs its own entry
        (share('launch'), glob('launch/*.launch.py')),
        (share('launch', 'backends'), glob('launch/backends/*.launch.py')),
        (share('launch', 'common'), glob('launch/common/*.launch.py')),
        (share('config'), glob('config/*.yaml')),
        # No maps/ here: a map belongs to an environment, not to this package,
        # so it lives in the world's own directory (worlds/<name>/map/).
        (share('rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='MihStev',
    maintainer_email='jd220010d@student.etf.bg.ac.rs',
    description='One bringup, three backends: real TurtleBot3, Gazebo Classic, Isaac Sim.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wait_for_sim = tb3_bringup.wait_for_sim:main',
            'drive_test = tb3_bringup.drive_test:main',
            'nav_test = tb3_bringup.nav_test:main',
            'scan_test = tb3_bringup.scan_test:main',
        ],
    },
)
