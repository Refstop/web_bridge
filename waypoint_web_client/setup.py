from setuptools import setup

import glob
import os

package_name = 'waypoint_web_client'
share_dir = 'share/' + package_name

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (share_dir, ['package.xml']),
        (share_dir + '/waypoints', glob.glob(os.path.join('waypoints', '*.txt'))),
        (share_dir + '/launch', glob.glob(os.path.join('launch', '*.launch.py'))),
        (share_dir + '/map', glob.glob(os.path.join('map', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sparo',
    maintainer_email='bhbhchoi@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_web_client = waypoint_web_client.waypoint_web_client:main'
        ],
    },
)
