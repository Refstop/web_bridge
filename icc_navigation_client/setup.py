from setuptools import setup

import glob
import os

package_name = 'icc_navigation_client'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', '*.py'))),
        ('share/' + package_name + '/rooms', glob.glob(os.path.join('rooms', '*.txt'))),
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
            'navigation_client = icc_navigation_client.navigation_client:main'
        ],
    },
)
