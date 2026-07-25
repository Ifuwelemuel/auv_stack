import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'auv_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch and config so ros2 launch can find them:
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='lemuelossaiifuwe@gmail.com',
    description='Launch files and configuration for AUV bring-up.',
    license='MIT',
    entry_points={'console_scripts': []},
)