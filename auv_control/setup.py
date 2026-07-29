from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'auv_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='ifuwelemuel123@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'actuator_mixer = auv_control.actuator_mixer:main',
            'teleop_node = auv_control.teleop_node:main',
            'actuator_driver = auv_control.actuator_driver:main',
            'pca9685_driver = auv_control.pca9685_driver:main',
            'heading_controller = auv_control.heading_controller:main',
            'depth_sensor = auv_control.depth_sensor:main',
            'depth_controller = auv_control.depth_controller:main',
            'los_guidance = auv_control.los_guidance:main',
            'waypoint_manager = auv_control.waypoint_manager:main',
        ],
    },
)
