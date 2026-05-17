from setuptools import setup
import os
from glob import glob

package_name = 'car_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name, 'launch'),  glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mohanad',
    maintainer_email='mohanad@example.com',
    description='ROS2 controller for 4WD car with L298N motor driver',
    license='MIT',
    entry_points={
        'console_scripts': [
            'motor_controller = car_control.motor_controller_node:main',
            'patrol           = car_control.patrol_node:main',
            'scan_to_range    = car_control.scan_to_range_node:main',
        ],
    },
)
