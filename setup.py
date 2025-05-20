from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'odom_computation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vignesh',
    maintainer_email='vanavenkatavignesh630@gmail.com',
    description='ROS2 Odometry from encoder ticks',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wheel_tick_pub = odom_computation.wheel_tick_pub_ros2:main',
            'odom_calc_node = odom_computation.odom_calc_node:main',
        ],
    },
)
