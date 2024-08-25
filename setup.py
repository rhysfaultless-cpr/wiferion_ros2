import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'wiferion-ros2'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rhys Faultless',
    maintainer_email='rfaultless@clearpathrobotics.com',
    description='Read CAN data from Wiferion etaLink mobile robot, publish as ROS topic',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wiferion-ros2 = wiferion-ros2.wiferion-ros2:main',
        ],
    },
)