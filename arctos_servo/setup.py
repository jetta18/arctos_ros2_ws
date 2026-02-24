from setuptools import setup
from glob import glob
import os

package_name = 'arctos_servo'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Michael',
    maintainer_email='mire18@gmx.de',
    description='MoveIt Servo integration for real-time cartesian teleop of the Arctos robot',
    license='BSD',
    entry_points={
        'console_scripts': [
            'keyboard_teleop = arctos_servo.keyboard_teleop:main',
        ],
    },
)
