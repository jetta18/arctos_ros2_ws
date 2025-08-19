from setuptools import setup

package_name = 'arctos_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/topic_based_bringup.launch.py']),
        ('share/' + package_name + '/config', ['config/topic_based.yaml']),
        ('share/' + package_name, ['README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Michael',
    maintainer_email='michael@example.com',
    description='Arctos control bridge and tools for topic_based_ros2_control and micro-ROS (ESP32)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim_esp32 = arctos_control.sim_esp32:main',
            'watchdog = arctos_control.watchdog:main',
        ],
    },
)
