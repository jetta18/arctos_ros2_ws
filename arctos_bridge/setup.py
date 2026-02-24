from setuptools import find_packages, setup

package_name = 'arctos_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    package_data={
        'arctos_bridge': [
            'core/*.py',
            'services/*.py',
            'communication/*.py',
            'protocol/*.py',
            'actions/*.py',
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/arctos_bridge.yaml']),
        ('share/' + package_name + '/launch', ['launch/arctos_bridge.launch.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='michael',
    maintainer_email='mire18@gmx.de',
    description='Communication bridge between the Arctos STM32 firmware and ROS2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arctos_bridge_node = arctos_bridge.arctos_bridge_node:main',
        ],
    },
)
