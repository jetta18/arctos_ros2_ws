from setuptools import setup

package_name = 'arctos_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=[
        package_name,
        f'{package_name}.backend',
        f'{package_name}.ui',
        f'{package_name}.main',
        f'{package_name}.components',
        f'{package_name}.components.servo_jog',
        f'{package_name}.components.debug',
        f'{package_name}.components.jog',
        f'{package_name}.components.motor_settings',
        f'{package_name}.components.robot_status',
        f'{package_name}.components.homing',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/arctos_gui.launch.py']),
        ('share/' + package_name + '/config', ['config/settings.yaml']),
    ],
    install_requires=['setuptools', 'pyyaml', 'nicegui>=2.0'],
    zip_safe=True,
    maintainer='michael',
    maintainer_email='mire18@gmx.de',
    description='Arctos NiceGUI Web Interface',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arctos_gui = arctos_gui.arctos_gui_main:main',
        ],
    },
)
