import os
from glob import glob

from setuptools import setup

package_name = 'arm_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'meshes', 'visual'), glob('meshes/visual/*')),
        (os.path.join('share', package_name, 'meshes', 'collision'), glob('meshes/collision/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RoboCorea',
    maintainer_email='nabetse069@gmail.com',
    description='Singularity-robust SDLS servo + keyboard/joystick teleop for the dicerox arm.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'sdls_servo = arm_teleop.sdls_servo:main',
            'keyboard_servo = arm_teleop.keyboard_servo:main',
            'joystick_servo = arm_teleop.joystick_servo:main',
        ],
    },
)
