from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'esp32_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='arepo',
    maintainer_email='nabetse069@gmail.com',
    description='RoboCorea ESP32 <-> ROS 2 serial bridge (Jetson side).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'esp32_bridge = esp32_bridge.main_bridge:main',
            'can_presence_check = esp32_bridge.can_presence_check:main',
        ],
    },
)
