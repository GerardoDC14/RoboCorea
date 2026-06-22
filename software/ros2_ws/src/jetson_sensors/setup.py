from glob import glob
from setuptools import find_packages, setup


package_name = 'jetson_sensors'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gerardo',
    maintainer_email='gerardo@example.com',
    description='Jetson I2C sensor drivers for RoboCorea thermal and magnetic sensors.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mlx90640_node = jetson_sensors.mlx90640_node:main',
            'lis3mdl_node = jetson_sensors.lis3mdl_node:main',
            'thermal_visualizer_node = jetson_sensors.thermal_visualizer_node:main',
        ],
    },
)
