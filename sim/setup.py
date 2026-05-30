from setuptools import setup
import os
from glob import glob

package_name = 'sentinel_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'worlds'),  glob('worlds/*.world')),
        (os.path.join('share', package_name, 'urdf'),    glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'config'),  glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'),    glob('maps/*')),
        (os.path.join('share', package_name, 'launch'),  glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Fede',
    maintainer_email='feferreyra98@gmail.com',
    description='DataCenter Sentinel simulation package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'patrol_node         = sentinel_sim.patrol_node:main',
            'virtual_sensor_node = sentinel_sim.virtual_sensor_node:main',
            'sensor_logger_node  = sentinel_sim.sensor_logger_node:main',
        ],
    },
)
