import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'tesikaga_lidar_detector'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(where='.'),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ryuddi',
    maintainer_email='koyasuryu@gmail.com',
    description='LiDAR-based object detector for Tesikaga Art.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detector_node = tesikaga_lidar_detector.object_cluster_detector_node:main',
            'calibration_node = tesikaga_lidar_detector.calibration_node:main',
            'run_calibration = tesikaga_lidar_detector.scripts.run_calibration:main',
        ],
    },
)