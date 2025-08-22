import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'tesikaga_lidar_detector'

setup(
    name=package_name,
    version='0.1.0',
    packages=['tesikaga_lidar_detector', 'scripts'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'srv'), glob(os.path.join('srv', '*.srv'))),
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
            'run_calibration = scripts.run_calibration:main',
            'capture_point = scripts.capture_point:main',
            'calculate_transform = scripts.calculate_transform:main',
            'clear_points = scripts.clear_points:main',
        ],
    },
)