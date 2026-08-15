import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'jackal_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ali',
    maintainer_email='ali@safirlab.org',
    description='YOLOv8 object detection for the Clearpath Jackal',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'yolo_detector_node = jackal_vision.yolo_detector_node:main',
        ],
    },
)
