from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'agv_person_detection'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bart',
    maintainer_email='bart@todo.com',
    description='Person detection with proximity warning using YOLOv8 and ifm O3R depth',
    license='MIT',
    entry_points={
        'console_scripts': [
            'person_detector = agv_person_detection.person_detector:main',
        ],
    },
)
