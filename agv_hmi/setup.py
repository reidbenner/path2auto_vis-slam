from setuptools import setup
import os
from glob import glob

package_name = 'agv_hmi'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'hmi'), glob('hmi/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'instruction_node = agv_hmi.instruction_node:main',
        ],
    },
)
