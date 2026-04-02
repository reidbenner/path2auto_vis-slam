from setuptools import find_packages, setup

package_name = 'agv_imu'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, ['scripts/imu_publisher']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='agv',
    maintainer_email='todo@todo.com',
    description='IMU publisher for ifm O3R port6',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'imu_publisher = agv_imu.imu_publisher:main',
        ],
    },
)
