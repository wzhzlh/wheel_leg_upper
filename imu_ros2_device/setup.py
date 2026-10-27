from setuptools import find_packages, setup

package_name = 'imu_ros2_device'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='admin',
    maintainer_email='2902981758@qq.com',
    description='YbImu 串口 IMU 驱动，发布 sensor_msgs/Imu 到 /imu/data',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ybimu_driver = imu_ros2_device.ybimu_driver:main'
        ],
    },
)
