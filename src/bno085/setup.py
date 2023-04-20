from setuptools import setup

package_name = 'bno085'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robert',
    maintainer_email='robertlstevenson01@gmail.com',
    description='ROS2 Node for interface via I2C with a BNO085 Sensor and publish the IMU msg data and the Sensor orientation (roll, pitch, yaw/heading) ',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bno085_publisher = bno085.BNO085_pub:main',
        ],
    },
)
