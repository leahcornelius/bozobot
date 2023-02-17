from setuptools import setup
from glob import glob
import os

package_name = 'bozobot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name), glob('config/*.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Leah Cornelius',
    maintainer_email='leocornelius@gmail.com',
    description='Core ROS2 package for the BozoBot robot, to be run on the Raspberry Pi',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'telop_subscriber = bozobot.telop_subscriber:main',
        ],
    },
)
