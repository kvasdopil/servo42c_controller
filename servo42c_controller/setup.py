import os
from glob import glob
from setuptools import setup

package_name = 'servo42c_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'description'), glob('description/*.urdf')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='Alexey Guskov',
    maintainer_email='kvasdopil@gmail.com',
    description='Servo42C Controller ROS2 package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo42c_controller = servo42c_controller.node:main',
            'test_trajectory = servo42c_controller.test_trajectory:main',
        ],
    },
)