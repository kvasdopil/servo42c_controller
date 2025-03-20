from setuptools import find_packages, setup

package_name = 'arm_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/description', ['description/arm.urdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alexey Guskov',
    maintainer_email='kvasdopil@gmail.com',
    description='Robot arm controller',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_state_publisher = arm_controller.arm_state_publisher:main',
        ],
    },
)
