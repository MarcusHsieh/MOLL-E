from setuptools import find_packages, setup

package_name = 'motor_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mj',
    maintainer_email='marcus.j.hsieh@gmail.com',
    description='Motor control node for MOLL-E using PCA9685 and VEX motor controllers',
    license='Apache-2.0',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_controller_node = molle_motor.motor_controller_node:main'
        ],
    },
)
