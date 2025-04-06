from setuptools import find_packages, setup

package_name = 'web_joy_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'websockets'],
    zip_safe=True,
    maintainer='mj',
    maintainer_email='marcus.j.hsieh@gmail.com',
    description='Joysticks over WebSocket for ROS2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_joy_node = web_joy_publisher.web_joy_node:main',
        ],
    },
)
