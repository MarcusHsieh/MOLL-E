from setuptools import find_packages, setup

package_name = 'jetson_csi_cam'

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
    description='Publishes compressed CSI camera images from Jetson Nano',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'csi_cam_publisher = jetson_csi_cam.csi_cam_publisher:main'
        ],
    },
)
