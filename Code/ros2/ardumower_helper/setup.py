from setuptools import setup

package_name = 'ardumower_helper'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='paddy-daun@web.de',
    description='Ardumower base controller, interface between Ardumower messages and ROS 2',
    license='Apache License 2.0',
    tests_require=['pytest']

)
