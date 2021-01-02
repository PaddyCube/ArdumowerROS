from setuptools import setup

package_name = 'ardumower_driver'

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
    description='Ardumower base driver, communication to Arduino',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ardumower_driver = ardumower_driver.ardumower_driver:main'
        ],
    },
)
