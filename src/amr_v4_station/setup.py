import os
from glob import glob
from setuptools import setup

package_name = 'amr_v4_station'

setup(
    name=package_name,
    version='0.0.0',
    # Packages to export
    packages=[package_name],
    package_dir={package_name: 'src/'},
    # Files we want to install, specifically launch files
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    # This is important as well
    install_requires=['setuptools'],
    zip_safe=True,
    author='ROS 2 Developer',
    author_email='ros2@ros.com',
    maintainer='ROS 2 Developer',
    maintainer_email='ros2@ros.com',
    keywords=['foo', 'bar'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: TODO',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='My awesome package.',
    license='TODO',
    # Like the CMakeLists add_executable macro, you can add your python
    # scripts here.
    entry_points={
        'console_scripts': [
            'station = amr_v4_station.station:main',
        ],
    },
)