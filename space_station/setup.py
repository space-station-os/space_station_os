from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'space_station'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='siddarth',
    maintainer_email='siddarth.dayasagar@gmail.com',
    description='GUI and simulation framework for SSOS',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'space_station = space_station.main_window:main',
        ],
    },
)
