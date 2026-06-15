from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'space_station'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['space_station/.env']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    # Bundle runtime assets (images, videos, fonts) inside the python package
    # so non-symlink installs still find them via importlib.resources.
    package_data={
        package_name: [
            'assets/*',
            'assets/fonts/*',
            'assets/iss/*',
        ],
    },
    include_package_data=True,
    install_requires=['setuptools', 'pyqtgraph', 'trimesh'],
    zip_safe=True,
    maintainer='siddarth',
    maintainer_email='siddarth.dayasagar@gmail.com',
    description='GUI and simulation framework for SSOS',
    license='MIT',
    entry_points={
        'console_scripts': [
            'space_station = space_station.main_window:main',
        ],
    },
)
