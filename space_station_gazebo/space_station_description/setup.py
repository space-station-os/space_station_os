from setuptools import setup
import os
from glob import glob

package_name = 'space_station_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),

        # Install model files individually
        (os.path.join('share', package_name, 'models/earth'), [
            'models/earth/model.config',
            'models/earth/model.sdf',
        ]),
        (os.path.join('share', package_name, 'models/earth/resources'), 
            glob('models/earth/resources/*')),

        # ROS 2 env hook for GZ_SIM_RESOURCE_PATH
        ('share/ament_index/resource_index/environment_hook',
            ['hooks/ssos_description.dsv.in']),
        ('share/' + package_name + '/hooks',
            ['hooks/ssos_description.dsv.in']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='siddarth',
    maintainer_email='siddarth.dayasagar@gmail.com',
    description='The ' + package_name + ' package',
    license='MIT',
    
    entry_points={
        'console_scripts': [],
    },
)
