from setuptools import setup

package_name = 'space_station_gnc'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your-email@example.com',
    description='Description of your package',
    license='License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'demo_crisis = space_station_gnc.demo_crisis:main',
        ],
    },
)

