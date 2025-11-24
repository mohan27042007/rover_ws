import os
import sys

# Force interpreter
os.environ["PYTHON_EXECUTABLE"] = "/home/mohanarangan-t-r/ml_env_312/bin/python3"

from setuptools import find_packages, setup

package_name = 'rover_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', 
            ['launch/perception.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mohanarangan-t-r',
    maintainer_email='mail4mohan27@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },

    entry_points={
        'console_scripts': [
            'boulder_detector = rover_perception.boulder_detector:main',
            'crater_detector = rover_perception.crater_detector:main',
            'perception_ensemble = rover_perception.ensemble_node:main',
        ],
    },

)
