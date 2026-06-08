import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'archer_sim'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alex-T-27',
    maintainer_email='alextran201007@gmail.com',
    description='Gazebo simulation of the Skeleton Archer (P2).',
    license='MIT',
    entry_points={
        'console_scripts': [
            'sequencer = archer_sim.sequencer:main',
        ],
    },
)
