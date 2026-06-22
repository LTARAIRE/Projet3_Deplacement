import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'hexapod_locomotion'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.stl')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Lucas Taraire',
    maintainer_email='lucas.taraire@ynov.com',
    description='ROS2 locomotion package for hexapod robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'locomotion_node = hexapod_locomotion.locomotion_node:main',
            'teleop_key_node = hexapod_locomotion.teleop_key_node:main',
            'world_node = hexapod_locomotion.world_node:main',
        ],
    },
)
