from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'nam_patrol_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Base files
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'),
         [os.path.join('resource', package_name)]),
        (os.path.join('share', package_name), ['package.xml']),
        
        # Include all directories
        (os.path.join('share', package_name, 'launch'), 
         glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'urdf'),
         glob(os.path.join('urdf', '*.urdf'))),
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'rviz'),
         glob(os.path.join('rviz', '*.rviz'))),
        (os.path.join('share', package_name, 'worlds'),
         glob(os.path.join('worlds', '*.world'))),
        
        # Include Python source files
        (os.path.join('lib', package_name),
         glob(os.path.join('nam_patrol_robot', '*.py'))),
    ],
    install_requires=['setuptools', 'numpy', 'opencv-python>=4.5.0'],
    zip_safe=True,
    maintainer='nammy',
    maintainer_email='nammy@todo.todo',
    description='Patrolling robot package for home security',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'patrol_node = nam_patrol_robot.patrol_node:main',
            'error_handler = nam_patrol_robot.error_handler:main',          # New
            'advanced_patrol = nam_patrol_robot.advanced_patrol:main'       # New
        ],
    },
)
