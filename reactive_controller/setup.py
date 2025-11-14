from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'reactive_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'resource'), glob('resource/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mohamed',
    maintainer_email='mohamedmajdi0002@gmail.com',
    description='Reactive controller package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'reactive_waypoint_controller = reactive_controller.reactive_waypoint_controller:main',
            'astar_global_planner = reactive_controller.astar_global_planner:main',
            'reactive_waypoint_controller_multirobot = reactive_controller.reactive_waypoint_controller_multirobot:main',
            'astar_global_planner_multirobot = reactive_controller.astar_global_planner_multirobot:main',
            'set_initial_pose = reactive_controller.set_initial_pose:main',
            'room_coordinator = reactive_controller.room_coordinator:main',
        ],
    },
)
