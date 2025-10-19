from setuptools import find_packages, setup

package_name = 'turtlebot_gazebo_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mohamed',
    maintainer_email='mohamedmajdi0002@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'velocity_publisher = turtlebot_gazebo_simulation.velocity_publisher:main',
            'goal_controller = turtlebot_gazebo_simulation.goal_controller:main',
            '4_robots_cw_topology = turtlebot_gazebo_simulation.4_robots_cw_topology:main',
        ],
    },
)