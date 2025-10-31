from setuptools import setup
import os
from glob import glob

package_name = 'multi_nav2'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # Include the package.xml
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Install launch, map, and param directories
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'map'), glob('map/*')),
        (os.path.join('share', package_name, 'param'), glob('param/**/*', recursive=True)),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mahmoud Aboelrayat',
    maintainer_email='your_email@example.com',
    description='Multi-robot navigation configuration and launch package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Add any Python executable nodes here if you have any
        ],
    },
)
