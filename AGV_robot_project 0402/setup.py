import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'agv_robot_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
        [f for f in glob('launch/*') if not os.path.isdir(f)]),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh')),
        (os.path.join('share', package_name, 'config'), ['.env']),
        (os.path.join('share', package_name, 'scripts'), ['.env'])
    ],
    install_requires=['setuptools','python-socketio','python-dotenv','pyserial'],
    zip_safe=True,
    maintainer='cghwang',
    maintainer_email='hwangwin7@naver.com',
    description='agv_robot_project',
    license='kinetix',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'connect_node = agv_robot_project.connect_node:main',
            'drive_node = agv_robot_project.drive_node:main',
            'ethernet_node = agv_robot_project.ethernet_node:main',
            'lidar_node = agv_robot_project.lidar_node:main',
            'pico_node = agv_robot_project.pico_node:main',
            'watchdog_node = agv_robot_project.watchdog_node:main',
        ],
    },
)