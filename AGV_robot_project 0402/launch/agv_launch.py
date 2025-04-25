# File: agv_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    package_dir = os.path.join(os.getenv('HOME'), 'ros2_humble', 'src', 'agv_robot_project')

    camera_process = ExecuteProcess(
        cmd=['bash', os.path.join(package_dir, 'scripts', 'camerastream.sh')],
        output='screen'
    )

    connect = Node(package='agv_robot_project', executable='connect_node', output='screen')
    drive = Node(package='agv_robot_project', executable='drive_node', output='screen')
    ethernet = Node(package='agv_robot_project', executable='ethernet_node', output='screen')
    lidar = Node(package='agv_robot_project', executable='lidar_node', output='screen')
    pico = Node(package='agv_robot_project', executable='pico_node', output='screen')
    watch = Node(package='agv_robot_project', executable='watchdog_node', output='screen')

    return LaunchDescription([
        camera_process,
        connect,
        # drive,
        # ethernet,
        # lidar,
        # pico,
        watch
    ])