from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    robot_path = get_package_share_directory('ros_cpp')
    return LaunchDescription([
        Node(
            package='ublox_gps',
            executable='ublox_gps_node',
        ),
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            #parameters= [os.path.join(robot_path, 'config', 'cam_parameters.yaml')],
        ),
        Node(
            package='ros_cpp',
            executable='sensors_sub',
        ),
        Node(
            package='joy',
            executable='joy_node',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d',os.path.join(robot_path,'rviv','camera.rviz')],
        ),
    ])
