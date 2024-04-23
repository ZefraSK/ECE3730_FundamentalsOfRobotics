from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import SetParameter
import os

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    robot_path = get_package_share_directory('ros_cpp')
    pose_path = get_package_share_directory('ros_cpp')
	
    executive = Node(
            package='ros_bt',
            executable='executive',
            output='screen',
        )
        
    gz_sim = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': '-r '+os.path.join(robot_path,'worlds/tugbot_depot.sdf'+' -s --headless-rendering')}.items(),
        )
        
    getPose = Node(
            package='ros_bt',
            executable='get_pose_server',
            output='screen',
        )
        
    nav2 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
            launch_arguments={'slam_params_file': os.path.join(robot_path,'config/mapper_params_online_async.yaml')}.items(),
        )
        
    slam = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_sync_launch.py')),
            launch_arguments={'slam_params_file': os.path.join(robot_path,'config/mapper_params_online_async.yaml')}.items(),
        )

        # RViz
    rviz = Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(robot_path, 'rviz', 'robot.rviz')],
        )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/navsat@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
            '/navsat@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
            '/magnetometer@sensor_msgs/msg/MagneticField[gz.msgs.Magnetometer',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        output='screen'
    )

    apriltags = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        remappings=[
            ('image_rect', '/camera'),
            ('camera_info', '/camera_info'),
        ],
        parameters=[robot_path+"/config/apriltags.yaml"]
    )

    keyboard = Node(
        package='teleop_twist_keyboard',
        executable="teleop_twist_keyboard",
        output='screen',
        prefix = 'xterm -e'
    )

    # Load the SDF file from "description" package
    sdf_file  =  os.path.join(robot_path,
        'models', 'robot', 'model.sdf')
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'robot_description': robot_desc},
            {'use_sim_time': True},
        ]
    )
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(robot_path, 'config', 'ekf.yaml')],
    )
    
    odom_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments= ["0", "0", "0", "0", "0", "0", "odom", "base_link"]
        )
    map_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments= ["0", "0", "0", "0", "0", "0", "map", "odom"]
        )

    return LaunchDescription([
    	SetParameter(name='use_sim_time', value=True),
        SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=robot_path),
        gz_sim,
        bridge,
        apriltags,
        keyboard,
        robot_state_publisher,
        rviz,
        #odom_tf,
        ekf,
        #map_tf,
        slam,
        nav2,
        executive,
        getPose
    ])
