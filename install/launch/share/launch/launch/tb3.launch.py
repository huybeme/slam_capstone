# launch file created using a modified version of 

import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir

def genereate_launch_description():
    TURTLEBOT3_MODEL = "burger"
    LDS_MODEL = "LDS-01"
    LDS_LAUNCH_FILE = '/hlds_laser.launch.py'

    usb_port = LaunchConfiguration('usb_port', default='/dev/ttyACM0')

    tb3_param_dir = LaunchConfiguration(
        'tb3_param_dir',
        default=os.path.join(get_package_share_directory('turtlebot3_bringup'), 'param', 'burger.yaml')
    )

    lidar_pkg_dir = LaunchConfiguration(
        'lidar_pkg_dir',            # pkg_share_directory = /opt/ros/foxy/share/
        default=os.path.join(get_package_share_directory('hls_lfcd_lds_driver', 'launch'))
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'
        ),

        DeclareLaunchArgument(
            'usb_port',
            default_value=usb_port,
            description='Connected USB port with OPENCR'
        ),

        DeclareLaunchArgument(
            'tb3_param_dir',
            default_value=tb3_param_dir,
            description='Full path to turtlebot3 parameter file to load'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/tb3_state_publisher.launch.py']
            ),
            launch_arguments={'use_sim_time': use_sim_time}.itens(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([lidar_pkg_dir, LDS_LAUNCH_FILE]),
            launch_arguments={'port': '/dev/ttyUSB0', 'frame_id': 'base_Scan'}.items(),
        ),

        Node(
            package='',
            executeable='',
            parameters=[tb3_param_dir],
            arguments=['-i', usb_port],
            output='screen'
        ),
        

    ])