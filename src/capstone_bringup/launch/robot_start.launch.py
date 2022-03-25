from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import launch_ros
import launch
from ament_index_python import get_package_share_directory
import os


def generate_launch_description():

    project_root = os.path.abspath(os.path.join(
        os.path.dirname(__file__), '../../../../..'))
    rviz_path = os.path.join(project_root, 'src/config/tb3_urdf.rviz')

    tb3_yaml_dir = os.path.join(project_root, 'src', 'config', 'burger.yaml')
    lidar_dir = os.path.join(
        get_package_share_directory('hls_lfcd_lds_driver'), 'launch')
    lidar_launch = 'hlds_laser.launch.py'

    lidar_launch = LaunchConfiguration('lidar_dir', default=lidar_dir)
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    usb_port = LaunchConfiguration('usb_port', default="/dev/ttyACM0")
    tb3_launch = LaunchConfiguration('tb3_yaml_dir', default=tb3_yaml_dir)

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    return launch.LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='do not use simulation mode'
        ),
        DeclareLaunchArgument(
            'usb_port',
            default_value=usb_port,
            description='usb port of OpenCR'
        ),
        DeclareLaunchArgument(
            'tb3_yaml_dir',
            default_value=tb3_yaml_dir,
            description='tb3 burger yaml'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([lidar_dir, lidar_launch]),
            launch_arguments={'port': '/dev/ttyUSB0',
                              'frame_id': 'base_scan'}.items(),
        ),
        Node(
            package='turtlebot3_node',
            executable='turtlebot3_ros',
            parameters=[tb3_launch],
            arguments=['-i', usb_port],
            output='screen',
        )

    ])
