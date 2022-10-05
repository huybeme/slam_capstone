import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    # occupancy_resolution = LaunchConfiguration('resolution', default='0.075')
    # publish_time = LaunchConfiguration('publish_period_sec', default='1.0')

    # tb3_cartographer_dir_prefix = get_package_share_directory(
    #     'turtlebot3_cartographer')
    # cartographer_config_dir = LaunchConfiguration(
    #     'cartographer_config_dir', default=os.path.join(tb3_cartographer_dir_prefix, 'config'))
    # config_basename = LaunchConfiguration(
    #     'configuration_basename', default='turtlebot3_lds_2d.lua')

    # rviz2_config_dir = '/home/hle/.rviz2/hl_030222.rviz'



    return LaunchDescription([

        Node(
            package='capstone_pkg',
            executable='tb3_status',
            name='tb3_status'
        ),

        Node(
            package='capstone_pkg',
            executable='circle_around',
            name='circle_around'
        ),

        Node(
            package='capstone_pkg',
            executable='robot_world',
            name='robot_world'
        ),

        # Node(
        #     package='cartographer_ros',
        #     executable='cartographer_node',
        #     name='cartographer_node',
        #     output='screen',
        #     parameters=[{'use_sim_time': use_sim_time}],
        #     arguments=['-configuration_directory', cartographer_config_dir,
        #                '-configuration_basename', config_basename]),

        # Node(
        #     package="cartographer_ros",
        #     executable="occupancy_grid_node",
        #     name="occupancy_grid_node",
        #     output='screen',
        #     parameters=[{'use_sim_time': use_sim_time}],
        #     arguments=['-resolution', occupancy_resolution,
        #                '-publish_period_sec', publish_time],
        #     # remap=[()],
        # ),

        # Node(
        #     package="rviz2",
        #     executable="rviz2",
        #     name="rviz2",
        #     arguments=['-d', rviz2_config_dir],
        #     parameters=[{'use_sim_time': use_sim_time}],
        #     output="screen",
        # ),

    ]) # end of LaunchDescription
