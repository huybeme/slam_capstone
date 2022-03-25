from ament_index_python import get_package_share_directory
import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../../..'))
    urdf_path = os.path.join(project_root, 'src/config/tb3_description.urdf')
    rviz_path = os.path.join(project_root, 'src/config/tb3_urdf.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    with open(urdf_path, 'r') as robot_xacro:
        robot_description = robot_xacro.read()

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}, {'use_sim_time': use_sim_time}]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )
    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='False',
                                            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=urdf_path,
                                            description='path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=rviz_path,
                                            description='path to rviz config file'),
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])