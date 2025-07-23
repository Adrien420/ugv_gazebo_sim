from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    namespace = LaunchConfiguration('namespace')

    fours_ws_control_cmd = Node(
        package='four_ws_control', 
        executable='robot_control.py', 
        namespace=namespace,
        parameters=[{"use_sim_time": True}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='ranger_mini_1'),
        fours_ws_control_cmd,
    ])