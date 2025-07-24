
 
import os, yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory

def generate_yaml_with_namespace(context, ranger_id):
    """Generate a yaml file for the controllers, including the namespace"""
    namespace = f"ranger_mini_{ranger_id}"
    original_yaml_path = os.path.join(
        FindPackageShare("ranger_mini").perform(context),
        "config",
        "ranger_mini_joint.yaml"
    )
    namespaced_yaml_path = f"/tmp/{namespace}_control.yaml"

    with open(original_yaml_path, 'r') as f:
        yaml_data = yaml.safe_load(f)

    namespaced_yaml = {}
    for key, value in yaml_data.items():
        namespaced_key = f"{namespace}/{key}"
        namespaced_yaml[namespaced_key] = value

    with open(namespaced_yaml_path, 'w') as f:
        yaml.dump(namespaced_yaml, f)

    return namespaced_yaml_path

def launch_setup(context):

    x = LaunchConfiguration('x_pose')
    y = LaunchConfiguration('y_pose')
    z = LaunchConfiguration('z_pose')

    ranger_id = int(LaunchConfiguration('ranger_id').perform(context))
    
    spawn_rangers_cmds = []

    namespace = f"ranger_mini_{ranger_id}"

    # Create a temporary configuration file for the controllers, adding the namespace in it
    generate_yaml_with_namespace(context, ranger_id)

    pkg_four_ws_control = get_package_share_directory('four_ws_control')

    # Path to the xacro file
    xacro_file = PathJoinSubstitution([
        FindPackageShare("ranger_mini"),
        "urdf",
        "ranger_mini_gazebo.xacro"
    ])

    # Robot's description (urdf generated from the xacro file, thanks to the command xacro)
    robot_description_config = ParameterValue(Command(['xacro', ' ', xacro_file, ' id:=', f'{ranger_id}']), value_type=str)
    
    start_robot_state_publisher_cmd = TimerAction( # Wait 3s before launching this node, which seems to help the controllers to load correctly
        period=3.0,
        actions=[
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                namespace=namespace,
                parameters=[{'robot_description': robot_description_config, 'use_sim_time': True, "frame_prefix": f'{namespace}/'}],
                output="screen",
            )
        ]
    )

    spawn_ranger_mini = Node(
        package="ros_gz_sim",
        executable="create",
        namespace=namespace,
        arguments=[
            '-name', f'{namespace}',
            '-topic', 'robot_description',
            '-x', x,
            '-y', y,
            '-z', z,
            '--ros-args', '--log-level', 'info'
        ],
        parameters=[{"use_sim_time": True}],
        output='screen'
    )

    # Bridge sensors topics from Gazebo to ROS2
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        arguments=[
            f'{namespace}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            f'{namespace}/imu_data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            f'{namespace}/navsat_data@gps_msgs/msg/GPSFix[gz.msgs.NavSat',
            f'{namespace}/color/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            f'{namespace}/color/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            f'{namespace}/depth/image_raw/image@sensor_msgs/msg/Image[gz.msgs.Image',
            f'{namespace}/depth/image_raw/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            f'{namespace}/depth/image_raw/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            f'{namespace}/depth/image_raw/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '--ros-args', '-p', 'expand_gz_topic_names:=true',
            '--log-level', 'info'
        ],
        parameters=[{"use_sim_time": True}],
        output='screen'
    )
 
    # Launch file allowing to control the ranger mini via velocity commands in the cmd_vel topic
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_four_ws_control, 'launch', 'four_ws_control.launch.py')
        ),
        launch_arguments={'namespace':namespace}.items(),
    )

    # Controllers

    # Warning : Unlike the Summit XL's controllers, these ones load very inconsistently, making Ranger Mini's integration difficult
    
    joint_state_broadcaster_spawner = Node(
            package="controller_manager",
            executable="spawner",
            namespace=namespace,
            arguments=["joint_state_broadcaster"],
            parameters=[{"use_sim_time": True}],
            output="screen"
        )

    forward_position_controller = Node(
            package="controller_manager",
            executable="spawner",
            namespace=namespace,
            arguments=["forward_position_controller"],
            parameters=[{"use_sim_time": True}],
        )


    forward_velocity_controller = Node(
            package="controller_manager",
            executable="spawner",
            namespace=namespace,
            arguments=["forward_velocity_controller"],
            parameters=[{"use_sim_time": True}],
        )
    
    # Camera controller, remove for now because of the instability of ranger's controllers
    ptz_camera_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=["ptz_camera_controller"],
        parameters=[{"use_sim_time": True}],
    )
    
    spawn_rangers_cmds.append(start_robot_state_publisher_cmd)
    spawn_rangers_cmds.append(spawn_ranger_mini)
    spawn_rangers_cmds.append(gz_bridge)
    spawn_rangers_cmds.append(controller)
    spawn_rangers_cmds.append(joint_state_broadcaster_spawner)
    spawn_rangers_cmds.append(forward_position_controller)
    spawn_rangers_cmds.append(forward_velocity_controller)
    # spawn_rangers_cmds.append(ptz_camera_controller_spawner)

    return spawn_rangers_cmds

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='2.0'),
        DeclareLaunchArgument('z_pose', default_value='1.0'),
        DeclareLaunchArgument('ranger_id', default_value='1'),
        OpaqueFunction(function=launch_setup)
    ])