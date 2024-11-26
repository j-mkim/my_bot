import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    pkg_share = get_package_share_directory('my_bot_description')
    default_model_path = os.path.join(pkg_share, 'description', 'robot.urdf.xacro') #xacro
    robot_description_config = Command(['xacro ', default_model_path, ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', use_sim_time])

    # RViz configuration file
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("my_bot_description"), "rviz", "rviz_test.rviz"]#"my_bot_description.rviz"]
    )

    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    channel_type =  LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200')
    frame_id = LaunchConfiguration('frame_id', default='base_link')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Sensitivity')

    return LaunchDescription([
        
        # Argument to load the URDF model
        DeclareLaunchArgument(
            name='model',
            default_value=default_model_path,
            description='Absolute path to robot urdf file'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description='Use ros2_control if true'),
        node_robot_state_publisher,
        

        # Node to visualize the robot in rviz
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', LaunchConfiguration('model')])
            }]
        ),

        # Joint State Publisher GUI node
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', LaunchConfiguration('model')])
            }]
        ),

        # Joint State Publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', LaunchConfiguration('model')])
            }]
        ),

        # RViz node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        ),

        # Node(
        #     package='slam_toolbox',
        #     executable='sync_slam_toolbox_node',
        #     name='slam_toolbox',
        #     output='screen',
        #     parameters=[
        #         {'use_sim_time': False},
        #         {'slam_toolbox_mapping.yaml'}  # Path to your config file
        #     ],
        #     remappings=[
        #         ('scan', '/scan')
        #     ]
        # ),

        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{'channel_type':channel_type,
                         'serial_port': serial_port, 
                         'serial_baudrate': serial_baudrate, 
                         'frame_id': frame_id,
                         'inverted': inverted, 
                         'angle_compensate': angle_compensate,
                         'scan_mode': scan_mode}],
            output='screen'),

        # Node(
        #     package='arduino_odometry',
        #     executable='arduino_odometry_node',
        #     name='arduino_odometry',
        #     parameters=[{'channel_type':'serial',
        #                  'serial_port':'/dev/ttyACM0',
        #                  'serial_baudrate':'9600'}],
        #     output='screen',
        #     # remappings=[('odom', '/')
        # ),

    ])
