from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # IMU publisher node
        Node(
            package='my_bot_imu',
            executable='my_bot_imu',
            name='my_bot_imu_node',
            output='screen',
            parameters=[{
                'frame_id': 'base_link',
                'i2c_device': '/dev/i2c-7',  # Update if needed
                'update_rate': 50
            }],
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=['/opt/ros/humble/share/robot_localization/params/ekf.yaml']
        ),
        # Static transform publisher (if dynamic transforms are not implemented)
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_transform_publisher',
        #     arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link'],
        # ),
        # RViz for visualization
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     output='screen',
        #     arguments=['-d', '/path/to/your/config.rviz'],  # Replace with your RViz config path
        # ),
    ])
