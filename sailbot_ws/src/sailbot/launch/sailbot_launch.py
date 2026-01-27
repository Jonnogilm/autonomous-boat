from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sailbot',
            executable='bno055_i2c_node',
            name='bno055_i2c_node',
            output='screen'
        ),
        Node(
            package='sailbot',
            executable='wind_sensor_node',
            name='wind_sensor_node',
            output='screen'
        ),
        Node(
            package='sailbot',
            executable='actuators_node',
            name='actuators_node',
            output='screen'
        ),
        Node(
            package='sailbot',
            executable='control_node',
            name='control_node',
            output='screen'
        ),
        Node(
            package='sailbot',
            executable='mission_node',
            name='mission_node',
            output='screen'
        ),
        # Data Logger
        Node(
            package='sailbot',
            executable='data_logger.py',
            name='data_logger',
            output='screen'
        )
    ])
