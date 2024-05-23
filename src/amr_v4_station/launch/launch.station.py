from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'station_name',
            default_value='station_default',
            description='Name of the station'
        ),
        Node(
            package='amr_v4_station',
            executable='station',
            name='station',
            output='screen',
            parameters=[{'station_name': LaunchConfiguration('station_name')}]
        )
    ])