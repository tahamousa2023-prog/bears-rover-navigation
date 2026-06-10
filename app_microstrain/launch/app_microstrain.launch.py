from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('app_microstrain'),
        'config', 'cv7_ahrs.yml'
    ])

    return LaunchDescription([
        Node(
            package='microstrain_inertial_driver',
            executable='microstrain_inertial_driver_node',
            name='microstrain_inertial_driver',
            output='screen',
            parameters=[config],
            remappings=[
                ('/imu/data', '/imu/data'),
                ('/imu/mag',  '/imu/mag'),
            ]
        )
    ])
