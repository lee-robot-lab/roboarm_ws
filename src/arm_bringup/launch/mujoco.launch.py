from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_xml = PathJoinSubstitution([FindPackageShare('arm_sim'), 'robot.xml'])

    return LaunchDescription([
        DeclareLaunchArgument('xml_path', default_value=default_xml),
        DeclareLaunchArgument('hz', default_value='200.0'),

        Node(
            package='arm_sim',
            executable='mujoco_mit_bridge',
            name='mujoco_mit_bridge',
            output='screen',
            parameters=[{
                'xml_path': LaunchConfiguration('xml_path'),
                'hz': LaunchConfiguration('hz'),
                'motor_ids': [1, 2, 3, 4],
                'joint_indices': [0, 1, 2, 3],
                'actuator_indices': [0, 1, 2, 3],
            }],
        ),
    ])
