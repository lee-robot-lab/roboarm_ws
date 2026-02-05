from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # robot.xml은 여전히 arm_sim 패키지 안에 있으므로 유지
    default_xml = PathJoinSubstitution([FindPackageShare('arm_sim'), 'robot.xml'])

    return LaunchDescription([
        DeclareLaunchArgument('xml_path', default_value=default_xml),
        DeclareLaunchArgument('hz', default_value='200.0'),

        Node(
            # [수정 1] 패키지 이름 변경: arm_sim -> arm_driver
            package='arm_driver',
            # [수정 2] 실행 파일 이름 변경: mujoco_mit_bridge -> mujoco_sim_driver
            executable='mujoco_sim_driver',
            name='mujoco_sim_driver',
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