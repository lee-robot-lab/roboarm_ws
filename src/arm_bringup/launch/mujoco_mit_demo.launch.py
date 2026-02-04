from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_xml = PathJoinSubstitution([FindPackageShare('arm_sim'), 'robot.xml'])

    xml_path = DeclareLaunchArgument('xml_path', default_value=default_xml)
    use_viewer = DeclareLaunchArgument('use_viewer', default_value='true')
    kp = DeclareLaunchArgument('kp', default_value='30.0')
    kd = DeclareLaunchArgument('kd', default_value='1.0')

    # 문자열로 받되, 노드에서 파싱하게 함
    q_targets = DeclareLaunchArgument('q_targets', default_value='[0.8, 0.0, 0.0, 0.0]')

    mujoco = Node(
        package='arm_sim',
        executable='mujoco_mit_bridge',
        name='mujoco_mit_bridge',
        output='screen',
        parameters=[{
            'xml_path': LaunchConfiguration('xml_path'),
            'use_viewer': LaunchConfiguration('use_viewer'),
            'publish_hz': 200.0,
            'motor_ids': [1, 2, 3, 4],
            'joint_indices': [0, 1, 2, 3],
            'actuator_indices': [0, 1, 2, 3],
        }]
    )

    control = Node(
        package='arm_control',
        executable='mit_hold',
        name='mit_hold',
        output='screen',
        parameters=[{
            'motor_ids': [1, 2, 3, 4],
            'q_targets': LaunchConfiguration('q_targets'),  # ✅ string
            'kp': LaunchConfiguration('kp'),
            'kd': LaunchConfiguration('kd'),
        }]
    )

    return LaunchDescription([
        xml_path, use_viewer, kp, kd, q_targets,
        mujoco,
        control,
    ])
