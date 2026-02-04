from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    xml_path = DeclareLaunchArgument(
        'xml_path',
        default_value='/home/su/roboarm_ws/src/arm_sim/robot.xml'
    )
    use_viewer = DeclareLaunchArgument('use_viewer', default_value='true')
    q_target = DeclareLaunchArgument('q_target', default_value='0.8')
    kp = DeclareLaunchArgument('kp', default_value='30.0')
    kd = DeclareLaunchArgument('kd', default_value='1.0')

    mujoco = Node(
        package='arm_sim',
        executable='mujoco_mit_bridge',
        name='mujoco_mit_bridge',
        output='screen',
        parameters=[{
            'xml_path': LaunchConfiguration('xml_path'),
            'use_viewer': LaunchConfiguration('use_viewer'),
            'motor_id': 1,
            'joint_index': 0,
            'actuator_index': 0,
            'publish_hz': 200.0,
        }]
    )

    control = Node(
        package='arm_control',
        executable='mit_hold',
        name='mit_hold',
        output='screen',
        parameters=[{
            'motor_id': 1,
            'q_target': LaunchConfiguration('q_target'),
            'kp': LaunchConfiguration('kp'),
            'kd': LaunchConfiguration('kd'),
        }]
    )

    return LaunchDescription([
        xml_path, use_viewer, q_target, kp, kd,
        mujoco,
        control,
    ])
