from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. 패키지 경로 찾기
    bringup_pkg = FindPackageShare('arm_bringup')
    planner_pkg = FindPackageShare('roboarm_planner') # 플래너 패키지 이름

    # 2. [Driver] MuJoCo 시뮬레이션 실행 (기존 launch 파일 재사용)
    #    이미 만들어둔 mujoco.launch.py를 포함(Include)시킵니다.
    mujoco_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([bringup_pkg, 'launch', 'mujoco.launch.py'])
        ]),
        launch_arguments={
            'hz': '200.0',  # 필요하면 파라미터 덮어쓰기 가능
        }.items()
    )

    # 3. [Adapter] MIT Bridge 실행
    mit_bridge_node = Node(
        package='roboarm_planner',
        executable='mit_bridge',
        name='mit_bridge',
        output='screen'
    )

    # 4. [Brain] IK Planner 실행
    #    (주의: Bridge가 켜진 후에 켜지면 좋겠지만, 동시에 켜져도 상관없습니다)
    ik_planner_node = Node(
        package='roboarm_planner',
        executable='ik_planner_node',
        name='ik_planner_node',
        output='screen',
        parameters=[{
            # 파라미터 설정이 필요하면 여기에 추가
            'traj_dt': 0.01,
            'traj_T': 3.0
        }]
    )

    return LaunchDescription([
        mujoco_launch,
        mit_bridge_node,
        ik_planner_node
    ])