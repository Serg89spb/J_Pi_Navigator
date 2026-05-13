import os
from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import launch_ros.events.lifecycle
from lifecycle_msgs.msg import Transition

def generate_launch_description():
    package_name = 'jeweler_nav'
    pkg_share = get_package_share_directory(package_name)
    
    # 1. Пути к ресурсам пакета
    urdf_path = os.path.join(pkg_share, 'urdf', 'robot.urdf')
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    config_path = os.path.join(pkg_share, 'config', 'mapper_params.yaml')

    # 2. Настройка SLAM Toolbox (с авто-активацией)
    slam_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        namespace='',
        parameters=[config_path, {'use_sim_time': False}],
    )

    configure_event = RegisterEventHandler(
        OnProcessStart(
            target_action=slam_node,
            on_start=[
                EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(slam_node),
                    transition_id=Transition.TRANSITION_CONFIGURE,
                ))
            ],
        )
    )

    activate_event = RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=slam_node,
            start_state='configuring', goal_state='inactive',
            entities=[
                EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(slam_node),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                ))
            ],
        )
    )

    return LaunchDescription([
        # 1. Робот (состояние трансформ)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc, 'use_sim_time': False}],
            output='screen'
        ),

        # 2. Виртуальный лидар UE
        ExecuteProcess(
            cmd=['python3', os.path.join(pkg_share, 'scripts', 'ue_lidar_bridge.py')],
            output='screen'
        ),

        # 3. Виртуальная одометрия UE
        ExecuteProcess(
            cmd=['python3', os.path.join(pkg_share, 'scripts', 'ue_odom_bridge.py')],
            output='screen'
        ),

        # 4. Foxglove Bridge (универсальный путь)
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                os.path.join(get_package_share_directory('foxglove_bridge'), 'launch', 'foxglove_bridge_launch.xml')
            ),
            launch_arguments={'port': '8765'}.items()
        ),

        # 5. SLAM Toolbox
        slam_node,
        configure_event,
        activate_event,

        # 6. Автопилот Jeweler (C++)
        Node(
            package=package_name,
            executable='jeweler_navigator',
            name='jeweler_navigator',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),

        # 7. Зрение UE
        ExecuteProcess(
            cmd=['python3', os.path.join(pkg_share, 'scripts', 'ue_yolo_laser_node.py')],
            output='screen'
        ),

        # 8. Инспектор
        ExecuteProcess(
            cmd=['python3', os.path.join(pkg_share, 'scripts', 'robot_inspector.py')],
            output='screen'
        )
    ])
