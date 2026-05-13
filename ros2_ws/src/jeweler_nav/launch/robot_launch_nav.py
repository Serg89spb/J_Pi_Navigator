import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LifecycleNode
import launch.actions
import launch_ros.events.lifecycle
from lifecycle_msgs.msg import Transition

def generate_launch_description():
    # Имя нашего пакета
    package_name = 'jeweler_nav'
    
    # Динамически получаем путь к share-директории пакета
    pkg_share = get_package_share_directory(package_name)
    
    # Путь к URDF (теперь берем из папки urdf внутри пакета)
    urdf_path = os.path.join(pkg_share, 'urdf', 'robot.urdf')
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # Пути к внешним зависимостям (находим их автоматически)
    foxglove_bridge_share = get_package_share_directory('foxglove_bridge')
    slam_toolbox_share = get_package_share_directory('slam_toolbox')
    config_path = os.path.join(
        get_package_share_directory('jeweler_nav'),
        'config',
        'mapper_params.yaml'
    )

    slam_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        namespace='', # Пустой неймспейс, чтобы не добавился префикс к путям
        parameters=[
            config_path,
            {'use_sim_time': False}
        ],
    )

    # Создаем событие: как только нода стартует -> перейти в Configure
    configure_event = launch.actions.RegisterEventHandler(
        launch.event_handlers.OnProcessStart(
            target_action=slam_node,
            on_start=[
                launch.actions.EmitEvent(
                    event=launch_ros.events.lifecycle.ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(slam_node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    )
                ),
            ],
        )
    )

    # Создаем событие: как только нода сконфигурирована -> перейти в Activate
    activate_event = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=slam_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                launch.actions.EmitEvent(
                    event=launch_ros.events.lifecycle.ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(slam_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
    )

    return LaunchDescription([
        # 1. Робот (robot_state_publisher)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc, 'use_sim_time': False}],
            output='screen'
        ),

        # 2. Лидар
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 460800,
                'frame_id': 'laser_frame',
                'inverted': True,
                'angle_compensate': True,
                'scan_mode': 'Standard'
            }],
            output='screen'
        ),

        # 3. Мост с ESP (запускаем скрипт из папки scripts нашего пакета)
        ExecuteProcess(
            cmd=['python3', os.path.join(pkg_share, 'scripts', 'esp_bridge.py')],
            output='screen',
        ),

        # 4. Foxglove Bridge
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                os.path.join(foxglove_bridge_share, 'launch', 'foxglove_bridge_launch.xml')
            ),
            launch_arguments={'port': '8765'}.items()
        ),

        # 5. SLAM Toolbox
        slam_node,
        configure_event,
        activate_event,

        # 6. НОДА (Автопилот Jeweler - бинарник C++)
        Node(
            package=package_name,
            executable='jeweler_navigator',
            name='jeweler_navigator',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),

        # 7. Зрение Jeweler (скрипт из папки scripts)
        ExecuteProcess(
            cmd=['python3', os.path.join(pkg_share, 'scripts', 'yolo_laser_node.py')],
            output='screen'
        ),

        # 8. Инспектор (скрипт из папки scripts)
        ExecuteProcess(
            cmd=['python3', os.path.join(pkg_share, 'scripts', 'robot_inspector.py')],
            output='screen'
        )
    ])
