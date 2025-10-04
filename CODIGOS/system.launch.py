from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Sistema completo:
      - Evasión
      - Velocity Policy
      - Smooth
      - Safety (remapeado a /cmd_vel)
    """

    # Parámetros opcionales (puedes cambiarlos al lanzar)
    publish_rate = DeclareLaunchArgument(
        'publish_rate_hz', default_value='20.0',
        description='Frecuencia del nodo safety (Hz)'
    )
    stop_dist = DeclareLaunchArgument(
        'stop_distance_m', default_value='0.4',
        description='Distancia de parada total (m)'
    )
    slow_dist = DeclareLaunchArgument(
        'slow_distance_m', default_value='0.8',
        description='Distancia de inicio de reducción (m)'
    )

    # === Nodo 1: Evasión ===
    evasion = Node(
        package='cmd_evasion',
        executable='evasion_node',
        name='evasion',
        output='screen'
    )

    # === Nodo 2: Velocity Policy ===
    policy = Node(
        package='cmd_policy',
        executable='velocity_policy_node',
        name='velocity_policy',
        output='screen'
    )

    # === Nodo 3: Smooth ===
    smooth = Node(
        package='cmd_smooth',
        executable='smooth_node',
        name='smooth',
        output='screen'
    )

    # === Nodo 4: Safety ===
    safety = Node(
        package='cmd_safety',
        executable='safety_node',
        name='safety',
        output='screen',
        remappings=[
            ('/cmd_safety', '/cmd_vel')
        ],
        parameters=[
            {'publish_rate_hz': LaunchConfiguration('publish_rate_hz')},
            {'stop_distance_m': LaunchConfiguration('stop_distance_m')},
            {'slow_distance_m': LaunchConfiguration('slow_distance_m')}
        ]
    )

    return LaunchDescription([
        publish_rate,
        stop_dist,
        slow_dist,
        evasion,
        policy,
        smooth,
        safety
    ])
